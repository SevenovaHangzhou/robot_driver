#include "dm_swerve_driver/igh_ethercat_bus.hpp"

#include <ecrt.h>
#include <array>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

namespace dm_swerve_driver {
namespace {
using Clock = std::chrono::steady_clock;

void checked(int result, const char * operation)
{
  if (result < 0) {
    throw std::runtime_error{std::string{"IgH "} + operation + " failed: " +
            std::to_string(result)};
  }
}

// PDO fields need not be naturally aligned. Avoid typed dereferences of domain memory.
template<typename T>
T read_value(const std::uint8_t * address)
{
  std::uint32_t bits{0U};
  for (std::size_t i{0U}; i < sizeof(T); ++i) {
    bits |= static_cast<std::uint32_t>(address[i]) << (8U * i);
  }
  using Unsigned = std::make_unsigned_t<T>;
  const Unsigned narrowed{static_cast<Unsigned>(bits)};
  T value{};
  std::memcpy(&value, &narrowed, sizeof(T));
  return value;
}

template<typename T>
void write_value(std::uint8_t * address, T value)
{
  const auto bits = static_cast<std::make_unsigned_t<T>>(value);
  for (std::size_t i{0U}; i < sizeof(T); ++i) {
    address[i] = static_cast<std::uint8_t>((bits >> (8U * i)) & 0xFFU);
  }
}

struct AxisOffsets {
  unsigned int control{0U}, mode{0U}, target{0U};
  unsigned int status{0U}, mode_display{0U}, position{0U}, velocity{0U};
  unsigned int error{0U}, extended_error{0U};
};

class IghEthercatBus final : public KincoEthercatBus {
public:
  explicit IghEthercatBus(const KincoParameters & parameters) : parameters_{parameters}
  {
    validate_kinco_parameters(parameters_);
  }
  ~IghEthercatBus() override {IghEthercatBus::close();}
  IghEthercatBus(const IghEthercatBus &) = delete;
  IghEthercatBus & operator=(const IghEthercatBus &) = delete;

  void open() override
  {
    if (master_) {return;}
    master_ = ecrt_request_master(parameters_.ethercat_master_index);
    if (!master_) {throw std::runtime_error{"cannot reserve IgH EtherCAT master"};}
    try {
      domain_ = ecrt_master_create_domain(master_);
      if (!domain_) {throw std::runtime_error{"cannot create IgH process domain"};}
      for (std::size_t i{0U}; i < kKincoAxisCount; ++i) {
        ec_slave_info_t info{};
        checked(ecrt_master_get_slave(master_, parameters_.slave_positions[i], &info),
          "read slave identity");
        if (info.vendor_id != parameters_.vendor_id || info.product_code != parameters_.product_code) {
          throw std::runtime_error{"unexpected EtherCAT slave identity at axis " + std::to_string(i)};
        }
        slaves_[i] = ecrt_master_slave_config(master_, 0U, parameters_.slave_positions[i],
          parameters_.vendor_id, parameters_.product_code);
        if (!slaves_[i]) {throw std::runtime_error{"cannot configure EtherCAT slave"};}
        configure_axis(i);
      }
      checked(ecrt_master_select_reference_clock(master_, slaves_[0]), "select DC reference");
    } catch (...) {close(); throw;}
  }

  void configure_preop(const std::vector<KincoSdoWrite> & writes) override
  {
    if (!master_ || active_) {throw std::logic_error{"SDO configuration requires idle master"};}
    for (const auto & write : writes) {
      if (write.axis_index >= slaves_.size() || write.bit_length != 16U ||
        write.raw_value > 65535U)
      {
        throw std::invalid_argument{"unsupported Kinco startup SDO"};
      }
      checked(ecrt_slave_config_sdo16(slaves_[write.axis_index], write.index,
        write.subindex, static_cast<std::uint16_t>(write.raw_value)), "configure SDO16");
    }
  }

  void activate() override
  {
    if (!master_) {throw std::logic_error{"IgH master is not open"};}
    checked(ecrt_master_activate(master_), "activate");
    active_ = true;
    data_ = ecrt_domain_data(domain_);
    if (!data_) {throw std::runtime_error{"IgH domain has no process data"};}
    next_ = Clock::now();
    // Use a fixed wall-clock anchor so NTP changes cannot jump application time.
    monotonic_anchor_ = next_;
    constexpr std::int64_t ethercat_epoch_seconds{946684800};
    app_anchor_ns_ = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() -
      ethercat_epoch_seconds * 1000000000LL);
  }

  KincoEthercatCycle exchange(const KincoCommandBatch & commands) override
  {
    if (!active_ || !data_) {throw std::logic_error{"IgH domain is inactive"};}
    const auto now = Clock::now();
    checked(ecrt_master_application_time(master_, app_anchor_ns_ +
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
          now - monotonic_anchor_).count())), "application time");
    for (std::size_t i{0U}; i < commands.size(); ++i) {
      const auto & offset = offsets_[i];
      write_value(data_ + offset.control, commands[i].control_word);
      write_value(data_ + offset.mode, static_cast<std::int8_t>(commands[i].mode));
      write_value(data_ + offset.target, i < kSwerveModuleCount ?
        commands[i].target_position : commands[i].target_velocity);
    }
    checked(ecrt_master_sync_reference_clock(master_), "sync reference");
    checked(ecrt_master_sync_slave_clocks(master_), "sync slaves");
    checked(ecrt_domain_queue(domain_), "queue domain");
    checked(ecrt_master_send(master_), "send");
    next_ += std::chrono::nanoseconds{parameters_.dc_cycle_ns};
    if (next_ < now) {next_ = now + std::chrono::nanoseconds{parameters_.dc_cycle_ns};}
    std::this_thread::sleep_until(next_);
    checked(ecrt_master_receive(master_), "receive");
    checked(ecrt_domain_process(domain_), "process domain");
    ec_domain_state_t domain_state{};
    ec_master_state_t master_state{};
    checked(ecrt_domain_state(domain_, &domain_state), "domain state");
    checked(ecrt_master_state(master_, &master_state), "master state");
    KincoEthercatCycle result;
    // IgH computes expected WKC from its FMMU mapping. WC_COMPLETE is authoritative.
    result.domain = EthercatDomainStatus{domain_state.working_counter,
      domain_state.wc_state == EC_WC_COMPLETE ? domain_state.working_counter : 0U,
      master_state.link_up != 0U, true};
    for (std::size_t i{0U}; i < commands.size(); ++i) {
      ec_slave_config_state_t slave_state{};
      checked(ecrt_slave_config_state(slaves_[i], &slave_state), "slave state");
      result.domain.all_slaves_operational &=
        slave_state.operational != 0U && slave_state.al_state == 8U;
      const auto & offset = offsets_[i];
      auto & feedback = result.feedback[i];
      feedback.online = slave_state.online != 0U;
      feedback.status_word = read_value<std::uint16_t>(data_ + offset.status);
      feedback.mode_display = read_value<std::int8_t>(data_ + offset.mode_display);
      feedback.actual_position = read_value<std::int32_t>(data_ + offset.position);
      feedback.actual_velocity = read_value<std::int32_t>(data_ + offset.velocity);
      feedback.error_word = read_value<std::uint16_t>(data_ + offset.error);
      feedback.extended_error_word = read_value<std::uint16_t>(data_ + offset.extended_error);
    }
    return result;
  }

  void deactivate() noexcept override
  {
    if (master_ && active_) {static_cast<void>(ecrt_master_deactivate(master_));}
    active_ = false;
    data_ = nullptr;
  }
  void close() noexcept override
  {
    deactivate();
    if (master_) {ecrt_release_master(master_);}
    master_ = nullptr;
    domain_ = nullptr;
    slaves_.fill(nullptr);
  }
  bool is_open() const noexcept override {return master_ != nullptr;}

private:
  unsigned int register_entry(std::size_t axis, std::uint16_t index)
  {
    unsigned int bit{0U};
    const int offset = ecrt_slave_config_reg_pdo_entry(slaves_[axis], index, 0U, domain_, &bit);
    checked(offset, "register PDO entry");
    if (bit != 0U) {throw std::runtime_error{"Kinco PDO field is not byte aligned"};}
    return static_cast<unsigned int>(offset);
  }

  void configure_axis(std::size_t i)
  {
    const std::uint16_t target_index = i < kSwerveModuleCount ? 0x607AU : 0x60FFU;
    ec_pdo_entry_info_t rx[]{{0x6040U, 0U, 16U}, {0x6060U, 0U, 8U}, {target_index, 0U, 32U}};
    ec_pdo_entry_info_t tx[]{{0x6041U, 0U, 16U}, {0x6061U, 0U, 8U},
      {0x6064U, 0U, 32U}, {0x606CU, 0U, 32U}, {0x2601U, 0U, 16U}, {0x2602U, 0U, 16U}};
    ec_pdo_info_t pdos[]{{0x1601U, 3U, rx}, {0x1A01U, 6U, tx}};
    ec_sync_info_t syncs[]{
      {0U, EC_DIR_OUTPUT, 0U, nullptr, EC_WD_DISABLE},
      {1U, EC_DIR_INPUT, 0U, nullptr, EC_WD_DISABLE},
      {2U, EC_DIR_OUTPUT, 1U, &pdos[0], EC_WD_ENABLE},
      {3U, EC_DIR_INPUT, 1U, &pdos[1], EC_WD_DISABLE}};
    checked(ecrt_slave_config_pdos(slaves_[i], 4U, syncs), "configure PDO mapping");
    checked(ecrt_slave_config_dc(slaves_[i], 0x0300U,
      static_cast<std::uint32_t>(parameters_.dc_cycle_ns), 0, 0U, 0), "configure DC");
    // ESC watchdog period: (divider + 2) * 40 ns. 24998 makes a 1 ms tick.
    checked(ecrt_slave_config_watchdog(slaves_[i], 24998U,
      static_cast<std::uint16_t>(parameters_.pdo_watchdog_ms)), "configure PDO watchdog");
    auto & o = offsets_[i];
    o.control = register_entry(i, 0x6040U);
    o.mode = register_entry(i, 0x6060U);
    o.target = register_entry(i, target_index);
    o.status = register_entry(i, 0x6041U);
    o.mode_display = register_entry(i, 0x6061U);
    o.position = register_entry(i, 0x6064U);
    o.velocity = register_entry(i, 0x606CU);
    o.error = register_entry(i, 0x2601U);
    o.extended_error = register_entry(i, 0x2602U);
  }

  KincoParameters parameters_;
  ec_master_t * master_{nullptr};
  ec_domain_t * domain_{nullptr};
  std::array<ec_slave_config_t *, kKincoAxisCount> slaves_{};
  std::array<AxisOffsets, kKincoAxisCount> offsets_{};
  std::uint8_t * data_{nullptr};
  bool active_{false};
  Clock::time_point next_{}, monotonic_anchor_{};
  std::uint64_t app_anchor_ns_{0U};
};
}  // namespace

bool igh_backend_available() noexcept {return true;}
std::unique_ptr<KincoEthercatBus> make_igh_ethercat_bus(const KincoParameters & parameters)
{
  return std::make_unique<IghEthercatBus>(parameters);
}
}  // namespace dm_swerve_driver
