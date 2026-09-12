#include <gtest/gtest.h>
#include <algorithm>
#include <deque>
#include <filesystem>
#include <cstdlib>
#include "dm_swerve_driver/kinco_control_loop.hpp"
#include "dm_swerve_driver/encoder_snapshot_store.hpp"
#include "fake_kinco_ethercat_bus.hpp"

namespace dm_swerve_driver {
namespace {
class Encoders final : public CanTransport {
public:
  bool opened{false}, drop{false};
  std::uint32_t position{0U};
  std::deque<ReceivedCanFrame> queue;
  void open() override {opened = true;}
  void close() noexcept override {opened = false;}
  bool is_open() const noexcept override {return opened;}
  void write_batch(const std::vector<CanFrame> & frames) override
  {
    for (const auto & frame : frames) {
      if (frame.id == 0x80U) {
        for (std::uint16_t node{1U}; node <= 4U; ++node) {
          if (drop && node == 3U) {continue;}
          CanFrame reply{static_cast<std::uint16_t>(0x280U + node), 4U, {}};
          for (unsigned int b{0U}; b < 4U; ++b) {
            reply.data[b] = static_cast<std::uint8_t>((position >> (8U * b)) & 0xFFU);
          }
          queue.push_back({reply, {}});
        }
      } else if (frame.id > 0x600U) {
        auto reply = frame;
        reply.id -= 0x80U;
        if (frame.data[0] == 0x40U) {
          const unsigned int index = frame.data[1] + (frame.data[2] << 8U);
          const std::uint32_t value = index == 0x6501U ? 10000U :
            index == 0x6502U ? 24U : index == 0x6000U ? 0U :
            index == 0x1A01U && frame.data[3] == 1U ? 0x60040020U : 1U;
          reply.data[0] = 0x43U;
          for (unsigned int b{0U}; b < 4U; ++b) {
            reply.data[b + 4U] = static_cast<std::uint8_t>((value >> (8U * b)) & 0xFFU);
          }
        } else {reply.data[0] = 0x60U;}
        queue.push_back({reply, {}});
      }
    }
  }
  std::vector<ReceivedCanFrame> collect(std::size_t count,
    std::chrono::steady_clock::time_point) override
  {
    std::vector<ReceivedCanFrame> result;
    while (count-- > 0U && !queue.empty()) {result.push_back(queue.front()); queue.pop_front();}
    return result;
  }
};

TEST(KincoRuntimeSmoke, EncoderGatePrecedesEnableAndBackupKeepsCyclesAlive)
{
  char pattern[]{"/tmp/dm-swerve-runtime-XXXXXX"};
  const char * created = ::mkdtemp(pattern);
  ASSERT_NE(created, nullptr);
  const std::filesystem::path directory{created};
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {std::error_code error; std::filesystem::remove_all(path, error);}
  } cleanup{directory};
  auto common = default_parameters();
  KincoParameters parameters;
  parameters.dc_cycle_ns = 10000000;
  parameters.pdo_watchdog_ms = 100;
  parameters.steering_encoder_resolution.fill(10000U);
  parameters.drive_encoder_resolution.fill(10000U);
  parameters.encoder_expected_counts_per_revolution.fill(10000U);
  parameters.encoder_expected_distinguishable_revolutions.fill(24U);
  parameters.encoder_ring_gear_teeth.fill(2U);
  parameters.encoder_pinion_teeth.fill(1U);
  parameters.encoder_source_disagreement_threshold_rad = 0.1;
  parameters.encoder_maximum_offline_axis_motion_rad = 1.0;
  parameters.encoder_maximum_rejoin_correction_rad = 0.01;
  parameters.encoder_snapshot_path = (directory / "snapshot").string();
  EncoderSnapshot snapshot{};
  snapshot.fill(EncoderPersistentRecord{0U, {10000U, 24U}});
  EncoderSnapshotStore{parameters.encoder_snapshot_path}.save(snapshot);
  const auto now = std::chrono::steady_clock::time_point{};

  {
    auto bus = std::make_unique<test::FakeKincoEthercatBus>();
    auto * observed = bus.get();
    for (std::size_t i{0U}; i < 4U; ++i) {observed->set_position(i, 0);}
    auto encoders = std::make_unique<Encoders>();
    encoders->position = 1000U;  // External 0.314 rad, motor zero: must reject before enable.
    KincoControlLoop rejected{common, parameters, std::move(bus), std::move(encoders)};
    EXPECT_FALSE(rejected.initialize(now));
    for (const auto & batch : observed->batches()) {
      for (const auto & axis : batch) {EXPECT_NE(axis.control_word, 0x000FU);}
    }
  }
  auto bus = std::make_unique<test::FakeKincoEthercatBus>();
  auto * observed = bus.get();
  for (std::size_t i{0U}; i < 4U; ++i) {observed->set_position(i, 0);}
  auto encoders = std::make_unique<Encoders>();
  auto * external = encoders.get();
  KincoControlLoop loop{common, parameters, std::move(bus), std::move(encoders)};
  ASSERT_TRUE(loop.initialize(now));
  const auto batches = observed->batches().size();
  loop.submit_command({0.2, 0.0, 0.0}, now);
  ASSERT_TRUE(loop.step(now + std::chrono::milliseconds{10}));
  EXPECT_EQ(observed->batches().size(), batches + 1U);
  EXPECT_GT(observed->batches().back()[4].target_velocity, 0);
  external->drop = true;
  ASSERT_TRUE(loop.step(now + std::chrono::milliseconds{20}));
  EXPECT_EQ(loop.status().steering_sources[2].source, SteeringAngleSource::motor_backup);
  EXPECT_FALSE(loop.status().fault_latched);
  observed->set_next_error(5U, 1U << 7U);
  ASSERT_TRUE(loop.step(now + std::chrono::milliseconds{30}));
  EXPECT_TRUE(loop.status().fault_latched);
  ASSERT_TRUE(loop.step(now + std::chrono::milliseconds{40}));
  for (std::size_t i{4U}; i < 8U; ++i) {
    EXPECT_EQ(observed->batches().back()[i].target_velocity, 0);
  }
}
}
}
