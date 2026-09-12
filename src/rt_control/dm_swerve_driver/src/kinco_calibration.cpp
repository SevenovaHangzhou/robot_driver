#include "dm_swerve_driver/kinco_calibration.hpp"
#include "dm_swerve_driver/igh_ethercat_bus.hpp"
#include "dm_swerve_driver/kinco_control_loop.hpp"
#include "dm_swerve_driver/encoder_snapshot_store.hpp"
#include "atomic_text_file.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace dm_swerve_driver {
void load_kinco_calibration(DriverParameters & common, KincoParameters & parameters)
{
  const auto path = parameters.encoder_snapshot_path + ".calibration";
  if (!std::filesystem::exists(path)) {return;}
  std::ifstream input{path};
  std::string magic;
  unsigned int version{0U};
  if (!(input >> magic >> version) || magic != "KINCO_STEERING_CALIBRATION" || version != 1U) {
    throw std::runtime_error{"invalid Kinco calibration file"};
  }
  auto external_offsets = parameters.encoder_installation_offset_rad;
  auto motor_offsets = common.steering.zero_offset_rad;
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    std::size_t index{0U};
    std::uint32_t resolution{0U}, revolutions{0U}, ring{0U}, pinion{0U};
    int sign{0};
    if (!(input >> index >> external_offsets[i] >> motor_offsets[i] >>
      resolution >> revolutions >> ring >> pinion >> sign) || index != i ||
      !std::isfinite(external_offsets[i]) || !std::isfinite(motor_offsets[i]) ||
      resolution != parameters.encoder_expected_counts_per_revolution[i] ||
      revolutions != parameters.encoder_expected_distinguishable_revolutions[i] ||
      ring != parameters.encoder_ring_gear_teeth[i] || pinion != parameters.encoder_pinion_teeth[i] ||
      sign != parameters.encoder_direction[i])
    {throw std::runtime_error{"Kinco calibration does not match encoder/mechanical configuration"};}
  }
  if (input >> magic) {throw std::runtime_error{"unexpected calibration data"};}
  parameters.encoder_installation_offset_rad = external_offsets;
  common.steering.zero_offset_rad = motor_offsets;
}

void calibrate_kinco_steering(DriverParameters & common, KincoParameters & parameters)
{
  validate_kinco_parameters(parameters);
  auto next_common = common;
  auto next_parameters = parameters;
  KincoSwerveHardware motors{kinco_hardware_config(common, parameters), make_igh_ethercat_bus(parameters)};
  CanopenEncoderClient encoders{encoder_canopen_config(parameters), make_encoder_transport(parameters)};
  encoders.open();
  const auto info = encoders.read_hardware_info();
  encoders.configure_sync();
  if (!motors.prepare(false)) {throw std::runtime_error{"disabled motor calibration sampling failed"};}
  const auto motor_sample = motors.feedback();
  const auto sample = encoders.sample(std::chrono::steady_clock::now() +
    std::chrono::microseconds{parameters.encoder_feedback_deadline_us});
  if (!sample.complete()) {throw std::runtime_error{"all four encoders required for calibration"};}
  EncoderSnapshot snapshot{};
  std::ostringstream text;
  text << "KINCO_STEERING_CALIBRATION 1\n" << std::setprecision(17);
  for (std::size_t i{0U}; i < kSwerveModuleCount; ++i) {
    auto config = external_encoder_config(parameters, i);
    if (info[i].counts_per_revolution != config.expected_counts_per_revolution ||
      info[i].distinguishable_revolutions != config.expected_distinguishable_revolutions)
    {throw std::runtime_error{"encoder identity mismatch during calibration"};}
    config.installation_offset_rad = 0.0;
    const double offset = external_encoder_position_to_axis_angle(sample.positions[i], config);
    next_parameters.encoder_installation_offset_rad[i] = offset;
    next_common.steering.zero_offset_rad[i] += motor_sample.modules[i].motor_steering_angle_rad;
    snapshot[i] = {sample.positions[i], info[i]};
    text << i << ' ' << offset << ' ' << next_common.steering.zero_offset_rad[i] << ' ' <<
      info[i].counts_per_revolution << ' ' << info[i].distinguishable_revolutions << ' ' <<
      config.ring_gear_teeth << ' ' << config.pinion_teeth << ' ' << config.direction << '\n';
  }
  motors.stop();
  write_atomic_text(parameters.encoder_snapshot_path + ".calibration", text.str());
  EncoderSnapshotStore{parameters.encoder_snapshot_path}.save(snapshot);
  common = next_common;
  parameters = next_parameters;
}
}  // namespace dm_swerve_driver
