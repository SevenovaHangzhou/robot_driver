#ifndef DM_SWERVE_DRIVER__ENCODER_SNAPSHOT_STORE_HPP_
#define DM_SWERVE_DRIVER__ENCODER_SNAPSHOT_STORE_HPP_

#include <array>
#include <filesystem>
#include <optional>
#include <stdexcept>

#include "dm_swerve_driver/external_steering_encoder.hpp"

namespace dm_swerve_driver {

using EncoderSnapshot = std::array<EncoderPersistentRecord, kSwerveModuleCount>;

class EncoderSnapshotError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

class EncoderSnapshotStore final {
public:
  explicit EncoderSnapshotStore(std::filesystem::path path);

  [[nodiscard]] std::optional<EncoderSnapshot> load() const;
  void save(const EncoderSnapshot & snapshot) const;

private:
  std::filesystem::path path_;
};

}  // namespace dm_swerve_driver

#endif  // DM_SWERVE_DRIVER__ENCODER_SNAPSHOT_STORE_HPP_
