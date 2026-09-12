#include "dm_swerve_driver/encoder_snapshot_store.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <unistd.h>

namespace dm_swerve_driver {
namespace {

constexpr const char * kSnapshotMagic{"DM_SWERVE_ENCODER_SNAPSHOT"};
constexpr unsigned int kSnapshotVersion{1U};
constexpr std::uint64_t kFnvOffset{14695981039346656037ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

class FileDescriptor final {
public:
  explicit FileDescriptor(int descriptor) noexcept : descriptor_{descriptor} {}
  ~FileDescriptor()
  {
    if (descriptor_ >= 0) {
      static_cast<void>(::close(descriptor_));
    }
  }
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;
  [[nodiscard]] int get() const noexcept {return descriptor_;}

private:
  int descriptor_{-1};
};

void mix(std::uint64_t & checksum, std::uint32_t value) noexcept
{
  for (unsigned int byte{0U}; byte < 4U; ++byte) {
    checksum ^= static_cast<std::uint8_t>((value >> (8U * byte)) & 0xFFU);
    checksum *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t checksum(const EncoderSnapshot & snapshot) noexcept
{
  std::uint64_t result{kFnvOffset};
  for (const auto & record : snapshot) {
    mix(result, record.position);
    mix(result, record.hardware.counts_per_revolution);
    mix(result, record.hardware.distinguishable_revolutions);
  }
  return result;
}

void validate_snapshot(const EncoderSnapshot & snapshot)
{
  for (const auto & record : snapshot) {
    const std::uint64_t total =
      static_cast<std::uint64_t>(record.hardware.counts_per_revolution) *
      static_cast<std::uint64_t>(record.hardware.distinguishable_revolutions);
    if (record.hardware.counts_per_revolution == 0U ||
      record.hardware.distinguishable_revolutions == 0U ||
      static_cast<std::uint64_t>(record.position) >= total)
    {
      throw EncoderSnapshotError{"invalid encoder snapshot record"};
    }
  }
}

void sync_path(const std::filesystem::path & path, int flags)
{
  const FileDescriptor descriptor{::open(path.c_str(), flags)};
  if (descriptor.get() < 0) {
    throw EncoderSnapshotError{
            "failed to open snapshot path for sync: " + std::string{std::strerror(errno)}};
  }
  if (::fsync(descriptor.get()) != 0) {
    throw EncoderSnapshotError{
            "failed to sync encoder snapshot: " + std::string{std::strerror(errno)}};
  }
}

[[nodiscard]] EncoderSnapshot parse_snapshot(std::istream & input)
{
  std::string magic;
  unsigned int version{0U};
  if (!(input >> magic >> version) || magic != kSnapshotMagic || version != kSnapshotVersion) {
    throw EncoderSnapshotError{"invalid encoder snapshot header"};
  }
  EncoderSnapshot result{};
  for (std::size_t expected_index{0U}; expected_index < result.size(); ++expected_index) {
    std::size_t index{0U};
    auto & record = result[expected_index];
    if (!(input >> index >> record.position >> record.hardware.counts_per_revolution >>
      record.hardware.distinguishable_revolutions) || index != expected_index)
    {
      throw EncoderSnapshotError{"invalid encoder snapshot record"};
    }
  }
  std::string checksum_label;
  std::uint64_t stored_checksum{0U};
  if (!(input >> checksum_label >> stored_checksum) || checksum_label != "CHECKSUM" ||
    stored_checksum != checksum(result))
  {
    throw EncoderSnapshotError{"encoder snapshot checksum mismatch"};
  }
  std::string trailing;
  if (input >> trailing) {
    throw EncoderSnapshotError{"unexpected trailing encoder snapshot data"};
  }
  validate_snapshot(result);
  return result;
}

}  // namespace

EncoderSnapshotStore::EncoderSnapshotStore(std::filesystem::path path)
: path_{std::move(path)}
{
  if (path_.empty()) {
    throw std::invalid_argument{"encoder snapshot path must not be empty"};
  }
}

std::optional<EncoderSnapshot> EncoderSnapshotStore::load() const
{
  std::ifstream input{path_};
  if (!input.is_open()) {
    if (!std::filesystem::exists(path_)) {
      return std::nullopt;
    }
    throw EncoderSnapshotError{"failed to open encoder snapshot"};
  }
  return parse_snapshot(input);
}

void EncoderSnapshotStore::save(const EncoderSnapshot & snapshot) const
{
  validate_snapshot(snapshot);
  const std::filesystem::path temporary{path_.string() + ".tmp"};
  try {
    std::ofstream output{temporary, std::ios::trunc};
    if (!output.is_open()) {
      throw EncoderSnapshotError{"failed to create temporary encoder snapshot"};
    }
    output << kSnapshotMagic << ' ' << kSnapshotVersion << '\n';
    for (std::size_t index{0U}; index < snapshot.size(); ++index) {
      const auto & record = snapshot[index];
      output << index << ' ' << record.position << ' ' <<
        record.hardware.counts_per_revolution << ' ' <<
        record.hardware.distinguishable_revolutions << '\n';
    }
    output << "CHECKSUM " << checksum(snapshot) << '\n';
    output.flush();
    if (!output.good()) {
      throw EncoderSnapshotError{"failed to write encoder snapshot"};
    }
    output.close();
    sync_path(temporary, O_RDONLY);
    std::filesystem::rename(temporary, path_);
    const auto parent = path_.parent_path().empty() ? std::filesystem::path{"."} :
      path_.parent_path();
    sync_path(parent, O_RDONLY | O_DIRECTORY);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}

}  // namespace dm_swerve_driver
