#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "dm_swerve_driver/encoder_snapshot_store.hpp"

namespace dm_swerve_driver {
namespace {

class TemporarySnapshot final {
public:
  TemporarySnapshot()
  : directory_{std::filesystem::temp_directory_path() /
      ("dm-swerve-encoder-test-" + std::to_string(::getpid()))},
    path_{directory_ / "snapshot.txt"}
  {
    std::filesystem::remove_all(directory_);
    std::filesystem::create_directories(directory_);
  }

  ~TemporarySnapshot() {std::filesystem::remove_all(directory_);}

  TemporarySnapshot(const TemporarySnapshot &) = delete;
  TemporarySnapshot & operator=(const TemporarySnapshot &) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept {return path_;}

private:
  std::filesystem::path directory_;
  std::filesystem::path path_;
};

[[nodiscard]] EncoderSnapshot snapshot()
{
  EncoderSnapshot result{};
  for (std::size_t index{0U}; index < result.size(); ++index) {
    result[index] = EncoderPersistentRecord{
      static_cast<std::uint32_t>(1000U + index),
      EncoderHardwareInfo{65536U, 24U}};
  }
  return result;
}

TEST(EncoderSnapshotStoreTest, MissingSnapshotIsExplicit)
{
  TemporarySnapshot temporary;
  const EncoderSnapshotStore store{temporary.path()};
  EXPECT_FALSE(store.load().has_value());
}

TEST(EncoderSnapshotStoreTest, AtomicallyRoundTripsAllFourRecords)
{
  TemporarySnapshot temporary;
  const EncoderSnapshotStore store{temporary.path()};
  const auto expected = snapshot();

  store.save(expected);
  const auto actual = store.load();

  ASSERT_TRUE(actual.has_value());
  EXPECT_EQ(*actual, expected);
  EXPECT_FALSE(std::filesystem::exists(temporary.path().string() + ".tmp"));
}

TEST(EncoderSnapshotStoreTest, RejectsCorruptOrTruncatedSnapshot)
{
  TemporarySnapshot temporary;
  {
    std::ofstream output{temporary.path()};
    output << "DM_SWERVE_ENCODER_SNAPSHOT 1\n0 100 65536 24\n";
  }
  const EncoderSnapshotStore store{temporary.path()};
  EXPECT_THROW(static_cast<void>(store.load()), EncoderSnapshotError);
}

}  // namespace
}  // namespace dm_swerve_driver
