#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "rolling_trajectory_controller/envelope_loader.hpp"
#include "rolling_trajectory_controller/limit_checker.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{
namespace
{

const std::filesystem::path kProvisionalEnvelopePath{
  ROLLING_PROVISIONAL_ENVELOPE_PATH};

std::string readText(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

class TemporaryEnvelope
{
public:
  explicit TemporaryEnvelope(const std::string & contents)
  {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("e102-envelope-loader-" + std::to_string(nonce) + ".yaml");
    std::ofstream stream(path_, std::ios::binary | std::ios::trunc);
    stream << contents;
    stream.close();
  }

  ~TemporaryEnvelope()
  {
    std::error_code error;
    (void)std::filesystem::remove(path_, error);
  }

  TemporaryEnvelope(const TemporaryEnvelope &) = delete;
  TemporaryEnvelope & operator=(const TemporaryEnvelope &) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept
  {
    return path_;
  }

private:
  std::filesystem::path path_{};
};

std::string versionHex(const std::array<std::uint8_t, 32> & version)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const std::uint8_t value : version) {
    stream << std::setw(2) << static_cast<unsigned int>(value);
  }
  return stream.str();
}

bool replaceFirst(
  std::string & contents, const std::string & old_value,
  const std::string & new_value)
{
  const std::size_t offset = contents.find(old_value);
  if (offset == std::string::npos) {
    return false;
  }
  contents.replace(offset, old_value.size(), new_value);
  return true;
}

bool removeFirstField(std::string & contents, const std::string & field)
{
  const std::string marker = "    " + field + ":";
  const std::size_t begin = contents.find(marker);
  if (begin == std::string::npos) {
    return false;
  }
  const std::size_t end = contents.find('\n', begin);
  if (end == std::string::npos) {
    return false;
  }
  contents.erase(begin, end - begin + 1U);
  return true;
}

TEST(EnvelopeLoader, LoadsExactFourteenAxisProvisionalArtifactAndHashesRawBytes)
{
  DynamicEnvelope envelope;
  std::string error;
  ASSERT_TRUE(loadProvisionalEnvelope(kProvisionalEnvelopePath.string(), envelope, error))
    << error;
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(envelope.source, LimitsSource::kProvisional);
  EXPECT_EQ(
    versionHex(envelope.limits_version),
    "e355f72990a1c73b62d591f733cd4d2e743b78298f541dc0e92ce0ec16ccd0c4");

  EXPECT_DOUBLE_EQ(envelope.axes[0].position_lower, -1.57079632679);
  EXPECT_DOUBLE_EQ(envelope.axes[3].position_upper, 3.14159265);
  EXPECT_DOUBLE_EQ(envelope.axes[12].velocity_positive, 0.2617993877991494);
  EXPECT_DOUBLE_EQ(envelope.axes[13].position_lower, 0.0);
  EXPECT_DOUBLE_EQ(envelope.axes[13].position_upper, 0.8);
  EXPECT_DOUBLE_EQ(envelope.axes[13].velocity_positive, 0.09);

  LimitChecker checker;
  EXPECT_TRUE(checker.configure(envelope, false, true));
}

TEST(EnvelopeLoader, RejectsEveryMissingAxisFieldWithoutFallback)
{
  const std::vector<std::string> fields = {
    "position_lower", "position_upper", "velocity_positive", "velocity_negative",
    "acceleration_positive", "acceleration_negative",
    "stop_acceleration_positive", "stop_acceleration_negative",
    "position_margin_lower", "position_margin_upper"};
  const std::string valid = readText(kProvisionalEnvelopePath);

  for (const std::string & field : fields) {
    std::string invalid = valid;
    ASSERT_TRUE(removeFirstField(invalid, field)) << field;
    TemporaryEnvelope file(invalid);
    DynamicEnvelope envelope;
    envelope.source = LimitsSource::kProduction;
    envelope.limits_version.fill(0xffU);
    std::string error;
    EXPECT_FALSE(loadProvisionalEnvelope(file.path().string(), envelope, error)) << field;
    EXPECT_EQ(envelope.source, LimitsSource::kUnspecified) << field;
    EXPECT_EQ(
      envelope.limits_version, (std::array<std::uint8_t, 32>{})) << field;
    EXPECT_FALSE(error.empty()) << field;
  }
}

TEST(EnvelopeLoader, RejectsWrongSourceAxisOrderUnknownFieldsAndInvalidNumbers)
{
  const std::string valid = readText(kProvisionalEnvelopePath);
  const std::vector<std::pair<std::string, std::string>> mutations = {
    {"limits_source: provisional", "limits_source: test_only"},
    {"name: right_joint1", "name: left_joint1"},
    {"    position_lower: -1.57079632679",
      "    unexpected_limit: 1.0\n    position_lower: -1.57079632679"},
    {"velocity_positive: 0.2617993877991494", "velocity_positive: 0.0"},
    {"acceleration_positive: 0.75", "acceleration_positive: .nan"},
    {"position_margin_lower: 0.008726646259971648", "position_margin_lower: -0.1"}};

  for (const auto & mutation : mutations) {
    std::string invalid = valid;
    ASSERT_TRUE(replaceFirst(invalid, mutation.first, mutation.second));
    TemporaryEnvelope file(invalid);
    DynamicEnvelope envelope;
    std::string error;
    EXPECT_FALSE(loadProvisionalEnvelope(file.path().string(), envelope, error))
      << mutation.first;
    EXPECT_EQ(envelope.source, LimitsSource::kUnspecified) << mutation.first;
    EXPECT_FALSE(error.empty()) << mutation.first;
  }
}

TEST(EnvelopeLoader, MissingFileFailsClosed)
{
  DynamicEnvelope envelope;
  envelope.source = LimitsSource::kProduction;
  std::string error;
  EXPECT_FALSE(loadProvisionalEnvelope(
      "/tmp/e102-envelope-that-does-not-exist.yaml", envelope, error));
  EXPECT_EQ(envelope.source, LimitsSource::kUnspecified);
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace rolling_trajectory_controller
