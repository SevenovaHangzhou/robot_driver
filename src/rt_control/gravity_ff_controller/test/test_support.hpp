#ifndef GRAVITY_FF_CONTROLLER__TEST_SUPPORT_HPP_
#define GRAVITY_FF_CONTROLLER__TEST_SUPPORT_HPP_
#include "rclcpp/parameter.hpp"
#include <array>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <openssl/evp.h>
#include <sstream>
#include <string>
#include <vector>

inline std::string test_sha256(const std::string & text)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0U;
  auto context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(
    EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context ||
    EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), text.data(), text.size()) != 1 ||
    EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1)
  {
    return {};
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int i = 0U; i < size; ++i) {
    output << std::setw(2) << static_cast<int>(digest[i]);
  }
  return output.str();
}

inline std::vector<rclcpp::Parameter>
gravity_parameters(
  const std::string & urdf, const bool active,
  const bool verified = true)
{
  return {
    {"joints", std::vector<std::string>{"joint1", "joint2"}},
    {"mode", active ? "active" : "shadow"},
    {"scale", std::vector<double>{1.0, 1.0}},
    {"max_effort_nm", std::vector<double>{100.0, 100.0}},
    {"max_slew_nm_per_s", std::vector<double>{100.0, 100.0}},
    {"fault_slew_nm_per_s", 1000.0},
    {"robot_description", urdf},
    {"gravity", std::vector<double>{0.0, 0.0, -9.81}},
    {"input_inertia_g_mm2", std::vector<double>{1000.0, 2000.0}},
    {"reduction_ratio", std::vector<double>{10.0, 20.0}},
    {"payload_frame", "tool"},
    {"model_validation.verified", verified},
    {"model_validation.urdf_sha256",
      verified ? test_sha256(urdf) : std::string("TBD")},
    {"model_validation.source",
      verified ? std::string("synthetic fixture") : std::string("TBD")},
    {"effort_calibration.verified", std::vector<bool>{verified, verified}},
    {"effort_calibration.permille_per_newton_metre",
      std::vector<double>{2.0, 3.0}},
    {"effort_calibration.source",
      std::vector<std::string>{"synthetic", "synthetic"}},
    {"diagnostic_publish_rate", 10.0},
    {"diagnostic_topic", "~/state"},
  };
}
#endif // GRAVITY_FF_CONTROLLER__TEST_SUPPORT_HPP_
