#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "robot_hw_can/parameter_tool.hpp"

namespace robot_hw_can
{
namespace
{

TEST(ParameterToolTest, ParsesExplicitVolatileOrPersistentProfileWrites)
{
  const auto request = parse_parameter_request({
    "--can-interface", "can2", "--motor-id", "1", "--master-id", "17",
    "--acceleration-krad-s2", "0.025", "--deceleration-krad-s2", "-0.05",
    "--maximum-speed-rad-s", "20", "--confirm", "WRITE_DAMIAO_PARAMETERS"});
  EXPECT_EQ(request.can_interface, "can2");
  EXPECT_EQ(request.motor_id, 1U);
  EXPECT_EQ(request.master_id, 17U);
  EXPECT_TRUE(request.profile.valid());
  EXPECT_FALSE(request.save_to_flash);

  auto persistent = std::vector<std::string>{
    "--can-interface", "can2", "--motor-id", "2", "--master-id", "18",
    "--acceleration-krad-s2", "0.02", "--deceleration-krad-s2", "-0.04",
    "--maximum-speed-rad-s", "18", "--save", "--confirm",
    "WRITE_DAMIAO_PARAMETERS"};
  EXPECT_TRUE(parse_parameter_request(persistent).save_to_flash);
}

TEST(ParameterToolTest, RejectsMissingConfirmationInvalidSignsAndUnknownArguments)
{
  const std::vector<std::string> base{
    "--can-interface", "can2", "--motor-id", "1", "--master-id", "17",
    "--acceleration-krad-s2", "0.025", "--deceleration-krad-s2", "-0.05",
    "--maximum-speed-rad-s", "20"};
  EXPECT_THROW(static_cast<void>(parse_parameter_request(base)), std::invalid_argument);

  auto invalid_sign = base;
  invalid_sign.insert(
    invalid_sign.end(), {"--confirm", "WRITE_DAMIAO_PARAMETERS"});
  invalid_sign[9] = "0.05";
  EXPECT_THROW(
    static_cast<void>(parse_parameter_request(invalid_sign)), std::invalid_argument);

  auto unknown = invalid_sign;
  unknown[9] = "-0.05";
  unknown.push_back("--unexpected");
  EXPECT_THROW(static_cast<void>(parse_parameter_request(unknown)), std::invalid_argument);
}

}  // namespace
}  // namespace robot_hw_can
