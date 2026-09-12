#ifndef ROBOT_HW_ETHERCAT__ZEROERR_PP_SLAVE_HPP_
#define ROBOT_HW_ETHERCAT__ZEROERR_PP_SLAVE_HPP_

#include <array>
#include <memory>

#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "robot_hw_ethercat/pp_gripper.hpp"

namespace robot_hw_ethercat
{
class ZeroErrPpSlave final : public ethercat_generic_plugins::GenericEcSlave
{
public:
  bool setupSlave(std::unordered_map<std::string, std::string> parameters,
    std::vector<double> * states, std::vector<double> * commands) override;
  void processData(size_t index, uint8_t * address) override;
  void onPdoCycleRead(bool complete) override;
  void onPdoCycleStart(bool complete) override;
  void onPdoCycleSent() override;
  bool initialized() override {return preloaded_ && is_operational_;}
  void set_state_is_operational(bool operational) override;

private:
  uint16_t base_control_word() const noexcept;
  std::unique_ptr<PpGripper> gripper_;
  std::array<size_t, 5> commands_{};
  std::array<size_t, 6> states_{};
  PpFeedback feedback_{false, 0, 0, 0, 0, 0.0};
  PpOutput output_;
  double velocity_factor_{0.0}, effort_factor_{0.0};
  bool have_position_{false}, preloaded_{false}, complete_{false};
  unsigned int written_{0};
};
}  // namespace robot_hw_ethercat
#endif  // ROBOT_HW_ETHERCAT__ZEROERR_PP_SLAVE_HPP_
