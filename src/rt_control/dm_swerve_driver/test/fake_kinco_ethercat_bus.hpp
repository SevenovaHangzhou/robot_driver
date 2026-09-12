#ifndef DM_SWERVE_DRIVER__TEST__FAKE_KINCO_ETHERCAT_BUS_HPP_
#define DM_SWERVE_DRIVER__TEST__FAKE_KINCO_ETHERCAT_BUS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "dm_swerve_driver/kinco_backend.hpp"

namespace dm_swerve_driver::test {

class FakeKincoEthercatBus final : public KincoEthercatBus {
public:
  FakeKincoEthercatBus()
  {
    for (std::size_t index{0U}; index < feedback_.size(); ++index) {
      feedback_[index].online = true;
      feedback_[index].status_word = 0x0040U;
      feedback_[index].actual_position = static_cast<std::int32_t>(1000U * (index + 1U));
    }
  }

  void open() override {open_ = true;}
  void configure_preop(const std::vector<KincoSdoWrite> & writes) override
  {
    preop_writes_ = writes;
  }
  void activate() override {active_ = true;}
  [[nodiscard]] KincoEthercatCycle exchange(const KincoCommandBatch & commands) override
  {
    batches_.push_back(commands);
    for (std::size_t index{0U}; index < feedback_.size(); ++index) {
      auto & feedback = feedback_[index];
      const auto & command = commands[index];
      feedback.mode_display = static_cast<std::int8_t>(command.mode);
      if (command.control_word == 0x0006U) {
        feedback.status_word = 0x0021U;
      } else if (command.control_word == 0x0007U) {
        feedback.status_word = 0x0023U;
      } else if (command.control_word == 0x000FU) {
        feedback.status_word = 0x0027U;
        if (index < kSwerveModuleCount) {
          feedback.actual_position = command.target_position;
        } else {
          feedback.actual_velocity = command.target_velocity;
          feedback.actual_position += command.target_velocity / 100;
        }
      } else if (command.control_word == 0x0000U) {
        feedback.status_word = 0x0040U;
        feedback.actual_velocity = 0;
      }
    }
    if (next_error_.has_value()) {
      feedback_[next_error_->first].error_word = next_error_->second;
      feedback_[next_error_->first].status_word = 0x0008U;
      next_error_.reset();
    }
    return KincoEthercatCycle{feedback_, domain_status_};
  }
  void deactivate() noexcept override {active_ = false;}
  void close() noexcept override {open_ = false;}
  [[nodiscard]] bool is_open() const noexcept override {return open_;}

  void set_domain_status(const EthercatDomainStatus & status) noexcept
  {
    domain_status_ = status;
  }
  void set_axis_online(std::size_t index, bool online) {feedback_.at(index).online = online;}
  void set_position(std::size_t index, std::int32_t position) {
    feedback_.at(index).actual_position = position;
  }
  void set_next_error(std::size_t index, std::uint16_t error_word)
  {
    next_error_ = std::pair<std::size_t, std::uint16_t>{index, error_word};
  }

  [[nodiscard]] bool active() const noexcept {return active_;}
  [[nodiscard]] const std::vector<KincoSdoWrite> & preop_writes() const noexcept
  {
    return preop_writes_;
  }
  [[nodiscard]] const std::vector<KincoCommandBatch> & batches() const noexcept
  {
    return batches_;
  }

private:
  KincoFeedbackBatch feedback_{};
  EthercatDomainStatus domain_status_{16U, 16U, true, true};
  std::vector<KincoSdoWrite> preop_writes_;
  std::vector<KincoCommandBatch> batches_;
  std::optional<std::pair<std::size_t, std::uint16_t>> next_error_;
  bool open_{false};
  bool active_{false};
};

}  // namespace dm_swerve_driver::test

#endif  // DM_SWERVE_DRIVER__TEST__FAKE_KINCO_ETHERCAT_BUS_HPP_
