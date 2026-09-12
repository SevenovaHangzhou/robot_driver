#include "dm_swerve_driver/swerve_driver_node.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "dm_swerve_driver/diagnostics.hpp"
#include "dm_swerve_driver/igh_ethercat_bus.hpp"
#include "dm_swerve_driver/imu_validation.hpp"
#include "dm_swerve_driver/kinco_calibration.hpp"
#include "dm_swerve_driver/kinco_control_loop.hpp"
#include "dm_swerve_driver/ros_params.hpp"
#include "ros_output.hpp"

namespace dm_swerve_driver {

class SwerveDriverNode::Impl {
public:
  explicit Impl(SwerveDriverNode & owner) : node_{owner} {}

  ~Impl() noexcept {stop_control();}

  [[nodiscard]] CallbackReturn configure()
  {
    try {
      declare_driver_parameters(node_);
      parameters_ = load_driver_parameters(node_);
      declare_kinco_parameters(node_);
      kinco_ = load_kinco_parameters(node_);
      load_kinco_calibration(parameters_, kinco_);
      odometry_publisher_ = node_.create_publisher<nav_msgs::msg::Odometry>("~/odom", 10U);
      joint_state_publisher_ =
        node_.create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10U);
      diagnostics_publisher_ =
        node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10U);
      create_subscriptions();
      create_services();
      diagnostics_timer_ = node_.create_wall_timer(
        std::chrono::seconds{1}, [this] {publish_diagnostics();});
      if (parameters_.odometry.publish_tf) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
      }
      configured_ = true;
      RCLCPP_INFO(node_.get_logger(), "swerve driver configured");
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_.get_logger(), "configuration failed: %s", error.what());
      static_cast<void>(cleanup());
      return CallbackReturn::FAILURE;
    } catch (...) {
      RCLCPP_ERROR(node_.get_logger(), "configuration failed with an unknown error");
      static_cast<void>(cleanup());
      return CallbackReturn::FAILURE;
    }
  }

  [[nodiscard]] CallbackReturn activate()
  {
    if (!configured_) {
      return CallbackReturn::FAILURE;
    }
    odometry_publisher_->on_activate();
    joint_state_publisher_->on_activate();
    try {
      if (!start_control()) {
        deactivate_publishers();
        return CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(node_.get_logger(), "swerve driver active");
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_.get_logger(), "activation failed: %s", error.what());
      stop_control();
      deactivate_publishers();
      return CallbackReturn::FAILURE;
    } catch (...) {
      RCLCPP_ERROR(node_.get_logger(), "activation failed with an unknown error");
      stop_control();
      deactivate_publishers();
      return CallbackReturn::FAILURE;
    }
  }

  [[nodiscard]] CallbackReturn deactivate()
  {
    stop_control();
    deactivate_publishers();
    RCLCPP_INFO(node_.get_logger(), "swerve driver inactive");
    return CallbackReturn::SUCCESS;
  }

  [[nodiscard]] CallbackReturn cleanup()
  {
    stop_control();
    cmd_subscription_.reset();
    imu_subscription_.reset();
    odometry_publisher_.reset();
    joint_state_publisher_.reset();
    diagnostics_publisher_.reset();
    diagnostics_timer_.reset();
    enable_service_.reset();
    disable_service_.reset();
    clear_faults_service_.reset();
    rezero_service_.reset();
    tf_broadcaster_.reset();
    configured_ = false;
    return CallbackReturn::SUCCESS;
  }

  [[nodiscard]] ControlLoopStatus control_status() const
  {
    return control_loop_ ? control_loop_->status() : last_status_;
  }

private:
  void create_subscriptions()
  {
    cmd_subscription_ = node_.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS{10U},
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        if (control_loop_) {
          control_loop_->submit_command(
            ChassisSpeeds{message->linear.x, message->linear.y, message->angular.z},
            std::chrono::steady_clock::now());
        }
      });
    imu_subscription_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      parameters_.odometry.imu_topic, rclcpp::SensorDataQoS{},
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        if (!control_loop_) {
          return;
        }
        if (!imu_orientation_valid(*message)) {
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000,
            "ignored IMU sample with unavailable or invalid orientation");
          return;
        }
        static_cast<void>(control_loop_->submit_imu_yaw(
            tf2::getYaw(message->orientation),
            std::chrono::steady_clock::now(),
            message->angular_velocity.z));
      });
  }

  void create_services()
  {
    using Trigger = std_srvs::srv::Trigger;
    enable_service_ = node_.create_service<Trigger>(
      "~/enable",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        handle_enable(response);
      });
    disable_service_ = node_.create_service<Trigger>(
      "~/disable",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        stop_control();
        response->success = true;
        response->message = "driver disabled";
      });
    clear_faults_service_ = node_.create_service<Trigger>(
      "~/clear_faults",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        if (!control_loop_ || !control_loop_->is_running()) {
          response->message = "driver must be enabled to clear faults";
          return;
        }
        control_loop_->request_clear_faults();
        response->success = true;
        response->message = "fault clear queued for control thread";
      });
    rezero_service_ = node_.create_service<Trigger>(
      "~/rezero_steering",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        if (control_loop_ && control_loop_->is_running()) {
          response->message = "disable the driver before steering rezero";
          return;
        }
        const auto result = rezero_steering();
        response->success = result.first;
        response->message = result.second;
      });
  }

  void handle_enable(const std_srvs::srv::Trigger::Response::SharedPtr & response)
  {
    try {
      if (!odometry_publisher_ || !odometry_publisher_->is_activated()) {
        response->message = "lifecycle node is not active";
        return;
      }
      response->success = start_control();
      response->message = response->success ? "driver enabled" : "driver enable failed";
    } catch (const std::exception & error) {
      stop_control();
      response->success = false;
      response->message = std::string{"driver enable failed: "} + error.what();
      RCLCPP_ERROR(node_.get_logger(), "%s", response->message.c_str());
    } catch (...) {
      stop_control();
      response->success = false;
      response->message = "driver enable failed with an unknown error";
      RCLCPP_ERROR(node_.get_logger(), "%s", response->message.c_str());
    }
  }

  [[nodiscard]] bool start_control()
  {
    if (control_loop_ && control_loop_->is_running()) {
      return true;
    }
    stop_control();
    control_loop_ = std::make_unique<KincoControlLoop>(
      parameters_, kinco_, make_igh_ethercat_bus(kinco_),
      make_encoder_transport(kinco_), control_callbacks());
    control_loop_->restore_fault_state(
      last_status_.fault_latched, last_status_.recovery_attempts);
    if (!control_loop_->initialize(std::chrono::steady_clock::now())) {
      stop_control();
      return false;
    }
    control_loop_->start();
    return true;
  }

  [[nodiscard]] std::pair<bool, std::string> rezero_steering() noexcept
  {
    try {
      calibrate_kinco_steering(parameters_, kinco_);
      return {true, "encoder installation offsets and motor reference saved; no hardware zero written"};
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_.get_logger(), "steering rezero failed: %s", error.what());
      return {false, std::string{"steering rezero failed: "} + error.what()};
    } catch (...) {
      RCLCPP_ERROR(node_.get_logger(), "steering rezero failed with an unknown error");
      return {false, "steering rezero failed with an unknown error"};
    }
  }

  void publish_diagnostics()
  {
    if (!diagnostics_publisher_) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node_.now();
    message.status = build_diagnostic_statuses(control_status());
    diagnostics_publisher_->publish(message);
  }

  [[nodiscard]] ControlLoopCallbacks control_callbacks()
  {
    return ControlLoopCallbacks{
      [this](const ControlLoopOutput & output) {publish_output(output);},
      [this](DriverLogLevel level, const std::string & message) {log(level, message);}};
  }

  void log(DriverLogLevel level, const std::string & message) const
  {
    switch (level) {
      case DriverLogLevel::debug:
        RCLCPP_DEBUG(node_.get_logger(), "%s", message.c_str());
        break;
      case DriverLogLevel::info:
        RCLCPP_INFO(node_.get_logger(), "%s", message.c_str());
        break;
      case DriverLogLevel::warning:
        RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
        break;
      case DriverLogLevel::error:
        RCLCPP_ERROR(node_.get_logger(), "%s", message.c_str());
        break;
    }
  }

  void publish_output(const ControlLoopOutput & output)
  {
    publish_control_output(
      node_, parameters_, output, odometry_publisher_, joint_state_publisher_,
      tf_broadcaster_.get());
  }

  void stop_control() noexcept
  {
    if (control_loop_) {
      control_loop_->stop();
      last_status_ = control_loop_->status();
      control_loop_.reset();
    }
  }

  void deactivate_publishers() noexcept
  {
    if (odometry_publisher_ && odometry_publisher_->is_activated()) {
      odometry_publisher_->on_deactivate();
    }
    if (joint_state_publisher_ && joint_state_publisher_->is_activated()) {
      joint_state_publisher_->on_deactivate();
    }
  }

  SwerveDriverNode & node_;
  DriverParameters parameters_{};
  std::unique_ptr<KincoControlLoop> control_loop_;
  KincoParameters kinco_{};
  ControlLoopStatus last_status_{};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_faults_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rezero_service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool configured_{false};
};

SwerveDriverNode::SwerveDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode{"swerve_driver", options},
  impl_{std::make_unique<Impl>(*this)}
{
}

SwerveDriverNode::~SwerveDriverNode() noexcept = default;

ControlLoopStatus SwerveDriverNode::control_status() const
{
  return impl_->control_status();
}

SwerveDriverNode::CallbackReturn SwerveDriverNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  return impl_->configure();
}

SwerveDriverNode::CallbackReturn SwerveDriverNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  return impl_->activate();
}

SwerveDriverNode::CallbackReturn SwerveDriverNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return impl_->deactivate();
}

SwerveDriverNode::CallbackReturn SwerveDriverNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return impl_->cleanup();
}

SwerveDriverNode::CallbackReturn SwerveDriverNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  static_cast<void>(impl_->deactivate());
  return CallbackReturn::SUCCESS;
}

}  // namespace dm_swerve_driver
