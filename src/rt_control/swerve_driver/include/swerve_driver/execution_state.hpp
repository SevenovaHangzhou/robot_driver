// Copyright 2026 robot_driver contributors
#ifndef SWERVE_DRIVER__EXECUTION_STATE_HPP_
#define SWERVE_DRIVER__EXECUTION_STATE_HPP_
#include <array>
#include <cstddef>
namespace swerve_driver {
// Experimental read-only state v1. SI units, control-axis signs, base_footprint.
// Transport: std_msgs/Float64MultiArray, /rt_control/chassis/execution_state.
// No commands, no ROS dependency, no allocation in the producer control cycle.
struct ExecutionStateWire {
  enum : std::size_t {
    Version=0, Stamp=1, Sequence=2, Ready=3, Status=4, Gated=5, Waiting=6,
    TranslationBranch=7, BodyVelocity=8, BodyAcceleration=11, MeasuredTwist=14,
    SteeringSpeed=17, SteeringAcceleration=18, SteeringJerk=19,
    BodyAccelerationLimit=20, BodyJerk=21, WheelAcceleration=22, WheelSpeed=23,
    Gate=24, Release=25, FlipHysteresis=26, Modules=27, Stride=16, Size=91
  };
  enum : std::size_t {
    Id=0, Angle=1, WheelVelocity=2, ReferenceAngle=3, ReferenceRate=4,
    ReferenceAcceleration=5, ReferenceWheelVelocity=6, Reversed=7, Holding=8,
    Goal=9, Error=10, Minimum=11, Maximum=12, Radius=13, X=14, Y=15
  };
  using Data=std::array<double, Size>;
};
}  // namespace swerve_driver
#endif
