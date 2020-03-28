/*
 * Copyright (c) 2020, SENAI CIMATEC
 */

#ifndef ROS_CONTROL_TESTS_DUMMY_DEFAULT_ROBOT_HW_HPP_
#define ROS_CONTROL_TESTS_DUMMY_DEFAULT_ROBOT_HW_HPP_

#include <default_robot_hw_base/actuator_data.h>
#include <default_robot_hw_base/default_robot_hw_base.h>
#include <hardware_interface/robot_hw.h>

namespace ros_control_tests {

class DummyDefaultRobotHW : public default_robot_hw_base::DefaultRobotHWBase {
 public:
  virtual bool initHW(const ros::NodeHandle &root_nh, const ros::NodeHandle &robot_hw_nh);
  virtual void readHW(const ros::Time &time, const ros::Duration &period);
  virtual void writeHW(const ros::Time &time, const ros::Duration &period);
};

}  // namespace ros_control_tests
#endif  // ROS_CONTROL_TESTS_DUMMY_DEFAULT_ROBOT_HW_HPP_
