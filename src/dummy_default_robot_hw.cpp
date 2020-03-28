/*
 * Copyright (c) 2020, SENAI CIMATEC
 */

#include "ros_control_tests/dummy_default_robot_hw.hpp"

#include <ros/ros.h>

namespace ros_control_tests {

bool DummyDefaultRobotHW::initHW(const ros::NodeHandle &root_nh, const ros::NodeHandle &robot_hw_nh) {
  ROS_DEBUG("DummyDefaultRobotHW INIT");
  return true;
}

void DummyDefaultRobotHW::readHW(const ros::Time &time, const ros::Duration &period) {
  ROS_DEBUG("DummyDefaultRobotHW READ");
}

void DummyDefaultRobotHW::writeHW(const ros::Time &time, const ros::Duration &period) {
  ROS_DEBUG("DummyDefaultRobotHW WRITE");

  static double new_position, new_velocity, new_effort;

  for (const auto& act_data : this->getActuators()) {
    if (act_data->isClaimed()) {  // to be deprecated
      // default values
      new_position = act_data->getPosition();
      new_velocity = act_data->getVelocity();
      new_effort = act_data->getEffort();

      // check hardware interfaces
      switch (act_data->getHardwareInterface()) {
        case default_robot_hw_base::POSITION:
          new_position = act_data->getPositionCommand();                             // pos_cmd
          new_velocity = (new_position - act_data->getPosition()) / period.toSec();  // dpos/dt
          new_effort = (new_velocity - act_data->getVelocity()) / period.toSec();    // dvel/dt
          break;
        case default_robot_hw_base::VELOCITY:
          new_velocity = act_data->getVelocityCommand();                                // vel_cmd
          new_position = (new_velocity + act_data->getVelocity()) / 2 * period.toSec();  // vel*dt
          new_effort = (new_velocity - act_data->getVelocity()) / period.toSec();       // dvel/dt
          break;
        case default_robot_hw_base::EFFORT:
          new_effort = act_data->getEffortCommand();                                     // eff_cmd
          new_velocity = (new_effort + act_data->getEffort()) / 2 * period.toSec();      // eff*dt
          new_position = (new_velocity + act_data->getVelocity()) / 2 * period.toSec();  // vel*dt
          break;
        default:
          ROS_WARN("UNSUPPORTED HARDWARE INTERFACE");
          break;
      }

      // Update the robot state
      act_data->setPosition(new_position);
      act_data->setVelocity(new_velocity);
      act_data->setEffort(new_effort);
    }
  }
}

}  // namespace ros_control_tests
