/*
 * Copyright (c) 2020, SENAI CIMATEC
 */

#ifndef ROS_CONTROL_TESTS_SIMPLE_CONTROL_LOOP_HPP_
#define ROS_CONTROL_TESTS_SIMPLE_CONTROL_LOOP_HPP_

#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <thread>

namespace ros_control_tests {

void simple_control_loop(hardware_interface::RobotHW* robot_hw) {
  controller_manager::ControllerManager cm(robot_hw);

  ros::NodeHandle nh, private_nh("~");
  robot_hw->init(nh, private_nh);

  // AsyncSpinner for ROS interface
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(100.);

  while (ros::ok()) {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;

    robot_hw->read(time, period);
    cm.update(time, period);
    robot_hw->write(time, period);

    rate.sleep();
  }
}

}  // namespace ros_control_tests
#endif  // ROS_CONTROL_TESTS_SIMPLE_CONTROL_LOOP_HPP_
