/*
 * Copyright (c) 2020, SENAI CIMATEC
 */

#include <thread>
#include "ros_control_tests/dummy_default_robot_hw.hpp"
#include "ros_control_tests/simple_control_loop.hpp"

using ros_control_tests::DummyDefaultRobotHW;
using controller_manager::ControllerManager;

int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_control_loop");

  ros_control_tests::DummyDefaultRobotHW robot_hw;

  ros_control_tests::simple_control_loop(&robot_hw);

  return 0;
}
