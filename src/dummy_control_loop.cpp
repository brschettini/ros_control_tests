/*
 * Copyright (c) 2020, SENAI CIMATEC
 */

#include "ros_control_tests/dummy_default_robot_hw.hpp"
#include "ros_control_tests/simple_control_loop.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_control_loop");
  ros_control_tests::DummyDefaultRobotHW dummy_hw;

  ros_control_tests::simple_control_loop(&dummy_hw);

  return 0;
}