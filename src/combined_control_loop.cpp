/*
 * Copyright (c) 2020, SENAI CIMATEC
 */

#include "combined_robot_hw/combined_robot_hw.h"
#include "ros_control_tests/simple_control_loop.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_control_loop");

  combined_robot_hw::CombinedRobotHW combined_hw;

  ros::NodeHandle nh;
  combined_hw.init(nh, nh);

  ros_control_tests::simple_control_loop(&combined_hw);

  return 0;
}