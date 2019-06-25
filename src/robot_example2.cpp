// Copyright [2019] <Mateus Amarante>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <random>
#include <thread>  // NOLINT
#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"


class DummyRobotHW : public hardware_interface::RobotHW {
 public:
  explicit DummyRobotHW(ros::NodeHandle* nh) {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle("dummy_joint", &pos_, &vel_, &eff_);
    jnt_state_interface.registerHandle(state_handle);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle("dummy_joint"), &pos_cmd_);
    jnt_pos_interface.registerHandle(pos_handle);

    registerInterface(&jnt_pos_interface);

    nh_ = nh;
    white_noise_generator_ = std::normal_distribution<double>(0., 0.);
    white_noise_sub_ = nh_->subscribe("dummy_joint/white_noise", 1, &DummyRobotHW::white_noise_cb, this);
  }

  void read(const ros::Time& time, const ros::Duration& period) {
    pos_ = pos_cmd_ + white_noise_generator_(rnd_generator_);  // Simply copy command with white noise
    std::cout << pos_ << std::endl;
  }

  void write(const ros::Time& time, const ros::Duration& period) {}

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber white_noise_sub_;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  double pos_cmd_;
  double pos_;
  double vel_;
  double eff_;

  double noise_ = 0;
  std::default_random_engine rnd_generator_;
  std::normal_distribution<double> white_noise_generator_;

  void white_noise_cb(const std_msgs::Float64::ConstPtr& msg) {
    white_noise_generator_ = std::normal_distribution<double>(0., msg->data);
  }
};




void control_loop(const double frequency, hardware_interface::RobotHW* hw, controller_manager::ControllerManager* cm) {
  auto rate = ros::Rate(frequency);
  auto period = ros::Duration(1. / frequency);

  // Control loop
  while (ros::ok()) {
    hw->read(ros::Time::now(), period);
    cm->update(ros::Time::now(), period);
    hw->write(ros::Time::now(), period);
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_robot_hw_node");

  ros::NodeHandle nh;

  // RobotHW and ControllerManager initializations
  DummyRobotHW dummy_hw = DummyRobotHW(&nh);

  controller_manager::ControllerManager cm(&dummy_hw);

  // Control Loop
  double freq = 50;
  std::thread control_loop_thread(control_loop, freq, &dummy_hw, &cm);

  ros::spin();

  control_loop_thread.join();

  return 0;
}