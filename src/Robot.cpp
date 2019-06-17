#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>



class Robot : public hardware_interface::RobotHW {
  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double joint_cmd_[1];
    double joint_pos_[1];
    double joint_vel_[1];
    double joint_eff_[1];
  public:
    Robot() {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle("A", &joint_pos_[0], &joint_vel_[0], &joint_eff_[0]);
        jnt_state_interface.registerHandle(state_handle);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle("A"), &joint_cmd_[0]);
        jnt_pos_interface.registerHandle(pos_handle);

        registerInterface(&jnt_pos_interface);
    }

    void write() {
      std::cout << "This is a write method" << std::endl;
    }

    void read() {
      std::cout << "This is a read method" << std::endl;
      // This will update jnt_pos_, jnt_vel_ and jnt_eff_ variables (talking with the driver)
    }
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_test");

  Robot robot;
  controller_manager::ControllerManager cm(&robot);
  ros::Duration period(1.0);
  while (ros::ok()) {
    robot.read();
    cm.update(ros::Time::now(), ros::Duration(1.0));
    robot.write();
    std::cout << "This is a controller manager loop (not sure!)" << std::endl;
    sleep(1);
  }
  
  ros::spin();
}