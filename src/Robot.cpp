#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


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
        hardware_interface::JointStateHandle state_handle("Maxon", &joint_pos_[0], &joint_vel_[0], &joint_eff_[0]);
        jnt_state_interface.registerHandle(&state_handle);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        hardware_interface::PositionJointInterface pos_handle(jnt_state_interface.getHandle("Maxon"),&joint_cmd_[0]);
        jnt_pos_interface.registerHandle(&pos_handle);

        registerInterface(&jnt_pos_interface);
    }

};