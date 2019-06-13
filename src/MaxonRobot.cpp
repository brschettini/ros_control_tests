#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class MaxonRobot : public hardware_interface::RobotHW {
  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double joint_cmd_[1];
    double joint_pos_[1];
    double joint_vel_[1];
    double joint_eff_[1];
  public:
    MaxonRobot() {
        
    }
};