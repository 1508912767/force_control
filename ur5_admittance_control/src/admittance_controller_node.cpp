#include "ros/ros.h"
#include "AdmittanceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_controller_node");

  ros::NodeHandle nh;
  double frequency = 100.0;

  // Parameters
  std::string topic_arm_state;
  std::string topic_arm_command;
  std::string topic_external_wrench;

  std::vector<double> M_a;
  std::vector<double> D_a;
  std::vector<double> K_a;
  std::vector<double> d_e;
  std::vector<double> workspace_limits;

  double arm_max_vel;
  double arm_max_acc;

  double wrench_filter_factor;
  double force_dead_zone_thres;
  double torque_dead_zone_thres;


  /// LOADING PARAMETERS FROM THE ROS SERVER
  if (!nh.getParam("topic_arm_state", topic_arm_state)) {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
    return -1;
  }
  if (!nh.getParam("topic_arm_command", topic_arm_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }
  if (!nh.getParam("topic_external_wrench", topic_external_wrench)) {
    ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor.");
    return -1;
  }

  /// ADMITTANCE PARAMETERS
  if (!nh.getParam("mass_arm", M_a)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    return -1;
  }
  if (!nh.getParam("damping_arm", D_a)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }
  if (!nh.getParam("stiffness_arm", K_a)) {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
    return -1;
  }
  if (!nh.getParam("equilibrium_point_spring", d_e)) {
    ROS_ERROR("Couldn't retrieve the desired equilibrium of the spring.");
    return -1;
  }

  /// SAFETY PARAMETERS
  if (!nh.getParam("workspace_limits", workspace_limits)) {
    ROS_ERROR("Couldn't retrieve the limits of the workspace.");
    return -1;
  }
  if (!nh.getParam("arm_max_vel", arm_max_vel)) {
    ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
    return -1;
  }
  if (!nh.getParam("arm_max_acc", arm_max_acc)) {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }

  /// FORCE/TORQUE-SENSOR PARAMETERS
  if (!nh.getParam("wrench_filter_factor", wrench_filter_factor)) {
    ROS_ERROR("Couldn't retrieve the desired wrench filter factor.");
    return -1;
  }
  if (!nh.getParam("force_dead_zone_thres", force_dead_zone_thres)) {
    ROS_ERROR("Couldn't retrieve the desired force_dead_zone threshold.");
    return -1;
  }
  if (!nh.getParam("torque_dead_zone_thres", torque_dead_zone_thres)) {
    ROS_ERROR("Couldn't retrieve the desired torque_dead_zone threshold. ");
    return -1;
  }

  // Constructing the controller
  AdmittanceController admittance_controller(
    nh,
    frequency,
    topic_arm_command,
    topic_arm_state,
    topic_external_wrench,
    M_a, D_a, K_a, d_e,
    workspace_limits,
    arm_max_vel, arm_max_acc,
    wrench_filter_factor,
    force_dead_zone_thres,
    torque_dead_zone_thres);

  // Running the controller
  admittance_controller.run();

  return 0;
}
