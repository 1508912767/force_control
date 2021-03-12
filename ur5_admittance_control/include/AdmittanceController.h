#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"
#include "ur5_admittance_control/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float32.h"

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class AdmittanceController
{
protected:
  ////// ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;


  ///// Subscribers:
  // Subscriber for the arm state
  ros::Subscriber sub_arm_state_;
  // Subscriber for the ft sensor at the endeffector
  ros::Subscriber sub_wrench_external_;

  ////// Publishers:
  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;

  ////// INPUT SIGNAL
  // external wrench (force/torque sensor) in "robotiq_force_torque_frame_id" frame
  Vector6d wrench_external_;


  ////// FORCE/TORQUE-SENSOR FILTER:
  // Parameters for the noisy wrench
  double wrench_filter_factor_;
  double force_dead_zone_thres_;
  double torque_dead_zone_thres_;
  double admittance_ratio_;


  /////// ADMITTANCE PARAMETERS:
  // M_a_ -> Desired mass of arm
  // D_a_ -> Desired damping of arm
  // K_ -> Desired Stiffness of the coupling
  Matrix6d M_a_, D_a_, K_a_;
  // equilibrium position of the coupling spring
  Vector3d equilibrium_position_;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;

  // OUTPUT COMMANDS
  // final arm desired velocity 
  Vector6d arm_desired_twist_final_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;


  ////// STATE VARIABLES:
  // Arm state: position, orientation, and twist (in "ur5_arm_base_link")
  Vector3d arm_real_position_;
  Quaterniond arm_real_orientation_;
  Vector6d arm_real_twist_;


  // Transform from base_link to world
  Matrix6d rotation_base_;

  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // 判断是否所有坐标系矩阵建立
  bool transformation;

  // Initialization
  void wait_for_transformations();

  // Control
  void compute_admittance();



  // Callbacks
  void state_arm_callback(const ur5_admittance_control::PoseTwistConstPtr msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);

  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,tf::TransformListener & listener,std::string from_frame,  std::string to_frame);

  void limit_to_workspace();

  void send_commands_to_robot();


public:
  AdmittanceController(ros::NodeHandle &n, double frequency,
                       std::string cmd_topic_arm,
                       std::string state_topic_arm,
                       std::string wrench_topic,
                       std::vector<double> M_a,
                       std::vector<double> D_a,
                       std::vector<double> K_a,
                       std::vector<double> d_e,
                       std::vector<double> workspace_limits,
                       double arm_max_vel,
                       double arm_max_acc,
                       double wrench_filter_factor,
                       double force_dead_zone_thres,
                       double torque_dead_zone_thres);
  void run();
};

#endif // ADMITTANCECONTROLLER_H

