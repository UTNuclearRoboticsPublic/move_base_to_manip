
//////////////////////////////////////////////////////////////////
// Move the base until a desired end-effector pose can be reached.
// Andy Zelenak, 2017
//////////////////////////////////////////////////////////////////

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "move_base_to_manip/desired_robot_pose.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_listener.h"

namespace move_base_to_manip
{
  const double cartesian_motion(const std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroupInterface& moveGroup, ros::NodeHandle& nh);

  void set_node_params(ros::NodeHandle &nh);

  void setup_move_group(ros::NodeHandle& nh, moveit::planning_interface::MoveGroupInterface& moveGroup);

  void setup_base_marker(visualization_msgs::Marker& baseMarker, move_base_msgs::MoveBaseGoal& goal);

  ros::ServiceClient clear_octomap_client;
  ros::ServiceClient clear_costmaps_client;

  // An empty service call
  std_srvs::Empty empty_srv;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define SUCCESS false
#define FAILURE true
