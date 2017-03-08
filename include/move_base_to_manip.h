
//////////////////////////////////////////////////////////////////
// Move the base until a desired end-effector pose can be reached.
// Andy Zelenak, 2017
//////////////////////////////////////////////////////////////////

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "std_srvs/Empty.h"

namespace move_base_to_manip
{
  const double cartesian_motion(const std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroupInterface& moveGroup, ros::NodeHandle& nh);

  ros::ServiceClient clear_octomap_client;
  ros::ServiceClient clear_costmaps_client;

  // An empty service call
  std_srvs::Empty empty_srv;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define SUCCESS false
#define FAILURE true
