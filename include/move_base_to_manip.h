
//////////////////////////////////////////////////////////////////
// Move the base until a desired end-effector pose can be reached.
// Andy Zelenak, 2017
//////////////////////////////////////////////////////////////////

#include <actionlib/client/simple_action_client.h>  // move_base needs an action client
#include <actionlib/server/simple_action_server.h>  // This node itself is an action server
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_to_manip/desired_poseAction.h>
#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_listener.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class base_planner
{
private:

  // Helper function to set node parameters, if they aren't defined in a launch file
  void set_node_params();

  // Once a goal is received, start planning and moving
  void do_motion_CB( const move_base_to_manip::desired_poseGoalConstPtr& goal );

  // Helper function to plan a Cartesian motion
  const double cartesian_motion(const std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroup& moveGroup);

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<move_base_to_manip::desired_poseAction> as_;
  moveit::planning_interface::MoveGroup::Plan move_plan_;
  ros::ServiceClient clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  ros::ServiceClient clear_octomap_client_ = nh_.serviceClient<std_srvs::Empty>("clear_octomap");
  std_srvs::Empty empty_srv_;

public:
  // Constructor
  base_planner();

};

#define SUCCESS false
#define FAILURE true
