
//////////////////////////////////////////////////////////////////
// Move the base until a desired end-effector pose can be reached.
// Andy Zelenak, 2017
//////////////////////////////////////////////////////////////////

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_to_manip/desired_poseAction.h>
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_listener.h"

class base_planner
{
private:

  // Set node parameters, if they aren't defined in a launch file
  void set_node_params();

  // Once a goal is received, start planning and moving
  void do_motion_CB( const move_base_to_manip::desired_poseGoalConstPtr& goal );

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<move_base_to_manip::desired_poseAction> as_;
  moveit::planning_interface::MoveGroupInterface::Plan move_plan_;

public:
  // Constructor
  base_planner();

};

#define SUCCESS false
#define FAILURE true
