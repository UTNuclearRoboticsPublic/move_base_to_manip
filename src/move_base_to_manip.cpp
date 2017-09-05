
# include "move_base_to_manip.h"

/////////////////
// The algorithm:
/////////////////

// Put the end-effector at the height & orientation of the desired pose.

// Calculate the X,Y vector from the ee's current pose to the desired pose.

// Plan a Cartesian move to the grasp pose. What % completes? If 100%, we're done!

// Based on the completed Cartesian %, how far along the (X,Y) vector must the base move?
// Move the base to that position.
  
// Should be able to reach it now. Run the program again to complete the motion or make minor
// corrections as needed.

///////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_to_manip");

  // Start an action server
  base_planner commander;

  // Wait for an action goal to trigger the CB
  ros::spin();

  return 0;
}


//////////////
// Constructor
//////////////
base_planner::base_planner():
    as_(nh_, "move_base_to_manip", boost::bind(&base_planner::do_motion_CB, this, _1), false)
{
  // Set node parameters, if they aren't defined in a launch file
  set_node_params();

  // Action server
  as_.start();

}


/////////////////////////////////////////////////////  
// Once a goal is received, start planning and moving
/////////////////////////////////////////////////////
void base_planner::do_motion_CB( const move_base_to_manip::desired_poseGoalConstPtr& goal )
{
  ROS_INFO("[move_base_to_manip] Received a new goal. Moving.");

  // Initialize MoveGroup for the arm
  std::string move_group_name;
  nh_.getParam("/move_base_to_manip/move_group_name", move_group_name);
  //moveit::planning_interface::MoveGroupInterface move_group( move_group_name );

  // get the current EE pose
  //geometry_msgs::PoseStamped start_pose = move_group.getCurrentPose();

  // get the desired EE pose

  // make sure it's in the right frame

  // We don't want to move in (X,Y), initially

  ///////////////////////////////////////////////////
  // Put the EE at the height & orientation we desire
  ///////////////////////////////////////////////////


  //////////////////////////////////////////////////////////////////////////
  // Calculate the X,Y vector from the EE's current pose to the desired pose
  //////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////
  // Plan a Cartesian move to the grasp pose. What % completes?
  // If 100% complete, we're done!
  /////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////////////////
  // Based on the completed Cartesian %, how far along the (X,Y) vector must the base move?
  // Move the base to that position
  /////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////
  // Tell the action client that we succeeded
  ///////////////////////////////////////////
  move_base_to_manip::desired_poseResult result;
  result.success = true;
  as_.setSucceeded( result );

  return;
}


///////////////////////////////////////////////////////////////
// Set node parameters, if they aren't defined in a launch file
///////////////////////////////////////////////////////////////
void base_planner::set_node_params()
{
  // Make the base move just a bit farther than the minimum req'd distance.
  // This should be a fraction between 0-1
  // Smaller ==> Will move closer to the goal pose 
  if (!nh_.hasParam("/move_base_to_manip/motion_buffer"))
    nh_.setParam("/move_base_to_manip/motion_buffer", 0.15);

  // Use a Cartesian motion plan or a regular motion plan?
  if (!nh_.hasParam("/move_base_to_manip/move_cartesian"))
  {
    nh_.setParam("/move_base_to_manip/move_cartesian", false);
  }

  // Clear the Octomap collision scene before planning the final arm motion?
  if (!nh_.hasParam("/move_base_to_manip/clear_octomap"))
  { 
    nh_.setParam("/move_base_to_manip/clear_octomap", false);
  }

  // Clear the move_base costmaps before moving the base?
  if (!nh_.hasParam("/move_base_to_manip/clear_costmaps"))
  { 
    nh_.setParam("/move_base_to_manip/clear_costmaps", false);
  }

  // Prompt the user to approve each arm motion before it executes?
  if (!nh_.hasParam("/move_base_to_manip/prompt_before_motion"))
  {
    nh_.setParam("/move_base_to_manip/prompt_before_motion", false);
  }

  // Cartesian planning resolution, in meters
  if (!nh_.hasParam("/move_base_to_manip/cartesian_plan_res"))
    nh_.setParam("/move_base_to_manip/cartesian_plan_res", 0.005);

  if (!nh_.hasParam("/move_base_to_manip/move_group_name"))
    nh_.setParam("/move_base_to_manip/move_group_name", "right_ur5");

  if (!nh_.hasParam("/move_base_to_manip/move_group_planner"))
    nh_.setParam("/move_base_to_manip/move_group_planner", "RRTConnectkConfigDefault");

  if (!nh_.hasParam("/move_base_to_manip/velocity_scale"))
    nh_.setParam("/move_base_to_manip/velocity_scale", 0.1);

  if (!nh_.hasParam("/move_base_to_manip/base_frame_name"))
    nh_.setParam("/move_base_to_manip/base_frame_name", "base_link");
    
 if (!nh_.hasParam("/move_base_to_manip/position_tolerance"))
    nh_.setParam("/move_base_to_manip/position_tolerance", 0.01);
    
 if (!nh_.hasParam("/move_base_to_manip/orientation_tolerance"))
    nh_.setParam("/move_base_to_manip/orientation_tolerance", 0.01);

 // If true, the planner will try to flip the gripper +/-180 deg about Z when it cannot reach a pose
 if (!nh_.hasParam("/move_base_to_manip/ok_to_flip"))
 {
   nh_.setParam("/move_base_to_manip/ok_to_flip", true);
  }

  return;
}
