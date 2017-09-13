
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
  moveit::planning_interface::MoveGroupInterface move_group( move_group_name );

  // get the current EE pose
  geometry_msgs::PoseStamped start_pose = move_group.getCurrentPose();

  // make sure the goal is in the right frame
  std::string base_frame_name;
  nh_.getParam("/move_base_to_manip/base_frame_name", base_frame_name);
  move_base_to_manip::desired_poseResult result;
  if (goal->desired_pose.header.frame_id != base_frame_name)
  {
    result.success = false;
    as_.setSucceeded( result );
    ROS_WARN_STREAM("The target pose should be given in " << base_frame_name);
    return;
  }

  // We don't want to move in (X,Y), initially
  geometry_msgs::PoseStamped desired_height_orient = goal->desired_pose;
  desired_height_orient.pose.position.x = start_pose.pose.position.x;
  desired_height_orient.pose.position.y = start_pose.pose.position.y;

  ///////////////////////////////////////////////////
  // Put the EE at the height & orientation we desire
  ///////////////////////////////////////////////////
  ROS_INFO_STREAM("[move_base_to_manip] Moving to the desired height and orientation.");
  move_group.setPoseTarget( desired_height_orient );

  // Get the orientation as RPY so we can manipulate it
  tf::Quaternion gripper_quat;
  tf::quaternionMsgToTF( desired_height_orient.pose.orientation, gripper_quat );
  double object_roll, object_pitch, object_yaw;
  tf::Matrix3x3(gripper_quat).getRPY(object_roll, object_pitch, object_yaw);

  moveit::planning_interface::MoveGroupInterface::Plan move_plan;

PLAN_AGAIN:
  bool ok_to_flip;
  nh_.getParam("/move_base_to_manip/ok_to_flip", ok_to_flip); 
  if ( !move_group.plan(move_plan) && ok_to_flip )  // If it fails, try spinning the gripper 180deg
  {
    geometry_msgs::Quaternion gripper_quat_msg = tf::createQuaternionMsgFromRollPitchYaw( 0., 0., object_yaw +3.14159);
    desired_height_orient.pose.orientation = gripper_quat_msg;
    move_group.setPoseTarget( desired_height_orient );
    if ( !move_group.plan(move_plan) ) // If it fails again, try spinning the gripper -180deg from the original attempt
    {  
      gripper_quat_msg = tf::createQuaternionMsgFromRollPitchYaw( 0., 0., object_yaw -3.14159);
      desired_height_orient.pose.orientation = gripper_quat_msg;
      move_group.setPoseTarget( desired_height_orient );
      if( !move_group.plan(move_plan) ) // One last attempt
      {
        ROS_ERROR("Failed to reach the desired height and orientation.");
        ROS_ERROR("Try starting from an arm position with better manipulability.");
        ros::shutdown();
        return;
      }
    }
  }

  bool prompt_before_motion;
  nh_.getParam("/move_base_to_manip/prompt_before_motion", prompt_before_motion);
  if ( prompt_before_motion )
  {
    char character;
    ROS_INFO_STREAM("-----------------------------------------");
    ROS_INFO_STREAM("Enter 'c' to continue, otherwise re-plan.");
    ROS_INFO_STREAM("-----------------------------------------");
    std::cin.clear();
    std::cin.get(character);
    if ( character != 'c' )
      goto PLAN_AGAIN;
  }

  move_group.execute(move_plan);


  //////////////////////////////////////////////////////////////////////////
  // Calculate the X,Y vector from the EE's current pose to the desired pose
  //////////////////////////////////////////////////////////////////////////
  geometry_msgs::Vector3 vec_from_cur_pose_to_goal;
  vec_from_cur_pose_to_goal.x = goal->desired_pose.pose.position.x - start_pose.pose.position.x;
  vec_from_cur_pose_to_goal.y = goal->desired_pose.pose.position.y - start_pose.pose.position.y;
  vec_from_cur_pose_to_goal.z = 0.;

  /////////////////////////////////////////////////////////////
  // Plan a Cartesian move to the grasp pose. What % completes?
  // If 100% complete, we're done!
  /////////////////////////////////////////////////////////////
  ROS_INFO_STREAM("Planning a Cartesian motion to the desired pose.");
  
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose cartesian_target_pose; // Cartesian motion requires a Pose (not PoseStamped)
  cartesian_target_pose.position = goal->desired_pose.pose.position;
  cartesian_target_pose.orientation = goal->desired_pose.pose.orientation;
  waypoints.push_back(cartesian_target_pose);
  
  moveit_msgs::RobotTrajectory trajectory;

PLAN_CARTESIAN_AGAIN:
  double fraction = cartesian_motion(waypoints, trajectory, move_group);
  ROS_INFO("Cartesian path: %.2f%% achieved", fraction * 100.);
 
  if ( prompt_before_motion )
  { 
    char character;
    ROS_INFO_STREAM("-----------------------------------------");
    ROS_INFO_STREAM("Enter 'c' to continue, otherwise re-plan.");
    ROS_INFO_STREAM("-----------------------------------------");
    std::cin.ignore (INT_MAX, '\n'); // Make sure the buffer is empty. 
    std::cin.get(character);
    if ( character != 'c' )
      goto PLAN_CARTESIAN_AGAIN;
  }


  if ( ( 0.999 < fraction) && (fraction < 1.001) ) // We're there! Move then quit.
  {
    ROS_INFO_STREAM("Making the final move.");
    
    bool move_cartesian;
    nh_.getParam("/move_base_to_manip/move_cartesian", move_cartesian);
    if ( move_cartesian ) // Use a Cartesian motion, i.e. keep the end-effector orientation constant as it moves
      move_group.move();
    else // Execute a regular motion
    {
      move_group.setPoseTarget( goal->desired_pose );
      move_group.plan(move_plan);
      move_group.execute(move_plan);
    }
    
    // Return "success" to the action server
    result.success = true;
    as_.setSucceeded( result );
    return;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  // Based on the completed Cartesian %, how far along the (X,Y) vector must the base move?
  // Move the base to that position
  /////////////////////////////////////////////////////////////////////////////////////////

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.frame_id = base_frame_name;
  mb_goal.target_pose.header.stamp = ros::Time::now();

  // Goal position = current base position + calculated change
  // The current base position is (0,0) in /base_link by default, so we don't need to add anything
  geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose();
  // motion_buffer: make the base move just a bit farther than the minimum req'd distance
  // fraction: fraction of the motion that the arm alone could complete
  double motion_buffer;
  nh_.getParam("/move_base_to_manip/motion_buffer", motion_buffer);
  mb_goal.target_pose.pose.position.x = (1-motion_buffer*fraction)*vec_from_cur_pose_to_goal.x;
  mb_goal.target_pose.pose.position.y = (1-motion_buffer*fraction)*vec_from_cur_pose_to_goal.y;
  mb_goal.target_pose.pose.position.z = 0.; // Stay in the plane
  // Maintain the base's current orientation
  mb_goal.target_pose.pose.orientation.x = 0.;
  mb_goal.target_pose.pose.orientation.y = 0.;
  mb_goal.target_pose.pose.orientation.z = 0.;
  mb_goal.target_pose.pose.orientation.w = 1.;

  // May want to disable collision checking or the manipulator will not approach an object.
  bool clear_costmaps;
  if ( nh_.getParam("/move_base_to_manip/clear_costmaps", clear_costmaps) )
    base_planner::clear_costmaps_client_.call( empty_srv_ );
  
  ac.sendGoal(mb_goal);


  ///////////////////////////////////////////
  // Tell the action client that we succeeded
  ///////////////////////////////////////////
  result.success = true;
  as_.setSucceeded( result );

  return;
}


/////////////////////////////////////////////
// Helper function to plan a Cartesian motion
/////////////////////////////////////////////
const double base_planner::cartesian_motion(const std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroupInterface& move_group)
{
  // May want to disable collision checking or the manipulator will not approach an object.
  bool clear_octomap;
  if ( nh_.getParam("/move_base_to_manip/clear_octomap", clear_octomap) )
  {
    clear_octomap_client_.call(empty_srv_);
  }
  double cartesian_path_resolution;
  nh_.getParam("/move_base_to_manip/cartesian_plan_res", cartesian_path_resolution);
  double fraction = move_group.computeCartesianPath( waypoints, cartesian_path_resolution, 0.0, trajectory);

  return fraction;
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
    nh_.setParam("/move_base_to_manip/prompt_before_motion", true);
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
