
#include "move_base_to_manip.h"

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
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh;
  move_base_to_manip::set_node_params(nh);

  std::string move_group_name;
  nh.getParam("/move_base_to_manip/move_group_name", move_group_name);
  moveit::planning_interface::MoveGroup moveGroup( move_group_name );
  
  move_base_to_manip::setup_move_group(nh, moveGroup);
  
  moveit::planning_interface::MoveGroup::Plan move_plan;
  
  geometry_msgs::PoseStamped start_pose = moveGroup.getCurrentPose();

  // Set up services
  move_base_to_manip::clear_octomap_client = nh.serviceClient<std_srvs::Empty>("clear_octomap");
  move_base_to_manip::clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  ////////////////////////////////////////////////////////////////////////
  // Get the desired EE pose from the "desired_robot_pose" service.
  ////////////////////////////////////////////////////////////////////////
  ros::ServiceClient client = nh.serviceClient<move_base_to_manip::desired_robot_pose>("desired_robot_pose");
  move_base_to_manip::desired_robot_pose srv;
  srv.request.shutdown_service = true; // Shut down the service after it sends the pose.
  geometry_msgs::PoseStamped desired_pose;

  while ( !client.call(srv) ) // If we couldn't read the desired pose. The service prob isn't up yet
  {
    ROS_INFO_STREAM("Waiting for the 'desired_robot_pose' service.");
    ros::Duration(2).sleep();
  }
  
  desired_pose = srv.response.desired_robot_pose;
  
  // We don't want to move in (X,Y), initially
  geometry_msgs::PoseStamped desired_height_orient = desired_pose;
  desired_height_orient.pose.position.x = start_pose.pose.position.x;
  desired_height_orient.pose.position.y = start_pose.pose.position.y;

  ///////////////////////////////////////////////////
  // Put the EE at the height & orientation we desire
  ///////////////////////////////////////////////////

  ROS_INFO_STREAM("Moving to the desired height and orientation.");
  moveGroup.setPoseTarget( desired_height_orient );

  // Get the orientation as RPY so we can manipulate it
  tf::Quaternion gripper_quat;
  tf::quaternionMsgToTF( desired_height_orient.pose.orientation, gripper_quat );
  double object_roll, object_pitch, object_yaw;
  tf::Matrix3x3(gripper_quat).getRPY(object_roll, object_pitch, object_yaw);

PLAN_AGAIN:
  bool ok_to_flip;
  nh.getParam("/move_base_to_manip/ok_to_flip", ok_to_flip);
  if ( !moveGroup.plan(move_plan) && ok_to_flip )  // If it fails, try spinning the gripper 180deg
  {
    geometry_msgs::Quaternion gripper_quat_msg = tf::createQuaternionMsgFromRollPitchYaw( 0., 0., object_yaw +3.14159);
    desired_height_orient.pose.orientation = gripper_quat_msg;
    moveGroup.setPoseTarget( desired_height_orient );
    if ( !moveGroup.plan(move_plan) ) // If it fails again, try spinning the gripper -180deg from the original attempt
    {  
      gripper_quat_msg = tf::createQuaternionMsgFromRollPitchYaw( 0., 0., object_yaw -3.14159);
      desired_height_orient.pose.orientation = gripper_quat_msg;
      moveGroup.setPoseTarget( desired_height_orient );
      if( !moveGroup.plan(move_plan) ) // One last attempt
      {
        ROS_ERROR("Failed to reach the desired height and orientation.");
        ROS_ERROR("Try starting from an arm position with better manipulability.");
        ros::shutdown();
        return FAILURE;
      }
    }
  }

  bool prompt_before_motion;
  nh.getParam("/move_base_to_manip/prompt_before_motion", prompt_before_motion);
  if ( prompt_before_motion )
  {
    char character;
    ROS_INFO_STREAM("Enter 'c' to continue, otherwise re-plan.");
    std::cin.clear();
    std::cin.get(character);
    if ( character != 'c' )
      goto PLAN_AGAIN;
  }

  moveGroup.execute(move_plan);
  
  //////////////////////////////////////////////////////////////////////////
  // Calculate the X,Y vector from the EE's current pose to the desired pose
  //////////////////////////////////////////////////////////////////////////
  geometry_msgs::Vector3 vec_from_cur_pose_to_goal;
  vec_from_cur_pose_to_goal.x = desired_pose.pose.position.x - start_pose.pose.position.x;
  vec_from_cur_pose_to_goal.y = desired_pose.pose.position.y - start_pose.pose.position.y;
  vec_from_cur_pose_to_goal.z = 0.;
  
  /////////////////////////////////////////////////////////////
  // Plan a Cartesian move to the grasp pose. What % completes?
  // If 100% complete, we're done!
  /////////////////////////////////////////////////////////////
  ROS_INFO_STREAM("Planning a Cartesian motion to the desired pose.");
  
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose cartesian_target_pose; // Cartesian motion requires a Pose (not PoseStamped)
  cartesian_target_pose.position = desired_pose.pose.position;
  cartesian_target_pose.orientation = desired_pose.pose.orientation;
  waypoints.push_back(cartesian_target_pose);
  
  moveit_msgs::RobotTrajectory trajectory;

PLAN_CARTESIAN_AGAIN:
  double fraction = move_base_to_manip::cartesian_motion(waypoints, trajectory, moveGroup, nh);
  ROS_INFO("Cartesian path: %.2f%% achieved", fraction * 100.);
 
  if ( prompt_before_motion )
  { 
    char character;
    ROS_INFO_STREAM("Enter 'c' to continue, otherwise re-plan.");
    std::cin.ignore (INT_MAX, '\n'); // Make sure the buffer is empty. 
    std::cin.get(character);
    if ( character != 'c' )
      goto PLAN_CARTESIAN_AGAIN;
  }
 
  if ( ( 0.999 < fraction) && (fraction < 1.001) ) // We're there! Move then quit.
  {
    ROS_INFO_STREAM("Making the final move.");
    
    bool move_cartesian;
    nh.getParam("/move_base_to_manip/move_cartesian", move_cartesian);
    if ( move_cartesian ) // Use a Cartesian motion, i.e. keep the end-effector orientation constant as it moves
      moveGroup.move();
    else // Execute a regular motion
    {
      moveGroup.setPoseTarget( desired_pose );
      moveGroup.plan(move_plan);
      moveGroup.execute(move_plan);
    }
    
    ros::shutdown();
    return SUCCESS;
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
  
  move_base_msgs::MoveBaseGoal goal;
  std::string base_frame_name;
  nh.getParam("/move_base_to_manip/base_frame_name", base_frame_name);
  goal.target_pose.header.frame_id = base_frame_name;
  goal.target_pose.header.stamp = ros::Time::now();

  // Goal position = current base position + calculated change
  // The current base position is (0,0) in /base_link by default, so we don't need to add anything
  geometry_msgs::PoseStamped currentPose = moveGroup.getCurrentPose();
  // motion_buffer: make the base move just a bit farther than the minimum req'd distance
  // fraction: fraction of the motion that the arm alone could complete
  double motion_buffer;
  nh.getParam("/move_base_to_manip/motion_buffer", motion_buffer);
  goal.target_pose.pose.position.x = (1-motion_buffer*fraction)*vec_from_cur_pose_to_goal.x;
  goal.target_pose.pose.position.y = (1-motion_buffer*fraction)*vec_from_cur_pose_to_goal.y;
  goal.target_pose.pose.position.z = 0.; // Stay in the plane
  // Maintain the base's current orientation
  goal.target_pose.pose.orientation.x = 0.;
  goal.target_pose.pose.orientation.y = 0.;
  goal.target_pose.pose.orientation.z = 0.;
  goal.target_pose.pose.orientation.w = 1.;

  // Mark the calculated base location in Rviz
  ros::Publisher baseVisualizationPublisher = nh.advertise<visualization_msgs::Marker>("base_pose_marker", 1);
  ros::Duration(1).sleep();
  visualization_msgs::Marker baseMarker;
  move_base_to_manip::setup_base_marker(baseMarker, goal);
  baseVisualizationPublisher.publish(baseMarker);
  ros::Duration(1).sleep();
  
  // May want to disable collision checking or the manipulator will not approach an object.
  bool clear_costmaps;
  if ( nh.getParam("/move_base_to_manip/clear_costmaps", clear_costmaps) )
    move_base_to_manip::clear_costmaps_client.call( move_base_to_manip::empty_srv );

  // Send the goal to move_base
  ac.sendGoal(goal);

  // If the robot still can't reach the goal (it should be very close), run this program again.

  ros::shutdown();
  return SUCCESS;
}

// Helper function to set node parameters, if they aren't defined in a launch file
void move_base_to_manip::set_node_params(ros::NodeHandle &nh)
{
  // Make the base move just a bit farther than the minimum req'd distance.
  // This should be a fraction between 0-1
  // Smaller ==> Will move closer to the goal pose 
  if (!nh.hasParam("/move_base_to_manip/motion_buffer"))
    nh.setParam("/move_base_to_manip/motion_buffer", 0.15);

  // Use a Cartesian motion plan or a regular motion plan?
  if (!nh.hasParam("/move_base_to_manip/move_cartesian"))
  {
    bool temp = false;
    nh.setParam("/move_base_to_manip/move_cartesian", temp);
  }

  // Clear the Octomap collision scene before planning the final arm motion?
  if (!nh.hasParam("/move_base_to_manip/clear_octomap"))
  {
    bool temp = true;  
    nh.setParam("/move_base_to_manip/clear_octomap", temp);
  }

  // Clear the move_base costmaps before moving the base?
  if (!nh.hasParam("/move_base_to_manip/clear_costmaps"))
  {
    bool temp = true;  
    nh.setParam("/move_base_to_manip/clear_costmaps", temp);
  }

  // Prompt the user to approve each arm motion before it executes?
  if (!nh.hasParam("/move_base_to_manip/prompt_before_motion"))
  {
    bool temp = true;
    nh.setParam("/move_base_to_manip/prompt_before_motion", temp);
  }

  // Cartesian planning resolution, in meters
  if (!nh.hasParam("/move_base_to_manip/cartesian_plan_res"))
    nh.setParam("/move_base_to_manip/cartesian_plan_res", 0.005);

  if (!nh.hasParam("/move_base_to_manip/move_group_name"))
    nh.setParam("/move_base_to_manip/move_group_name", "right_ur5");

  if (!nh.hasParam("/move_base_to_manip/move_group_planner"))
    nh.setParam("/move_base_to_manip/move_group_planner", "RRTConnectkConfigDefault");

  if (!nh.hasParam("/move_base_to_manip/velocity_scale"))
    nh.setParam("/move_base_to_manip/velocity_scale", 0.1);

  if (!nh.hasParam("/move_base_to_manip/base_frame_name"))
    nh.setParam("/move_base_to_manip/base_frame_name", "base_link");
    
 if (!nh.hasParam("/move_base_to_manip/position_tolerance"))
    nh.setParam("/move_base_to_manip/position_tolerance", 0.01);
    
 if (!nh.hasParam("/move_base_to_manip/orientation_tolerance"))
    nh.setParam("/move_base_to_manip/orientation_tolerance", 0.0001);

 // If true, the planner will try to flip the gripper +/-180 deg about Z when it cannot reach a pose
 if (!nh.hasParam("/move_base_to_manip/ok_to_flip"))
 {
   bool temp = true;
   nh.setParam("/move_base_to_manip/ok_to_flip", temp);
  }
}

// Helper function to plan a Cartesian motion
const double move_base_to_manip::cartesian_motion(const std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroup& moveGroup, ros::NodeHandle &nh)
{
  // May want to disable collision checking or the manipulator will not approach an object.
  bool clear_octomap;
  if ( nh.getParam("/move_base_to_manip/clear_octomap", clear_octomap) )
    move_base_to_manip::clear_octomap_client.call(empty_srv);
  double cartesian_path_resolution;
  nh.getParam("/move_base_to_manip/cartesian_plan_res", cartesian_path_resolution);
  double fraction = moveGroup.computeCartesianPath( waypoints, cartesian_path_resolution, 0.0, trajectory);

  return fraction;
}

// Helper function to initialize move_group
void move_base_to_manip::setup_move_group(ros::NodeHandle& nh, moveit::planning_interface::MoveGroup& moveGroup)
{
  std::string move_group_planner;
  nh.getParam("/move_base_to_manip/move_group_planner", move_group_planner);
  moveGroup.setPlannerId( move_group_planner );
  double velocity_scale;
  nh.getParam("/move_base_to_manip/velocity_scale", velocity_scale);
  moveGroup.setMaxVelocityScalingFactor( velocity_scale );
  
  double pos_tol;
  nh.getParam("/move_base_to_manip/position_tolerance", pos_tol);
  moveGroup.setGoalPositionTolerance(pos_tol);
  
  double orient_tol;
  nh.getParam("/move_base_to_manip/orientation_tolerance", orient_tol);
  moveGroup.setGoalOrientationTolerance(orient_tol);
}

// Helper function to set the RViz marker
void move_base_to_manip::setup_base_marker(visualization_msgs::Marker& baseMarker, move_base_msgs::MoveBaseGoal& goal)
{
  baseMarker.header = goal.target_pose.header;
  baseMarker.id = 927;
  baseMarker.ns = "basic_shapes";
  baseMarker.type = visualization_msgs::Marker::CUBE;
  baseMarker.action = visualization_msgs::Marker::ADD;
  baseMarker.pose = goal.target_pose.pose;

  baseMarker.scale.x = 0.22;
  baseMarker.scale.y = 0.08;
  baseMarker.scale.z = 0.08;
  baseMarker.color.a = 1.0;
  baseMarker.color.r = 1.0f;
  baseMarker.color.g = 0.0f;
  baseMarker.color.b = 0.0f;
  baseMarker.lifetime = ros::Duration();
}
