
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
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh;
  move_base_to_manip::set_node_params(nh);
  
  std::string move_group_name;
  nh.getParam("/move_base_to_manip/move_group_name", move_group_name);
  moveit::planning_interface::MoveGroupInterface moveGroup( move_group_name );
  move_base_to_manip::setup_move_group(nh, moveGroup);
  
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;
  
  geometry_msgs::PoseStamped start_pose = moveGroup.getCurrentPose();

  // Set up services
  move_base_to_manip::clear_octomap_client = nh.serviceClient<std_srvs::Empty>("clear_octomap");
  move_base_to_manip::clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");


  /////////////////////////////////////////////////////////////////
  // Get the desired EE pose from the "desired_robot_pose" service.
  /////////////////////////////////////////////////////////////////
  ros::ServiceClient client = nh.serviceClient<move_base_to_manip::desired_robot_pose>("desired_robot_pose");
  move_base_to_manip::desired_robot_pose desired_robot_pose_srv;

  bool shutdown_flag;
  nh.getParam("/move_base_to_manip/shutdown", shutdown_flag);
  desired_robot_pose_srv.request.shutdown_service = shutdown_flag; // Shut down the service after it sends the pose?
  std_msgs::String str;
  str.data = "cylinder";
  desired_robot_pose_srv.request.object_type = str;
  str.data = "side";
  desired_robot_pose_srv.request.grasp_approach = str;

  while ( !client.call(desired_robot_pose_srv) ) // If we couldn't read the desired pose. The service prob isn't up yet
  {
    ROS_INFO_STREAM("Waiting for the 'desired_robot_pose' service.");
    ros::Duration(2).sleep();
  }

  // desired_robot_pose returns a pose in the world frame.
  // Store it so we know the pose even as the robot moves.
  geometry_msgs::PoseStamped desired_pose_world = desired_robot_pose_srv.response.desired_robot_pose;

  // Also convert it to /base_link for further calculations
  geometry_msgs::PoseStamped desired_pose_base_link;
  std::string base_frame_name;
  nh.getParam("/move_base_to_manip/base_frame_name", base_frame_name);
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  tf::TransformListener listener;
  listener.waitForTransform(base_frame_name, desired_robot_pose_srv.response.desired_robot_pose.header.frame_id, ros::Time(0), ros::Duration(10.0) );
  try{
    geometry_msgs::TransformStamped tf_to_base_link_frame = tfBuffer.lookupTransform("base_link", desired_robot_pose_srv.response.desired_robot_pose.header.frame_id, ros::Time(0) );

    tf2::doTransform(desired_robot_pose_srv.response.desired_robot_pose, desired_pose_base_link, tf_to_base_link_frame);
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  // We don't want to move in (X,Y), initially
  geometry_msgs::PoseStamped desired_height_orient = desired_pose_base_link;
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
        return false;
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
  vec_from_cur_pose_to_goal.x = desired_pose_base_link.pose.position.x - start_pose.pose.position.x;
  vec_from_cur_pose_to_goal.y = desired_pose_base_link.pose.position.y - start_pose.pose.position.y;
  vec_from_cur_pose_to_goal.z = 0.;
  

  /////////////////////////////////////////////////////////////
  // Plan a Cartesian move to the grasp pose. What % completes?
  // If 100% complete, we're done!
  /////////////////////////////////////////////////////////////
  ROS_INFO_STREAM("Planning a Cartesian motion to the desired pose.");
  
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose cartesian_target_pose; // Cartesian motion requires a Pose (not PoseStamped)
  cartesian_target_pose.position = desired_pose_base_link.pose.position;
  cartesian_target_pose.orientation = desired_pose_base_link.pose.orientation;
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
      moveGroup.setPoseTarget( desired_pose_base_link );
      moveGroup.plan(move_plan);
      moveGroup.execute(move_plan);
    }
    
    ros::shutdown();
    return true;
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
  
  //ac.sendGoal(goal);


  ///////////////////////////////////////////////////////////////////////
  // Call a service to calculate a new camera rotation.
  // This is helpful to keep the desired pose in view after base motion.
  ///////////////////////////////////////////////////////////////////////

  // Call the service
  look_at_pose::LookAtPose look_at_pose_srv;
  if ( !move_base_to_manip::look_at_pose_call( nh, desired_pose_world, listener, look_at_pose_srv ) )
  {
  	ROS_ERROR_STREAM("move_base_to_manip: The look_at_pose service call failed.");
  	return false;
  }

  // Now rotate the camera
  ROS_INFO_STREAM("Waiting, then rotating the camera.");
  ros::Duration(15).sleep();

  // New cam pose in base_link
  listener.waitForTransform( "base_link", look_at_pose_srv.response.new_cam_pose.header.frame_id, ros::Time(0), ros::Duration(10.0) );
  try{
    geometry_msgs::TransformStamped tf_to_base_link_frame = tfBuffer.lookupTransform("base_link", look_at_pose_srv.response.new_cam_pose.header.frame_id, ros::Time(0) );
    tf2::doTransform(look_at_pose_srv.response.new_cam_pose, look_at_pose_srv.response.new_cam_pose, tf_to_base_link_frame);
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  ROS_INFO_STREAM( "New camera pose in base_link:  " << look_at_pose_srv.response.new_cam_pose );

  // A special moveGroup planner that uses camera_ee_link as the end-effector
  moveit::planning_interface::MoveGroupInterface cameraMoveGroup( "right_ur5_camera" );
  move_base_to_manip::setup_move_group(nh, cameraMoveGroup);

  // Visualize the new camera pose
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( look_at_pose_srv.response.new_cam_pose.pose.position.x, look_at_pose_srv.response.new_cam_pose.pose.position.y, look_at_pose_srv.response.new_cam_pose.pose.position.z ) );
  //tf::Quaternion q( look_at_pose_srv.new_cam_pose.pose.orientation.x, look_at_pose_srv.new_cam_pose.pose.orientation.y, look_at_pose_srv.new_cam_pose.pose.orientation.z, look_at_pose_srv.new_cam_pose.pose.orientation.w );

  // Move to the new camera pose
  waypoints.clear();
  waypoints.push_back(look_at_pose_srv.response.new_cam_pose.pose);
  fraction = move_base_to_manip::cartesian_motion(waypoints, trajectory, cameraMoveGroup, nh);
  ROS_INFO("Cartesian path: %.2f%% achieved", fraction * 100.);
  cameraMoveGroup.move();

  // If the robot still can't reach the goal (it should be very close), run this program again.
  ros::shutdown();
  return true;
}

// Helper function to make a "look_at_pose" service call
bool move_base_to_manip::look_at_pose_call(ros::NodeHandle &nh, geometry_msgs::PoseStamped &desired_pose_world, tf::TransformListener &listener, look_at_pose::LookAtPose &look_at_pose_srv )
{
  ros::ServiceClient look_at_pose_client = nh.serviceClient<look_at_pose::LookAtPose>("look_at_pose");

  look_at_pose_srv.request.initial_cam_pose.header.frame_id = "camera_ee_link";
  look_at_pose_srv.request.initial_cam_pose.pose.position.x = 0;
  look_at_pose_srv.request.initial_cam_pose.pose.position.y = 0;
  look_at_pose_srv.request.initial_cam_pose.pose.position.z = 0;
  look_at_pose_srv.request.initial_cam_pose.pose.orientation.x = 0;
  look_at_pose_srv.request.initial_cam_pose.pose.orientation.y = 0;
  look_at_pose_srv.request.initial_cam_pose.pose.orientation.z = 0;
  look_at_pose_srv.request.initial_cam_pose.pose.orientation.w = 1;

  look_at_pose_srv.request.target_pose = desired_pose_world;

  geometry_msgs::Vector3Stamped up_vector;
  up_vector.header.frame_id = "base_link";
  up_vector.vector.x = 0;
  up_vector.vector.y = 0;
  up_vector.vector.z = 1;

  // Remove the leading "/" for tf2
  if ( look_at_pose_srv.request.initial_cam_pose.header.frame_id.at(0) == '/' )
    look_at_pose_srv.request.initial_cam_pose.header.frame_id.erase(0,1);

  // Make sure all parts of the request are in the same frame as initial_cam_pose
  // up vector first:
  tf2_ros::Buffer tfBuffer;
  listener.waitForTransform( look_at_pose_srv.request.initial_cam_pose.header.frame_id, up_vector.header.frame_id, ros::Time(0), ros::Duration(10.0) );
  try{
    tf::StampedTransform tf_to_ini_cam_frame;
    listener.lookupTransform(look_at_pose_srv.request.initial_cam_pose.header.frame_id, up_vector.header.frame_id, ros::Time(0), tf_to_ini_cam_frame );

    // Needs to be a geometry_msgs::TransformStamped to use tf2::doTransform()
    geometry_msgs::TransformStamped tf_msg_to_ini_cam_frame;
    tf::transformStampedTFToMsg(tf_to_ini_cam_frame, tf_msg_to_ini_cam_frame);

    tf2::doTransform(up_vector, up_vector, tf_msg_to_ini_cam_frame);
    look_at_pose_srv.request.up = up_vector;
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  ROS_INFO_STREAM("Up vector: " << up_vector);

  // target pose:
  listener.waitForTransform( look_at_pose_srv.request.initial_cam_pose.header.frame_id, look_at_pose_srv.request.target_pose.header.frame_id, ros::Time(0), ros::Duration(10.0) );
  try{
    tf::StampedTransform tf_to_ini_cam_frame;
    listener.lookupTransform(look_at_pose_srv.request.initial_cam_pose.header.frame_id, look_at_pose_srv.request.target_pose.header.frame_id, ros::Time(0), tf_to_ini_cam_frame );
    
    // Needs to be a geometry_msgs::TransformStamped to use tf2::doTransform()
    geometry_msgs::TransformStamped tf_msg_to_ini_cam_frame;
    tf::transformStampedTFToMsg(tf_to_ini_cam_frame, tf_msg_to_ini_cam_frame);
    
    tf2::doTransform(look_at_pose_srv.request.target_pose, look_at_pose_srv.request.target_pose, tf_msg_to_ini_cam_frame);
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  ROS_INFO_STREAM("Target pose: " << look_at_pose_srv.request.target_pose);

  // Make the service call
  while ( !look_at_pose_client.call(look_at_pose_srv) ) // If we couldn't read the desired pose. The service prob isn't up yet
  {
    ROS_INFO_STREAM("Waiting for the 'look_at_pose' service.");
    ros::Duration(2).sleep();
  }
  ROS_INFO_STREAM("New camera pose in camera_ee_link:  " << look_at_pose_srv.response.new_cam_pose);  // The response is a new cam pose, PoseStamped

  return true;
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
    nh.setParam("/move_base_to_manip/move_cartesian", false);
  }

  // Clear the Octomap collision scene before planning the final arm motion?
  if (!nh.hasParam("/move_base_to_manip/clear_octomap"))
  { 
    nh.setParam("/move_base_to_manip/clear_octomap", true);
  }

  // Clear the move_base costmaps before moving the base?
  if (!nh.hasParam("/move_base_to_manip/clear_costmaps"))
  { 
    nh.setParam("/move_base_to_manip/clear_costmaps", true);
  }

  // Prompt the user to approve each arm motion before it executes?
  if (!nh.hasParam("/move_base_to_manip/prompt_before_motion"))
  {
    nh.setParam("/move_base_to_manip/prompt_before_motion", true);
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
   nh.setParam("/move_base_to_manip/ok_to_flip", true);
  }

 // Part of the service request.
 // True ==> May be used to signal that the server which provides the pose can shut down after the service returns.
 if (!nh.hasParam("/move_base_to_manip/shutdown"))
 {
   nh.setParam("/move_base_to_manip/shutdown", true);
  }
}

// Helper function to plan a Cartesian motion
const double move_base_to_manip::cartesian_motion(const std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, moveit::planning_interface::MoveGroupInterface& moveGroup, ros::NodeHandle &nh)
{
  // May want to disable collision checking or the manipulator will not approach an object.
  bool clear_octomap;
  if ( nh.getParam("/move_base_to_manip/clear_octomap", clear_octomap) )
  {
    move_base_to_manip::clear_octomap_client.call(empty_srv);
  }
  double cartesian_path_resolution;
  nh.getParam("/move_base_to_manip/cartesian_plan_res", cartesian_path_resolution);
  double fraction = moveGroup.computeCartesianPath( waypoints, cartesian_path_resolution, 0.0, trajectory);

  return fraction;
}

// Helper function to initialize move_group
void move_base_to_manip::setup_move_group(ros::NodeHandle& nh, moveit::planning_interface::MoveGroupInterface& moveGroup)
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

  std::string base_frame_name;
  nh.getParam("/move_base_to_manip/base_frame_name", base_frame_name);
  moveGroup.setPoseReferenceFrame(base_frame_name);
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
