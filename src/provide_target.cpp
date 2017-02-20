
#include "move_base_to_manip/desired_robot_pose.h" // Service that provides a desired pose
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

namespace provide_target
{
  visualization_msgs::Marker desired_ee_pose; // Used to visualize the approach pose
  bool shutdown_now = false;
}


// This service provides a desired pose to the requester, then shuts down the node.
bool desired_robot_pose(move_base_to_manip::desired_robot_pose::Request  &req,
         move_base_to_manip::desired_robot_pose::Response &res)
{
  res.desired_robot_pose.header.frame_id = provide_target::desired_ee_pose.header.frame_id;

  res.desired_robot_pose.pose.position.x = provide_target::desired_ee_pose.pose.position.x;
  res.desired_robot_pose.pose.position.y = provide_target::desired_ee_pose.pose.position.y;
  res.desired_robot_pose.pose.position.z = provide_target::desired_ee_pose.pose.position.z;

  res.desired_robot_pose.pose.orientation.x = provide_target::desired_ee_pose.pose.orientation.x;
  res.desired_robot_pose.pose.orientation.y = provide_target::desired_ee_pose.pose.orientation.y;
  res.desired_robot_pose.pose.orientation.z = provide_target::desired_ee_pose.pose.orientation.z;
  res.desired_robot_pose.pose.orientation.w = provide_target::desired_ee_pose.pose.orientation.w;
  
  // If shutdown_now is true, this node will shut down
  provide_target::shutdown_now = req.shutdown_service;

  return provide_target::shutdown_now;
}

///////
// MAIN
///////
int main(int argc, char **argv)
{

  ros::init(argc, argv, "provide_target");
  ros::NodeHandle nh;

  ////////////////////////////////
  // Show the pose we want in Rviz
  ////////////////////////////////

  ros::Publisher approachVisualizationPublisher = nh.advertise<visualization_msgs::Marker>("desired_ee_pose", 1);
  ros::Duration(1).sleep();

  provide_target::desired_ee_pose.header.frame_id = "base_link";
  provide_target::desired_ee_pose.header.stamp = ros::Time();

  // For this example, we'll use a meaningless default
  provide_target::desired_ee_pose.pose.position.x = 0.;  
  provide_target::desired_ee_pose.pose.position.y = 0.;  
  provide_target::desired_ee_pose.pose.position.z = 0.;

  provide_target::desired_ee_pose.pose.orientation.x = 0.;  
  provide_target::desired_ee_pose.pose.orientation.y = 0.;  
  provide_target::desired_ee_pose.pose.orientation.z = 0.;
  provide_target::desired_ee_pose.pose.orientation.w = 1.;

  // Cosmetic details for Rviz
  provide_target::desired_ee_pose.type = visualization_msgs::Marker::CYLINDER;
  provide_target::desired_ee_pose.scale.x = 0.09;
  provide_target::desired_ee_pose.scale.y = 0.04;
  provide_target::desired_ee_pose.scale.z = 0.22;
  provide_target::desired_ee_pose.color.a = 1.0;
  provide_target::desired_ee_pose.color.r = 0.0f;
  provide_target::desired_ee_pose.color.g = 1.0f;
  provide_target::desired_ee_pose.color.b = 0.0f;
  provide_target::desired_ee_pose.lifetime = ros::Duration();
  approachVisualizationPublisher.publish(provide_target::desired_ee_pose);

  // Start a service. Wait until another node requests the desired pose, then shut down (if requested).
  ros::ServiceServer service = nh.advertiseService("desired_robot_pose", desired_robot_pose);
  while ( !provide_target::shutdown_now && ros::ok() ) // Until the client(s) requests a shutdown
  {  
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  ros::shutdown();
  return 0;
}
