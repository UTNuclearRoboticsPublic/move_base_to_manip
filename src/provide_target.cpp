
#include "move_base_to_manip/desired_robot_pose.h" // Service that provides a desired pose
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

namespace provide_target
{
  visualization_msgs::Marker desired_ee_pose; // Stores the target pose
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


  // Cosmetic details for Rviz arrow
  // Head is at points.at(0), tail is at points.at(1)
  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.frame_id = "/base_link";
  arrow_marker.header.stamp = ros::Time();
  arrow_marker.id = 47;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;

  arrow_marker.scale.x = 0.02; // Shaft dia
  arrow_marker.scale.y = 0.04;  // Head dia
  arrow_marker.color.r = 0.0;  
  arrow_marker.color.g = 1.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.a = 1.0;

  arrow_marker.points.resize(2);
  // Head of the arrow
  arrow_marker.points[1] = provide_target::desired_ee_pose.pose.position;
  // Tail of the arrow
  arrow_marker.points[0].x = provide_target::desired_ee_pose.pose.position.x;
  arrow_marker.points[0].y = provide_target::desired_ee_pose.pose.position.y;
  arrow_marker.points[0].z = provide_target::desired_ee_pose.pose.position.z+0.25;

  provide_target::desired_ee_pose.lifetime = ros::Duration();
approachVisualizationPublisher.publish(arrow_marker);

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
