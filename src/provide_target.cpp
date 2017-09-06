
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_to_manip/desired_poseAction.h>
#include "ros/ros.h"
#include "temoto/Status.h"
#include "visualization_msgs/Marker.h"

namespace provide_target
{
  geometry_msgs::PoseStamped goal_pose;
}
using namespace provide_target;



void goal_pose_CB(const temoto::Status::ConstPtr& msg)
{
  goal_pose = msg->commanded_pose;
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "provide_target");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<move_base_to_manip::desired_poseAction> ac("move_base_to_manip", true);
  ROS_INFO("[provide_target] Waiting for the move_base_to_manip action server to start.");
  ac.waitForServer();

  // Listen to /temoto/status.commanded_pose to get the hand marker pose
  ros::Subscriber sub = n.subscribe("temoto/status", 1, goal_pose_CB);
  // Wait til we hear a pose from Temoto
  while ( goal_pose.header.frame_id == "" )
  {
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  move_base_to_manip::desired_poseGoal goal;
  goal.desired_pose = goal_pose;
  ROS_INFO("[provide_target] Sending goal.");
  ac.sendGoal(goal);

  return 0;
}
