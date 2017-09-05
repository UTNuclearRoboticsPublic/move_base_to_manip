
#include <actionlib/client/simple_action_client.h>
#include <move_base_to_manip/desired_poseAction.h>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "provide_target");

  actionlib::SimpleActionClient<move_base_to_manip::desired_poseAction> ac("move_base_to_manip", true);

  ROS_INFO("[provide_target] Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_link";

  move_base_to_manip::desired_poseGoal goal;
  goal.desired_pose = goal_pose;
  ac.sendGoal(goal);

  return 0;
}
