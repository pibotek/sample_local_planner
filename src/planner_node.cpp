#include "sample_local_planner/planner_node.h"
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(sample_local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace sample_local_planner {
LocalPlanner::LocalPlanner() {
}

#if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 14, 0)
void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
#else
void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
#endif
{
  ROS_INFO("LocalPlanner initialize");
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  ROS_INFO("LocalPlanner computeVelocityCommands");
  if (start_flag_) {
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0.8;
  } else {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
  }
  
  return true;
}

bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  ROS_INFO("LocalPlanner setPlan");

  if (!start_flag_) {
    start_flag_ = true;
    stopwatch_.reset();
  }

  return true;
}

bool LocalPlanner::isGoalReached() {
  if (stopwatch_.elapsed(std::chrono::seconds(2))) {
    ROS_INFO("LocalPlanner GoalReached");
    start_flag_ = false;
    return true;
  }

  return false;
}
}  // namespace sample_local_planner
