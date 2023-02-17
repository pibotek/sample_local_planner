#ifndef SAMPLE_LOCAL_PLANNER_H_
#define SAMPLE_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include "sample_local_planner/stopwatch.h"

namespace sample_local_planner
{
    class LocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        LocalPlanner();

#if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 14, 0)
        void initialize(std::string name, tf2_ros::Buffer* tf, 
                        costmap_2d::Costmap2DROS* costmap_ros);
#else
        void initialize(std::string name, tf::TransformListener *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);
#endif

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool isGoalReached();

    private:
        
        // ros::Publisher plan_pub_;
        // std::string frame_id_;
        Stopwatch stopwatch_;

        bool start_flag_ = false;
    };
}

#endif