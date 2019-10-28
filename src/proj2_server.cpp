#include "ros/ros.h"
#include "stero_mobile_init/Proj2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_listener.h"
#include <ros/console.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>

using std::string;


bool add(stero_mobile_init::Proj2::Request  &req,
         stero_mobile_init::Proj2::Response &res)
{

    geometry_msgs::Pose goal = req.goal;

    res.message = "xd";
    return true;
}


// void sendTransform() 
// {
//     static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//     geometry_msgs::TransformStamped tf;

//     tf.header.stamp = ros::Time::now();
//     tf.header.frame_id = "base_link";
//     tf.child_frame_id = "odom";
//     tf.transform.translation.x = 0;
//     tf.transform.translation.y = 0;
//     tf.transform.translation.z = 0;
//     tf2::Quaternion quat;
//     quat.setRPY(0, 0, 0);
//     tf.transform.rotation.x = quat.x();
//     tf.transform.rotation.y = quat.y();
//     tf.transform.rotation.z = quat.z();
//     tf.transform.rotation.w = quat.w();
//     static_broadcaster.sendTransform(tf);
// }

void configureGlobalCostmap() 
{   
    // sendTransform();
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tfBuffer.setUsingDedicatedThread(true);
    string mapName = "";
    costmap_2d::Costmap2DROS costmap("global_costmap", tfBuffer);
    // ROS_ERROR("xd", "FINISHED");
    

    string globalPlannerName = "base_global_planner";
    global_planner::GlobalPlanner globalPlanner;
    globalPlanner.initialize("base_global_planner", &costmap);
    geometry_msgs::PoseStamped start;
    start.header.frame_id = "map";
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 5;
    std::vector<geometry_msgs::PoseStamped> plan;
    bool wasSuccessful = globalPlanner.makePlan(start, goal, plan);
    char xd[20];
    std::sprintf(xd, "%d elementow",plan.size());
    if(wasSuccessful)
        ROS_ERROR(xd);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "proj2_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("proj2", add);
    configureGlobalCostmap();
    ros::spin();

    return 0;
}