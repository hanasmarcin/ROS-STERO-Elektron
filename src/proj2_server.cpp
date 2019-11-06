#include "ros/ros.h"
#include "stero_mobile_init/Proj2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/layered_costmap.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_listener.h"
#include <ros/console.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include "dwa_local_planner/dwa_planner_ros.h"

using std::string;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using geometry_msgs::Twist;

using costmap_2d::Costmap2DROS;
using global_planner::GlobalPlanner;
using dwa_local_planner::DWAPlannerROS;
using std::unique_ptr;


void setStart(PoseStamped &start, tf2_ros::Buffer *tfBuffer)
{
    TransformStamped currentTf = tfBuffer->lookupTransform("base_link", "map", ros::Time::now(), ros::Duration(0.1));
    start.header.frame_id = "map";
    start.pose.position.x = currentTf.transform.translation.x;
    start.pose.position.y = currentTf.transform.translation.y;
    start.pose.position.z = currentTf.transform.translation.z;
    start.pose.orientation = currentTf.transform.rotation;
}

void setGoal(PoseStamped &goal, Pose &goalPose)
{
    goal.header.frame_id = "map";
    goal.pose = goalPose;
}

void recovery(Twist &twist, Twist &prevTwist, ros::Publisher &pub, ros::Rate &unsuccessfulRate)
{
    // Move backwards for 1.25s
    twist.linear.x = -0.1;
    pub.publish(twist);
    unsuccessfulRate.sleep();

    // Reset twist to default
    twist.linear.x = 0;

    // Rotate in opposite direction than the last non-default twist
    if (prevTwist.angular.z>=0)
        twist.angular.z = -0.2;
    else
        twist.angular.z = 0.2;
}

bool isGoalReached(DWAPlannerROS *localPlanner, Twist &prevTwist, int &unsuccessfulCount)
{
    return localPlanner->isGoalReached() || (unsuccessfulCount == 0 && prevTwist.linear.x == 0 && prevTwist.angular.z == 0);
}

tf2_ros::Buffer *tfBuffer;
tf2_ros::TransformListener *tfListener;
// Configure and initialize global costmap
Costmap2DROS *globalCostmap;
// Configure and initialize global planner
GlobalPlanner *globalPlanner;
// Configure and initialize DWA local plannertf2_ros::TransformListener
DWAPlannerROS *localPlanner;
// Configure and initialize local costmap
Costmap2DROS *localCostmap;

void initPlannersAndCostmaps(){
    tfBuffer = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    globalCostmap = new Costmap2DROS("global_costmap", *tfBuffer);
    globalPlanner = new GlobalPlanner();
    localPlanner = new DWAPlannerROS();
    localCostmap = new Costmap2DROS("local_costmap", *tfBuffer);
    // Configure transform buffer
    tfBuffer->setUsingDedicatedThread(true);

    globalPlanner->initialize("base_global_planner", globalCostmap->getCostmap(), "map");

    localPlanner->initialize("base_local_planner", tfBuffer, localCostmap);
}

bool getToPose(stero_mobile_init::Proj2::Request  &req,
               stero_mobile_init::Proj2::Response &res)
{


    PoseStamped start;
    setStart(start, tfBuffer);
    PoseStamped goal;
    setGoal(goal, req.goal);

    // Make global plan
    std::vector<PoseStamped> plan;
    globalPlanner->makePlan(start, goal, plan);


    localCostmap->start();
    localPlanner->setPlan(plan);

    // Create publisher to publish robot's velocities
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<Twist>("/elektron/mobile_base_controller/cmd_vel", 5);

    // Create variables used in the loop
    Twist twist;
    Twist prevTwist;
    int unsuccessfulCount = 1;
    ros::Rate rate(20);
    ros::Rate unsuccessfulRate(0.8);

    while(!isGoalReached(localPlanner, prevTwist, unsuccessfulCount))
    {
        globalPlanner->publishPlan(plan);
        ros::spinOnce(); // For laser_scan to be received by local costmap's obstacles layer
        tfBuffer->canTransform("odom", "map", ros::Time::now(), ros::Duration(0.01)); // Check if the transform is available, to avoid
                                                                                     // errors about transforms being set in the future

        bool isSuccessful = localPlanner->computeVelocityCommands(twist); // Use DWA local planner
                                                                         // Sets twist to calculated values or default values 
                                                                         // if twist wasn't able to be calculated
        
        if (!isSuccessful) // If planner couldn't compute velocities
            unsuccessfulCount++;
        else {
            unsuccessfulCount = 0;
            prevTwist = twist; // Save only successfully computed twists
        }
        
        if (unsuccessfulCount >= 10) // If planner wasn't able to calculate velocities more than 10 times in row
        {
            recovery(twist, prevTwist, pub, unsuccessfulRate);
            prevTwist = twist;
        }

        // Publish twist calculated by DWA planner or set by recovery mechanism 
        pub.publish(twist);

        // Wait for some time
        if (unsuccessfulCount >= 10) {
            int k;
            for(k=unsuccessfulCount; k>9; k--)
                unsuccessfulRate.sleep();
        }
        else
            rate.sleep();
    }

    // Finish the move
    geometry_msgs::Twist emptyTwist;
    pub.publish(emptyTwist);

    res.message = "Success";
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "proj2_server");
    initPlannersAndCostmaps();
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("proj2", getToPose);
    ros::spin();

    return 0;
}
