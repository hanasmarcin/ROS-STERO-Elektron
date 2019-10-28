#include "ros/ros.h"
#include "stero_mobile_init/Proj2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"

using std::string;


bool add(stero_mobile_init::Proj2::Request  &req,
         stero_mobile_init::Proj2::Response &res)
{

    geometry_msgs::Pose goal = req.goal;

    res.message = "xd";
    return true;
}

void configureCostmap() 
{   
    try {
        tf2_ros::Buffer tf2Buffer;
        string mapName = "map";
        costmap_2d::Costmap2DROS costmap(mapName, tf2Buffer);
    } catch (std::exception& e) {

    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proj2_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("proj2", add);
    configureCostmap();
    ros::spin();

    return 0;
}