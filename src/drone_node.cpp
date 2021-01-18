#include <cxxopts.hpp>
#include <drone.h>
#include <ros/ros.h>
#include <mavsdk.h>
#include <mavlink_passthrough/mavlink_passthrough.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ea");
    ros::NodeHandle nh;

    cxxopts::Options options("drone_node", "Node that drives the drone");
    options.add_options()
        ("s,simulation", "Connect to simulation");
    auto result = options.parse(argc, argv);
    bool simulation = false;
    if(result.count("simulation")){
        simulation = result["simulation"].as<bool>();
    }

    //declare drone
    drone ascend_drone(&nh,simulation);
    bool succ = ascend_drone.search_and_connect(true);
    if(!succ){
        ROS_FATAL("Error finding drone");
        return 1;
    }

    ascend_drone.subscribe_metrics();

    //keep node alive
    ros::spin();

    return 0;
}
