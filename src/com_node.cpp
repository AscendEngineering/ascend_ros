#include <cxxopts.hpp>
#include <ros/ros.h>
#include "utilities.h"
#include 


int main(int argc, char **argv)
{
    //initialize ros
    ros::init(argc, argv, "coms");
    ros::NodeHandle nh;

    //register 

    //keep node alive
    ros::spin();

    return 0;
}