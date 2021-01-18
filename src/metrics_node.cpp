#include <cxxopts.hpp>
#include <ros/ros.h>

#include "ascend_ros/mavlink.h"
#include "loguru.hpp"
#include "utilities.h"


void log_metric(ascend_ros::mavlink::ConstPtr mavlink_msg){
    LOG_S(INFO) << "MSG ID: " << mavlink_msg->id << "\n"
                << "----------------------------\n"
                << mavlink_msg->msg
                << "\n\n";
}

int main(int argc, char **argv)
{
    //initialize ros
    ros::init(argc, argv, "metrics");
    ros::NodeHandle nh;

    //initialize loguru
    std::string timestamp = utilities::timestamp();
    std::string filename = "~/drone_logs/"+timestamp+".log";
    bool create_succ = loguru::add_file( filename.c_str() , loguru::Truncate, loguru::Verbosity_INFO);
    if(!create_succ){
        ROS_FATAL_STREAM("Cannot create logfile " << filename);
    }

    //turn off output to stdout and stderr
    loguru::g_stderr_verbosity = loguru::Verbosity_OFF;

    ros::Subscriber sub = nh.subscribe("/metrics", 25, log_metric);

    //keep node alive
    ros::spin();

    return 0;
}