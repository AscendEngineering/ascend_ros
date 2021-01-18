#pragma once

#include <action/action.h>
#include <chrono>
#include <iostream>
#include <mavsdk.h>
#include <mavlink_passthrough/mavlink_passthrough.h>
#include <mission/mission.h>
#include <mutex>
#include <ros/ros.h>
#include <telemetry/telemetry.h>

#include "ascend_ros/mission_pointList.h"
#include "ascend_ros/mission_point.h"
#include "ascend_ros/mavlink.h"

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


class drone{
    public:
        drone(ros::NodeHandle* in_nh, bool simulation);
        ~drone();

        /*Searches and connects to an instance of px4 if available
            @param block -> (bool) block call to function until px4 is returned
            @return -> n/a
        */
        bool search_and_connect(int retries=5);

        /* Returns whether PX4 is connected or not
            @param -> n/a
            @return -> (bool) if px4 is connected
        */
        bool is_connected();

        /* Subscribes to mavlink health messages
            @param -> n/a
            @return -> (bool) if subscriptions was successful
        */
        bool subscribe_metrics();

        /* Unsubscribes all mavlink health messages if connect to px4
            @param -> n/a
            @return -> (bool) if unsubscription was successful
        */
        bool unsubscribe_metrics();

    private:
        
        //lock
        std::mutex mtx; 

        //mavsdk
        mavsdk::Mavsdk px4;
        std::shared_ptr<mavsdk::System> system;
        std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink;
        std::shared_ptr<mavsdk::Action> action;
        std::shared_ptr<mavsdk::Mission> mission;
        std::shared_ptr<mavsdk::Telemetry> telemetry;
        void initialize_action_objects();
        
        //ros
        ros::NodeHandle* nh;
        ros::Publisher metric_publisher;
        ros::Subscriber geofence_clear_handler;
        ros::Subscriber geofence_upload_handler;
        ros::Subscriber waypoint_clear_handler;
        ros::Subscriber waypoint_upload_handler;
        ros::Subscriber waypoint_start_mission_handler;
        ros::Subscriber rallypoint_clear_handler;
        ros::Subscriber rallypoint_upload_handler;
        ros::Subscriber arm_handler;

        //callbacks
        void waypoint_clear_cb(std_msgs::Bool::ConstPtr clear);
        void waypoint_upload_cb(ascend_ros::mission_pointList::ConstPtr point_list);
        void waypoint_mission_start_cb(std_msgs::Bool::ConstPtr start);
        void geofence_clear_cb(std_msgs::Bool::ConstPtr clear);
        void geofence_upload_cb(ascend_ros::mission_pointList::ConstPtr point_list);
        void rallypoint_clear_cb(std_msgs::Bool::ConstPtr clear);
        void rallypoint_upload_cb(ascend_ros::mission_pointList::ConstPtr point_list);
        void arm_cb(std_msgs::Bool::ConstPtr arm);
   
        //member
        bool m_simulation;
};