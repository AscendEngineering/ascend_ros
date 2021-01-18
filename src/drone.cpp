#include <geofence/geofence.h>
#include <thread>
#include <ros/ros.h>
#include <sstream>

#include "drone.h"
#include "utilities.h"


namespace {

    //30 second wait for register
    bool wait_for_px4_register(mavsdk::Mavsdk& px4){
        for(int i =0; i< 30; i++ ) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if(px4.systems().size() > 0){
                return true;
            }
        }
        return false;
    }
}

static mavsdk::Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    mavsdk::Mission::MissionItem::CameraAction camera_action);

drone::drone(ros::NodeHandle* in_nh, bool simulation){
    nh = in_nh;
    m_simulation = simulation;

    //initialize ros
    metric_publisher = nh->advertise<ascend_ros::mavlink>("/metrics",1,true);
    geofence_clear_handler = nh->subscribe("/geofence/clear",1,&drone::geofence_clear_cb,this);
    geofence_upload_handler = nh->subscribe("/geofence/geofence_upload",1,&drone::geofence_upload_cb,this);
    waypoint_clear_handler = nh->subscribe("/waypoint/clear",1,&drone::waypoint_clear_cb,this);
    waypoint_start_mission_handler = nh->subscribe("/waypoint/start_mission", 1, &drone::waypoint_mission_start_cb, this);
    waypoint_upload_handler = nh->subscribe("/waypoint/waypoint_upload", 1, &drone::waypoint_upload_cb, this);
    arm_handler = nh->subscribe("/arm",1,&drone::arm_cb,this);
    rallypoint_clear_handler = nh->subscribe("/rallypoint/clear",1,&drone::rallypoint_clear_cb,this);
    rallypoint_upload_handler = nh->subscribe("/rallypoint/rallypoint_upload",1,&drone::rallypoint_upload_cb,this);

}

drone::~drone(){
    unsubscribe_metrics();
}

bool drone::search_and_connect(int retries){

    //poll for px4
    int found = false;
    for(int i =0; i < retries; i++){
        mavsdk::ConnectionResult conn_result;

        if(m_simulation){
            conn_result = px4.add_udp_connection("",14540);
        }
        else{
            conn_result = px4.add_serial_connection("/dev/ttyTHS1");
        }
        
        if(conn_result==mavsdk::ConnectionResult::Success){
            bool registered = wait_for_px4_register(px4);
            if(registered && px4.systems()[0]->get_uuid() != 0){
                found = true;
                break;
            }
        }
        else{
            std::this_thread::sleep_for(std::chrono::seconds(5));
            continue;
        }
    }

    if(!found){
        return false;
    }

    //connect instance
    system = px4.systems()[0];

    //log success
    ROS_INFO_STREAM("Connected to PX4: " << system->get_uuid());
    initialize_action_objects();
    
    return true;
}

bool drone::is_connected(){
    if(system!=nullptr){
        return system->is_connected();
    }
    else {
        return false;
    }
}

bool drone::subscribe_metrics(){

    //sanity
    if(system==nullptr){
        return false;
    }

    //MAV_SYS_STATUS_SENSOR
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_SYS_STATUS,[&](const mavlink_message_t& message){
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(&message,&sys_status);
        std::stringstream msg_output;
        msg_output << "onboard_control_sensors_present: " << utilities::bitmap_to_str(sys_status.onboard_control_sensors_present) << " ";
        msg_output << "onboard_control_sensors_enabled: " << utilities::bitmap_to_str(sys_status.onboard_control_sensors_enabled) << " ";
        msg_output << "onboard_control_sensors_health: " << utilities::bitmap_to_str(sys_status.onboard_control_sensors_health) << " ";
        msg_output << "load: "<< sys_status.load << " ";
        msg_output << "voltage_battery: " << sys_status.voltage_battery << " ";
        msg_output << "current_battery: "<< sys_status.current_battery << " ";
        msg_output << "drop_rate_comm: "<< sys_status.drop_rate_comm << " ";
        msg_output << "errors_comm: " << sys_status.errors_comm << " ";
        msg_output << "errors_count1: " << sys_status.errors_count1 << " ";
        msg_output << "errors_count2: " << sys_status.errors_count2 << " ";
        msg_output << "errors_count3: " << sys_status.errors_count3 << " ";
        msg_output << "errors_count4: " << sys_status.errors_count4 << " ";
        msg_output << "battery_remaining: " << sys_status.battery_remaining << " ";

        ascend_ros::mavlink status_msg;
        status_msg.id =  MAVLINK_MSG_ID_SYS_STATUS;
        status_msg.msg = msg_output.str();
        metric_publisher.publish(status_msg);
        
    });

    //FENCE_BREACH & FENCE_MITIGATE
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_FENCE_STATUS,[&](const mavlink_message_t& message){
        mavlink_fence_status_t fence_status;
        mavlink_msg_fence_status_decode(&message,&fence_status);
        std::stringstream msg_output;
        msg_output << "breach_status: " << fence_status.breach_status << " ";
        msg_output << "breach_count: " << fence_status.breach_count << " ";
        msg_output << "breach_type: " << fence_status.breach_type << " ";
        msg_output << "breach_time: " << fence_status.breach_time << " ";
        msg_output << "breach_mitigation: " << fence_status.breach_mitigation << " ";

        ascend_ros::mavlink status_msg;
        status_msg.id =  MAVLINK_MSG_ID_FENCE_STATUS;
        status_msg.msg = msg_output.str();
        metric_publisher.publish(status_msg);
    });

    //ESC_FAILURE_FLAGS
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_ESC_INFO,[&](const mavlink_message_t& message){
        mavlink_esc_info_t esc_info;
        mavlink_msg_esc_info_decode(&message,&esc_info);
        std::stringstream msg_output;
        msg_output << "index: " << esc_info.index << " ";
        msg_output << "time_usec: " << esc_info.time_usec << " ";
        msg_output << "counter: " << esc_info.counter << " ";
        msg_output << "count: " << esc_info.count << " ";
        msg_output << "connection_type: " << esc_info.connection_type << " ";
        msg_output << "info: " << esc_info.info << " ";
        msg_output << "failure_flags: " << esc_info.failure_flags << " ";
        msg_output << "error_count: " << esc_info.error_count << " ";
        msg_output << "temperature: " << esc_info.temperature << " ";

        ascend_ros::mavlink status_msg;
        status_msg.id =  MAVLINK_MSG_ID_ESC_INFO;
        status_msg.msg = msg_output.str();
        metric_publisher.publish(status_msg);
    });

    //POWER_STATUS
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_POWER_STATUS,[&](const mavlink_message_t& message){
        mavlink_power_status_t power_status;
        mavlink_msg_power_status_decode(&message,&power_status);
        std::stringstream msg_output;
        msg_output << "Vcc: " << power_status.Vcc << " ";
        msg_output << "Vservo: " << power_status.Vservo << " ";
        msg_output << "flags: " << utilities::bitmap_to_str(power_status.flags) << " ";

        ascend_ros::mavlink status_msg;
        status_msg.id =  MAVLINK_MSG_ID_POWER_STATUS;
        status_msg.msg = msg_output.str();
        metric_publisher.publish(status_msg);
    });

    //BATTERY_STATUS
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_BATTERY_STATUS,[&](const mavlink_message_t& message){
        mavlink_battery_status_t battery_status;
        mavlink_msg_battery_status_decode(&message,&battery_status);
        std::stringstream msg_output;
        msg_output << "id: " << battery_status.id << " ";
        msg_output << "battery_function: " << battery_status.battery_function << " ";
        msg_output << "type: " << battery_status.type << " ";
        msg_output << "temperature: " << battery_status.temperature << " ";
        msg_output << "voltages: " << battery_status.voltages << " ";
        msg_output << "current_battery: " << battery_status.current_battery << " ";
        msg_output << "current_consumed: " << battery_status.current_consumed << " ";
        msg_output << "energy_consumed: " << battery_status.energy_consumed << " ";
        msg_output << "battery_remaining: " << battery_status.battery_remaining << " ";
        msg_output << "time_remaining: " << battery_status.time_remaining << " ";
        msg_output << "charge_state: " << battery_status.charge_state << " ";
        msg_output << "voltages_ext: " << battery_status.voltages_ext << " ";
        msg_output << "mode: " << battery_status.mode << " ";
        msg_output << "fault_bitmask: " << utilities::bitmap_to_str(battery_status.fault_bitmask) << " ";

        ascend_ros::mavlink status_msg;
        status_msg.id =  MAVLINK_MSG_ID_BATTERY_STATUS;
        status_msg.msg = msg_output.str();
        metric_publisher.publish(status_msg);
    });

    //ATTITUDE
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_ATTITUDE,[&](const mavlink_message_t& message){
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&message,&attitude);
        std::stringstream msg_output;
        msg_output << "time_boot_ms: " << attitude.time_boot_ms << " ";
        msg_output << "roll: " << attitude.roll << " ";
        msg_output << "pitch: " << attitude.pitch << " ";
        msg_output << "yaw: " << attitude.yaw << " ";
        msg_output << "rollspeed: " << attitude.rollspeed << " ";
        msg_output << "pitchspeed: " << attitude.pitchspeed << " ";
        msg_output << "yawspeed: " << attitude.yawspeed << " ";

        ascend_ros::mavlink status_msg;
        status_msg.id =  MAVLINK_MSG_ID_ATTITUDE;
        status_msg.msg = msg_output.str();
        metric_publisher.publish(status_msg);
    });

    return true;
}

bool drone::unsubscribe_metrics(){

    //sanity
    if(system==nullptr){
        return false;
    }

    //unsubscribe
    if(system->is_connected()){
        mavlink->subscribe_message_async(MAVLINK_MSG_ID_SYS_STATUS,nullptr);
        mavlink->subscribe_message_async(MAVLINK_MSG_ID_FENCE_STATUS,nullptr);
        mavlink->subscribe_message_async(MAVLINK_MSG_ID_ESC_INFO,nullptr);
        mavlink->subscribe_message_async(MAVLINK_MSG_ID_POWER_STATUS,nullptr);
        mavlink->subscribe_message_async(MAVLINK_MSG_ID_BATTERY_STATUS,nullptr);
        mavlink->subscribe_message_async(MAVLINK_MSG_ID_ATTITUDE,nullptr);
    }
    return true;
}

void drone::initialize_action_objects(){

    mavlink = std::make_shared<mavsdk::MavlinkPassthrough>(*system);
    action = std::make_shared<mavsdk::Action>(*system);
    mission = std::make_shared<mavsdk::Mission>(*system);
    telemetry = std::make_shared<mavsdk::Telemetry>(*system);
}

void drone::waypoint_clear_cb(std_msgs::Bool::ConstPtr clear){
    if(clear->data == true){
        mtx.lock();
        mission->clear_mission();
        mtx.unlock();
        ROS_INFO("Missions cleared");
    }
}

mavsdk::Mission::MissionItem make_mission_item(double latitude_deg,double longitude_deg,float relative_altitude_m,float speed_m_s,bool is_fly_through,float gimbal_pitch_deg,float gimbal_yaw_deg,mavsdk::Mission::MissionItem::CameraAction camera_action){
    mavsdk::Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

void drone::waypoint_upload_cb(ascend_ros::mission_pointList::ConstPtr point_list){
    //push back waypoints
    std::vector<mavsdk::Mission::MissionItem> mission_items;
    
    mavsdk::Telemetry::Position position = telemetry->position();

    mission_items.push_back(make_mission_item(
    position.latitude_deg,
    position.longitude_deg,
    point_list->mission_points[0].z_alt,
    5.0f,
    false,
    20.0f,
    60.0f,
    mavsdk::Mission::MissionItem::CameraAction::None));
    
    for(unsigned int i = 1; i < point_list->mission_points.size(); i++){
        mission_items.push_back(make_mission_item(
        point_list->mission_points[i].x_lat,
        point_list->mission_points[i].y_long,
        point_list->mission_points[i].z_alt,
        5.0f,
        false,
        20.0f,
        60.0f,
        mavsdk::Mission::MissionItem::CameraAction::None));
    }
    mavsdk::Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;
    mission->upload_mission(mission_plan);
    ROS_INFO("Mission uploaded.");
}

void drone::waypoint_mission_start_cb(std_msgs::Bool::ConstPtr start){
    if(start->data){
        mtx.lock();
        mission->subscribe_mission_progress( [](mavsdk::Mission::MissionProgress mission_progress) {
        ROS_INFO_STREAM("Mission status update: " << mission_progress.current << " / " << mission_progress.total);
        });
        mission->start_mission();
        mtx.unlock();
        while (true) {
            mtx.lock();
            if(mission->is_mission_finished().second){
                mtx.unlock();
                break;
            }
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        ROS_INFO("Mission Finished");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ROS_INFO("Landing...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        //land
        mtx.lock();
        const mavsdk::Action::Result land_result = action->land();
        mtx.unlock();

        //check land cmd
        if(land_result != mavsdk::Action::Result::Success){
            ROS_WARN_STREAM("Landing Denied: " << land_result);
        }

        //wait for land
        while (true){
            mtx.lock();
            if(!(telemetry->armed() && telemetry->in_air())){
                mtx.unlock();
                break;
            }
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        ROS_INFO("Disarmed and Landed");
        mtx.unlock();
    }
}

void drone::geofence_upload_cb(ascend_ros::mission_pointList::ConstPtr point_list){

    std::vector< mavsdk::Geofence::Point> geofence_points;
    for(unsigned int i = 0; i < point_list->mission_points.size(); i++){
        mavsdk::Geofence::Point point;
        point.latitude_deg = point_list->mission_points[i].x_lat;
        point.longitude_deg = point_list->mission_points[i].y_long;
        geofence_points.push_back(point);
        std::cout << point.latitude_deg << " " << point.longitude_deg << std::endl;
    }

    //create polyon
    mavsdk::Geofence::Polygon geofence;

    //set exclusion/inclusion
    if(point_list->param1 == "exclusion"){
        geofence.fence_type = mavsdk::Geofence::Polygon::FenceType::Exclusion;
    }
    else{
        geofence.fence_type = mavsdk::Geofence::Polygon::FenceType::Inclusion;
    }

    //add to vector
    geofence.points = geofence_points;
    std::vector< mavsdk::Geofence::Polygon > polygons;
    polygons.push_back(geofence);

    //send to drone
    std::shared_ptr<mavsdk::Geofence> geofence_loader = std::make_shared<mavsdk::Geofence>(*system);
    mtx.lock();
    auto succ = geofence_loader->upload_geofence(polygons);
    ROS_INFO_STREAM("Geofence Upload: " << succ);
    mtx.unlock();
}

void drone::geofence_clear_cb(std_msgs::Bool::ConstPtr clear){
    
    if(clear->data == true){
        mtx.lock();

        //create clear message
        mavlink_mission_clear_all_t clear_msg;
        clear_msg.mission_type = MAV_MISSION_TYPE::MAV_MISSION_TYPE_FENCE;
        clear_msg.target_system = mavlink->get_target_sysid();
        clear_msg.target_component = mavlink->get_target_compid();
        mavlink_message_t snd_msg;
        mavlink_msg_mission_clear_all_encode(mavlink->get_target_sysid(),mavlink->get_target_compid(),&snd_msg,&clear_msg);
        auto result = mavlink->send_message(snd_msg);

        //error checking
        if(result != mavsdk::MavlinkPassthrough::Result::Success){
            ROS_WARN_STREAM("Fence clear error: " << result);
        }

        ROS_INFO("All Geofences Cleared");
        mtx.unlock();
    }
}

void drone::arm_cb(std_msgs::Bool::ConstPtr arm){
    if(arm->data == true){
        mtx.lock();
        action->arm();
        mtx.unlock();
        ROS_INFO("Armed");
    }
}

void drone::rallypoint_clear_cb(std_msgs::Bool::ConstPtr clear){
    if(clear->data == true){
        mtx.lock();

        //create clear message
        mavlink_mission_clear_all_t clear_msg;
        clear_msg.mission_type = MAV_MISSION_TYPE::MAV_MISSION_TYPE_RALLY;
        clear_msg.target_system = mavlink->get_target_sysid();
        clear_msg.target_component = mavlink->get_target_compid();
        mavlink_message_t snd_msg;
        mavlink_msg_mission_clear_all_encode(mavlink->get_target_sysid(),mavlink->get_target_compid(),&snd_msg,&clear_msg);
        auto result = mavlink->send_message(snd_msg);

        //error checking
        if(result != mavsdk::MavlinkPassthrough::Result::Success){
            ROS_WARN_STREAM("Rally clear error: " << result);
        }

        ROS_INFO("All Rallypoints Cleared");
        mtx.unlock();
    }

}

void drone::rallypoint_upload_cb(ascend_ros::mission_pointList::ConstPtr rallypoint_list){
    mtx.lock();

    int target_system = mavlink->get_target_sysid();
    int target_comp = mavlink->get_target_compid();

    //create a vector of mission_int
    std::vector<mavlink_message_t> rally_points;
    for(unsigned int i = 0; i < rallypoint_list->mission_points.size(); i++){

        //form rally msg
        mavlink_mission_item_int_t rally_point;
        rally_point.target_system = target_system;
        rally_point.target_component = target_comp;
        rally_point.seq = i;
        rally_point.frame = MAV_FRAME::MAV_FRAME_GLOBAL;
        rally_point.command = MAV_CMD_NAV_RALLY_POINT;
        rally_point.x = rallypoint_list->mission_points[i].x_lat * 10000000;
        rally_point.y = rallypoint_list->mission_points[i].y_long * 10000000;
        rally_point.z = 0;
        rally_point.mission_type = MAV_MISSION_TYPE::MAV_MISSION_TYPE_RALLY;

        //add to list
        mavlink_message_t cnvrted;
        mavlink_msg_mission_item_int_encode(target_system,target_comp,&cnvrted,&rally_point);
        rally_points.push_back(cnvrted);
    }

    //set up subscriber for MISSION_REQUEST_INT (bool next_rallypoint to true if we need to send next item)
    bool upload_complete = false;
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_MISSION_ACK,[&](const mavlink_message_t& message){
        mavlink_mission_ack_t mission_ack;
        mavlink_msg_mission_ack_decode(&message,&mission_ack);
        if(mission_ack.mission_type == MAV_MISSION_TYPE::MAV_MISSION_TYPE_RALLY){
            upload_complete = true;
        }
    });
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST_INT,[&](const mavlink_message_t& message){
        mavlink_mission_request_int_t mission_request;
        mavlink_msg_mission_request_int_decode(&message,&mission_request);
        int seq = mission_request.seq;
        mavlink->send_message(rally_points[seq]);
    });
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST,[&](const mavlink_message_t& message){
        mavlink_mission_request_t mission_request;
        mavlink_msg_mission_request_decode(&message,&mission_request);
        int seq = mission_request.seq;
        mavlink->send_message(rally_points[seq]);
    });

    //initiate upload
    mavlink_mission_count_t upload_initiator;
    upload_initiator.target_system = target_system;
    upload_initiator.target_component = target_comp;
    upload_initiator.count = rallypoint_list->mission_points.size();
    upload_initiator.mission_type = MAV_MISSION_TYPE::MAV_MISSION_TYPE_RALLY;
    mavlink_message_t cnvrted;
    mavlink_msg_mission_count_encode(target_system,target_comp,&cnvrted,&upload_initiator);
    mavlink->send_message(cnvrted);

    //wait to finish
    while(!upload_complete){}

    mavlink->subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST_INT,nullptr);
    mavlink->subscribe_message_async(MAVLINK_MSG_ID_MISSION_ACK,nullptr);
    
    ROS_INFO("Rally Points Uploaded");

    mtx.unlock();
}


