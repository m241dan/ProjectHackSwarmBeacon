//
// Created by korisd on 11/21/18.
//

#ifndef BEHAVIOURS_ROVERMAP_H
#define BEHAVIOURS_ROVERMAP_H

//ROS Specific
#include <ros/ros.h>

//ROS Msgs
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <swarmie_msgs/GetOdom.h>
#include <swarmie_msgs/GetPosition.h>
#include <swarmie_msgs/SetOffset.h>
#include <swarmie_msgs/Beacon.h>
#include <swarmie_msgs/BeaconUpdate.h>
#include <swarmie_msgs/GetTopBeacon.h>
#include <swarmie_msgs/NewBeacon.h>
#include <swarmie_msgs/UpdateBeacon.h>
#include <swarmie_msgs/UninterestBeacon.h>
#include <std_srvs/Empty.h>

//STL
#include <string>
#include <vector>
#include <map>
#include <algorithm>

//Custom Libs
#include "RoverBeacon.h"

class RoverMap
{
    public:
        explicit RoverMap( ros::NodeHandle& nh, std::string name_of_rover );
        ~RoverMap();

    private:
        void odomHandler                ( const nav_msgs::Odometry::ConstPtr& msg );
        void ekfHandler                 ( const nav_msgs::Odometry::ConstPtr& msg );
        bool getPosition                ( swarmie_msgs::GetPositionRequest& req, swarmie_msgs::GetPositionResponse& res );
        bool getOdom                    ( swarmie_msgs::GetOdomRequest& req, swarmie_msgs::GetOdomResponse& res );
        bool setOffset                  ( swarmie_msgs::SetOffsetRequest& req, swarmie_msgs::SetOffsetResponse& res );

        void newBeaconHandler           ( const swarmie_msgs::Beacon::ConstPtr& beacon );
        void roverInterestedHandler     ( const swarmie_msgs::BeaconUpdate::ConstPtr& message );
        void cubesSeenHandler           ( const swarmie_msgs::BeaconUpdate::ConstPtr& message );

        bool newBeacon                  ( swarmie_msgs::NewBeaconRequest& req, swarmie_msgs::NewBeaconResponse& res );
        bool beaconUpdate               ( swarmie_msgs::UpdateBeaconRequest &req, swarmie_msgs::UpdateBeaconResponse& res );
        bool getTopBeacon               ( swarmie_msgs::GetTopBeaconRequest &req, swarmie_msgs::GetTopBeaconResponse& res );
        bool roverUninterestBeacon      ( swarmie_msgs::UninterestBeaconRequest &req, swarmie_msgs::UninterestBeaconResponse& res );
        bool clearTempBeacons           ( std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res );

        //ROS specific
        ros::NodeHandle&                node_handle;

        //ROS Publishers
        ros::Publisher                  new_beacon_pub;
        ros::Publisher                  rover_interested_pub;
        ros::Publisher                  cubes_seen_pub;

        //ROS Subscribers
        ros::Subscriber                 raw_odom;
        ros::Subscriber                 ekf;
        ros::Subscriber                 new_beacon_sub;
        ros::Subscriber                 rover_interested_sub;
        ros::Subscriber                 cubes_seen_sub;

        //ROS Services
        ros::ServiceServer              get_position;
        ros::ServiceServer              get_odom;
        ros::ServiceServer              set_offset;
        ros::ServiceServer              new_beacon_service;
        ros::ServiceServer              beacon_update_service;
        ros::ServiceServer              get_top_beacon;
        ros::ServiceServer              rover_uninterest_beacon;
        ros::ServiceServer              clear_temporary_beacons;

        //Poses
        geometry_msgs::Pose2D           present_odom;
        geometry_msgs::Pose2D           present_position;
        geometry_msgs::Pose2D           present_offset;

        //Beacon System
        std::vector<RoverBeacon>        beacon_heap;
        std::map<std::string,uint32_t>  beacon_map;
        uint32_t                        beacon_identifier;
        std::string                     present_beacon;

        //other
        std::string rover_name;
};


#endif //BEHAVIOURS_ROVERMAP_H
