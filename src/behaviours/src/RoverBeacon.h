//
// Created by korisd on 12/10/18.
//

#ifndef BEHAVIOURS_ROVERBEACON_H
#define BEHAVIOURS_ROVERBEACON_H

//ROS Msgs
#include <geometry_msgs/Pose2D.h>
#include <swarmie_msgs/Beacon.h>


#include "RoverUtilities.h"

const uint16_t cube_value   = 5;
const uint16_t meter_value  = cube_value;
const uint16_t rover_value  = cube_value * 3;
const uint16_t arena_size   = 22; //max search area

class RoverBeacon
{
    public:
        RoverBeacon( std::string id, geometry_msgs::Pose2D pos, geometry_msgs::Pose2D &rp );
        RoverBeacon( const swarmie_msgs::Beacon& beacon, geometry_msgs::Pose2D &rp );
        RoverBeacon( RoverBeacon&& o ) noexcept;

        int32_t                     getWeight() const;
        geometry_msgs::Pose2D       getPosition();
        void                        addRoverInterested();
        void                        remRoverInterested();
        void                        updateBeacon( const swarmie_msgs::BeaconConstPtr& beacon );
        void                        setCubes( uint16_t cubes );
        void                        remCube();
        bool                        isTemporary();
        bool                        operator<( const RoverBeacon& beacon );
        RoverBeacon&                operator=( RoverBeacon&& other ) noexcept;
        std::string                 getIdentifier();
        swarmie_msgs::Beacon        toMessage();

    private:
        std::string                 identifier;
        geometry_msgs::Pose2D       beacon_position;
        geometry_msgs::Pose2D&      rover_position;
        uint16_t                    num_of_cubes;
        uint8_t                     num_of_rovers_interested;
        int32_t                     weight;
        bool                        temporary;


};


#endif //BEHAVIOURS_ROVERBEACON_H
