//
// Created by korisd on 12/10/18.
//

#include "RoverBeacon.h"

RoverBeacon::RoverBeacon( std::string id, geometry_msgs::Pose2D pos, geometry_msgs::Pose2D &rp ) :
        identifier( std::move( id )), beacon_position( pos ),
        rover_position( rp )
{

}

RoverBeacon::RoverBeacon( const swarmie_msgs::Beacon &beacon, geometry_msgs::Pose2D &rp ) : identifier(
                                                                                                    beacon.identifier ),
                                                                                            beacon_position(
                                                                                                    beacon.position ),
                                                                                            num_of_cubes(
                                                                                                    beacon.num_of_cubes ),
                                                                                            num_of_rovers_interested(
                                                                                                    beacon.num_of_rovers_interested ),
                                                                                            weight( beacon.weight ),
                                                                                            temporary(
                                                                                                    beacon.temporary ),
                                                                                            rover_position( rp )
{

}


RoverBeacon::RoverBeacon( RoverBeacon &&o ) noexcept : identifier( std::move(o.identifier) ),
                                                       beacon_position( o.beacon_position ),
                                                       rover_position( o.rover_position ),
                                                       num_of_cubes( o.num_of_cubes ),
                                                       num_of_rovers_interested( o.num_of_rovers_interested ),
                                                       weight( o.weight ),
                                                       temporary( o.temporary )
{

}

int32_t RoverBeacon::getWeight() const
{
    //handle the intrinsic weight of the beacon
    int32_t beacon_weight = weight;

    //distance portion
    beacon_weight += ( arena_size * meter_value );
    beacon_weight -= static_cast<uint16_t>( static_cast<double>( meter_value ) * hypot( beacon_position.x, beacon_position.y ) ); // RoverUtilities::Math::distance( rover_position, beacon_position ) );

    //cube portion
    beacon_weight += num_of_cubes * cube_value;

    //rover portion
    beacon_weight -= num_of_rovers_interested * rover_value;

    return beacon_weight;
}

geometry_msgs::Pose2D RoverBeacon::getPosition()
{
    return beacon_position;
}

void RoverBeacon::addRoverInterested()
{
    num_of_rovers_interested++;
}

void RoverBeacon::remRoverInterested()
{
    if( num_of_rovers_interested != 0 )
        num_of_rovers_interested--;
}

void RoverBeacon::updateBeacon( const swarmie_msgs::BeaconConstPtr &beacon )
{
    if( beacon->identifier == identifier )
    {
        num_of_cubes                = beacon->num_of_cubes;
        num_of_rovers_interested    = beacon->num_of_rovers_interested;
        weight                      = beacon->weight;
    }
}

void RoverBeacon::setCubes( uint16_t cubes )
{
    num_of_cubes = cubes;
}

void RoverBeacon::remCube()
{
    if( num_of_cubes != 0 )
        num_of_cubes--;
}

bool RoverBeacon::isTemporary()
{
    return temporary;
}

bool RoverBeacon::operator<( const RoverBeacon &beacon )
{
    return this->getWeight() < beacon.getWeight();
}

RoverBeacon& RoverBeacon::operator=( RoverBeacon&& other ) noexcept
{
    identifier = std::move(other.identifier);
    beacon_position = other.beacon_position;
    rover_position = other.rover_position;
    num_of_cubes = other.num_of_cubes;
    num_of_rovers_interested = other.num_of_rovers_interested;
    weight = other.weight;
    temporary = other.temporary;
    return *this;
}

std::string RoverBeacon::getIdentifier()
{
    return identifier;
}

swarmie_msgs::Beacon RoverBeacon::toMessage()
{
    swarmie_msgs::Beacon msg;
    msg.identifier = identifier;
    msg.num_of_cubes = num_of_cubes;
    msg.num_of_rovers_interested = num_of_rovers_interested;
    msg.temporary = temporary ? static_cast<unsigned  char>(1) : static_cast<unsigned char>(0);
    msg.weight = weight;
    msg.position = beacon_position;
    return msg;
}