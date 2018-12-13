//
// Created by korisd on 12/10/18.
//

#include "RoverBeacon.h"
#include <ros/ros.h>
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


//RoverBeacon::RoverBeacon( RoverBeacon &&o ) noexcept : identifier( std::move(o.identifier) ),
//                                                       beacon_position( o.beacon_position ),
//                                                       rover_position( o.rover_position ),
//                                                       num_of_cubes( o.num_of_cubes ),
//                                                       num_of_rovers_interested( o.num_of_rovers_interested ),
//                                                       weight( o.weight ),
//                                                       temporary( o.temporary )
//{
//
//}

int32_t RoverBeacon::getWeight() const
{
    //handle the intrinsic weight of the beacon
    int32_t beacon_weight = weight;

    if( num_of_cubes != 0 )
    {
        //distance portion
        beacon_weight += (arena_size * meter_value);
        beacon_weight -= static_cast<uint16_t>( static_cast<double>( meter_value ) * hypot( beacon_position.x,
                                                                                            beacon_position.y )); // RoverUtilities::Math::distance( rover_position, beacon_position ) );

        //cube portion
        beacon_weight += num_of_cubes * cube_value;

        //rover portion
        beacon_weight -= num_of_rovers_interested * rover_value;
    }
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

uint16_t RoverBeacon::getCubes()
{
    return num_of_cubes;
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

//RoverBeacon& RoverBeacon::operator=( const RoverBeacon& other )
//{
//    identifier = other.identifier;
//    beacon_position = other.beacon_position;
//    rover_position = other.rover_position;
//    num_of_cubes = other.num_of_cubes;
//    num_of_rovers_interested = other.num_of_rovers_interested;
//    weight = other.weight;
//    temporary = other.temporary;
//    return *this;
//}
//
//RoverBeacon& RoverBeacon::operator=( RoverBeacon&& other ) noexcept
//{
//    identifier = std::move(other.identifier);
//    beacon_position = other.beacon_position;
//    rover_position = other.rover_position;
//    num_of_cubes = other.num_of_cubes;
//    num_of_rovers_interested = other.num_of_rovers_interested;
//    weight = other.weight;
//    temporary = other.temporary;
//    return *this;
//}

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

void BeaconUtilities::heapPush( BeaconUtilities::BeaconHeap &heap, BeaconUtilities::BeaconMap &map, RoverBeacon beacon )
{
    heap.push_back( beacon );

    uint64_t i = heap.size() - 1;
    map.insert( std::pair<std::string, uint64_t>( beacon.getIdentifier(), i ));
    if( heap.size() > 1 )
    {
        ROS_INFO( "(i-1)/2: %d", heap[std::floor((i - 1) / 2)].getWeight());
        ROS_INFO( "i      : %d", heap[i].getWeight());
    }

    while( i > 0 and ( heap[std::floor((i-1)/2)].getWeight() < heap[i].getWeight() ) )
    {
        heapSwap( heap, map, heap[std::floor((i-1)/2)].getIdentifier(), heap[i].getIdentifier() );
        i = static_cast<uint64_t>(std::floor((i-1)/2));
        ROS_INFO( "calling swap..." );
    }

}

void BeaconUtilities::heapWeightUp( BeaconUtilities::BeaconHeap &heap, BeaconUtilities::BeaconMap &map,
                                   std::string beacon_identifier )
{
    uint64_t i = map[beacon_identifier];

    while( i > 0 and ( heap[(i-1)/2].getWeight() < heap[i].getWeight() ) )
    {
        heapSwap( heap, map, heap[(i-1)/2].getIdentifier(), heap[i].getIdentifier() );
        i = static_cast<uint64_t>(std::floor((i-1)/2));
    }

}

//essentially a max_heapify
void BeaconUtilities::heapWeightDown( BeaconUtilities::BeaconHeap &heap, BeaconUtilities::BeaconMap &map,
                                     std::string beacon_identifier )
{
    uint64_t i = map[beacon_identifier];
    uint64_t left = 2*i + 1;
    uint64_t right = 2*i + 2;
    uint64_t largest = i;

    if( left <= heap.size() and heap[left].getWeight() > heap[largest].getWeight() )
        largest = left;

    if( right <= heap.size() and heap[right].getWeight() > heap[largest].getWeight() )
        largest = right;

    if( largest != i )
    {
        heapSwap( heap, map, heap[i].getIdentifier(), heap[largest].getIdentifier() );
        heapWeightDown( heap, map, heap[largest].getIdentifier() );
    }
}

void BeaconUtilities::heapSwap( BeaconUtilities::BeaconHeap &heap, BeaconUtilities::BeaconMap &map, std::string beacon_one,
                               std::string beacon_two )
{
    uint64_t one_index = map[beacon_one];
    uint64_t two_index = map[beacon_two];

    ROS_INFO( "I1: %d I2: %d", (int)one_index, (int)two_index );
    std::iter_swap( heap.begin()+one_index, heap.begin()+two_index );

    map[beacon_one] = two_index;
    map[beacon_two] = one_index;
}

void BeaconUtilities::heapPop( BeaconUtilities::BeaconHeap &heap, BeaconUtilities::BeaconMap &map )
{
    if( !heap.empty() )
    {
        if( heap.size() == 1 )
        {
            heap.clear();
            map.clear();
        }
        else
        {
            std::string top_identifier = heap[0].getIdentifier();
            heapSwap( heap, map, heap[0].getIdentifier(), (*heap.end()).getIdentifier() );
            map.erase( top_identifier );
            heap.erase( heap.begin()+(heap.size()-1) );
            heapWeightDown( heap, map, heap[0].getIdentifier() );
        }
    }
}