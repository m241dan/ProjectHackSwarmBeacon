//
// Created by korisd on 11/21/18.
//

#include "RoverMap.h"

RoverMap::RoverMap( ros::NodeHandle &nh, std::string name_of_rover ) : node_handle( nh ),
                                                                       rover_name( std::move( name_of_rover )),
                                                                       beacon_identifier( 0 ),
                                                                       present_beacon("")
{
    //Publishers
    new_beacon_pub          = node_handle.advertise<swarmie_msgs::Beacon>( "new_beacon", 10, true );
    rover_interested_pub    = node_handle.advertise<swarmie_msgs::BeaconUpdate>( "rover_interested_update", 10, true );
    cubes_seen_pub          = node_handle.advertise<swarmie_msgs::BeaconUpdate>( "cubes_seen_update", 10, true );

    //Subscribers
    raw_odom                = node_handle.subscribe( ( rover_name + "/odom" ), 10, &RoverMap::odomHandler, this );
   // ekf = node_handle.subscribe( ( rover_name + "/odom/ekf" ), 10, &RoverMap::ekfHandler, this );
    //For Simulator use only. Needs to be changed in Rev 2, but need to post of forum about the EKF being completely fucked in sim
    ekf                     = node_handle.subscribe( ( rover_name + "/odom/filtered" ), 10, &RoverMap::ekfHandler, this );
    new_beacon_sub          = node_handle.subscribe( "new_beacon", 10, &RoverMap::newBeaconHandler, this );
    rover_interested_sub    = node_handle.subscribe( "rover_interested_update", 10, &RoverMap::roverInterestedHandler, this );
    cubes_seen_sub          = node_handle.subscribe( "cubes_seen_update", 10, &RoverMap::cubesSeenHandler, this );

    //Services
    get_position            = node_handle.advertiseService( (rover_name + "/getPosition"), &RoverMap::getPosition, this );
    get_odom                = node_handle.advertiseService( (rover_name + "/getOdom"), &RoverMap::getOdom, this );
    set_offset              = node_handle.advertiseService( (rover_name + "/setOffset"), &RoverMap::setOffset, this );
    new_beacon_service      = node_handle.advertiseService( (rover_name + "/newBeacon"), &RoverMap::newBeacon, this );
    beacon_update_service   = node_handle.advertiseService( (rover_name + "/beaconUpdate"), &RoverMap::beaconUpdate, this );
    get_top_beacon          = node_handle.advertiseService( (rover_name + "/getTopBeacon"), &RoverMap::getTopBeacon, this );
    rover_uninterest_beacon = node_handle.advertiseService( (rover_name + "/roverUninterestBeacon"), &RoverMap::roverUninterestBeacon, this );
    clear_temporary_beacons = node_handle.advertiseService( (rover_name + "/clearTemporaryBeacons"), &RoverMap::clearTempBeacons, this );
}

RoverMap::~RoverMap()
{

}

void RoverMap::odomHandler( const nav_msgs::Odometry::ConstPtr &msg )
{
    present_odom.x = msg->pose.pose.position.x;
    present_odom.y = msg->pose.pose.position.y;
}

void RoverMap::ekfHandler( const nav_msgs::Odometry::ConstPtr &msg )
{
    tf::Quaternion q( msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w );

    tf::Matrix3x3 m( q );

    double roll, pitch, yaw;
    m.getRPY( roll, pitch, yaw );

    present_position.x = msg->pose.pose.position.x;
    present_position.y = msg->pose.pose.position.y;
    present_position.theta = yaw;
}

bool RoverMap::getPosition( swarmie_msgs::GetPositionRequest& req, swarmie_msgs::GetPositionResponse& res )
{
    res.position.x = present_position.x - present_offset.x;
    res.position.y = present_position.y - present_offset.y;
    res.position.theta = present_position.theta;
    return true;
}

bool RoverMap::getOdom( swarmie_msgs::GetOdomRequest& req, swarmie_msgs::GetOdomResponse& res )
{
    res.odom.x = present_odom.x;
    res.odom.y = present_odom.y;
    return  true;
}

bool RoverMap::setOffset( swarmie_msgs::SetOffsetRequest &req, swarmie_msgs::SetOffsetResponse &res )
{
    present_offset.x = present_position.x + req.x;
    present_offset.y = present_position.y + req.y;
    return true;
}

void RoverMap::newBeaconHandler( const swarmie_msgs::Beacon::ConstPtr &beacon )
{
    if( beacon_map.find( beacon->identifier ) == beacon_map.end() )
    {
        beacon_heap.emplace_back( *beacon, present_position );
        std::push_heap( beacon_heap.begin(), beacon_heap.end() );

        uint32_t index = UINT32_MAX;
        for( uint32_t x = 0; x < beacon_heap.size(); x++ )
        {
            if( beacon_heap[x].getIdentifier() == beacon->identifier )
                index = x;
        }

        if( index != UINT32_MAX )
            beacon_map.insert( std::pair<std::string,uint32_t>( beacon->identifier, index ) );
    }
    else
    {
        ROS_INFO( "newBeaconHandler: attempting to add beacon with identifier %s but it already exists!", beacon->identifier.c_str() );
    }
}

void RoverMap::roverInterestedHandler( const swarmie_msgs::BeaconUpdate::ConstPtr &message )
{
    if( beacon_map.find( message->identifier ) != beacon_map.end() )
    {
        if( message->value > 0 )
            beacon_heap[beacon_map[message->identifier]].addRoverInterested();
        else
            beacon_heap[beacon_map[message->identifier]].remRoverInterested();

        std::make_heap( beacon_heap.begin(), beacon_heap.end() );

        for( uint32_t x = 0; x < beacon_heap.size(); x++ )
            beacon_map[beacon_heap[x].getIdentifier()] = x;
    }
    else
    {
        ROS_INFO( "roverInterestedHandler: attempting to update a beacon with identifier %s that does not exist", message->identifier.c_str() );
    }
}

void RoverMap::cubesSeenHandler( const swarmie_msgs::BeaconUpdate::ConstPtr &message )
{
    if( beacon_map.find( message->identifier ) != beacon_map.end() )
    {
        beacon_heap[beacon_map[message->identifier]].setCubes( static_cast<uint16_t>(message->value) );
        std::make_heap( beacon_heap.begin(), beacon_heap.end() );

        for( uint32_t x = 0; x < beacon_heap.size(); x++ )
            beacon_map[beacon_heap[x].getIdentifier()] = x;
    }
    else
    {
        ROS_INFO( "cubeSeenHandler: attempting to update a beacon with identifier %s that does not exist", message->identifier.c_str() );
    }
}

bool RoverMap::newBeacon( swarmie_msgs::NewBeaconRequest &req, swarmie_msgs::NewBeaconResponse &res )
{
    req.beacon.identifier = rover_name + std::to_string( beacon_identifier++ );
    if( req.beacon.num_of_cubes == 0 && req.beacon.temporary == 0 )
    {
        beacon_heap.emplace_back( req.beacon, present_position );
        std::push_heap( beacon_heap.begin(), beacon_heap.end() );

        uint32_t index = UINT32_MAX;
        for( uint32_t x = 0; x < beacon_heap.size(); x++ )
        {
            if( beacon_heap[x].getIdentifier() == req.beacon.identifier )
                index = x;
        }

        if( index != UINT32_MAX )
            beacon_map.insert( std::pair<std::string,uint32_t>( req.beacon.identifier, index ) );
    }
    else
        new_beacon_pub.publish( req.beacon );
    return true;
}

bool RoverMap::beaconUpdate( swarmie_msgs::UpdateBeaconRequest &req, swarmie_msgs::UpdateBeaconResponse &res )
{
    swarmie_msgs::BeaconUpdate update_message;
    update_message.identifier = req.identifier;
    if( update_message.identifier.empty() )
        update_message.identifier = present_beacon;
    update_message.value = req.num_of_cubes;
    cubes_seen_pub.publish( update_message );
    return true;
}

bool RoverMap::getTopBeacon( swarmie_msgs::GetTopBeaconRequest &req, swarmie_msgs::GetTopBeaconResponse &res )
{
    RoverBeacon& b = beacon_heap.front();
    res.beacon = b.toMessage();

    if( res.beacon.num_of_cubes > 0 )
    {
        //if num cubes is greater than zero than its global, so publish
        swarmie_msgs::BeaconUpdate update_message;
        update_message.identifier = b.getIdentifier();
        update_message.value = 1;

        rover_interested_pub.publish( update_message );
    }
    present_beacon = res.beacon.identifier;
    return true;
}

bool RoverMap::roverUninterestBeacon( swarmie_msgs::UninterestBeaconRequest &req,
                                      swarmie_msgs::UninterestBeaconResponse &res )
{
    if( req.beacon.num_of_cubes > 0 )
    {
        swarmie_msgs::BeaconUpdate update_message;
        update_message.identifier = req.beacon.identifier;
        if( update_message.identifier.empty() )
            update_message.identifier = present_beacon;
        update_message.value = -1;
        rover_interested_pub.publish( update_message );
    }
    else
    {
        for( auto it = beacon_heap.begin(); it != beacon_heap.end(); it++ )
        {
            if( (*it).getIdentifier() == present_beacon )
                beacon_heap.erase( it );
        }
        std::make_heap( beacon_heap.begin(), beacon_heap.end() );

    }
    present_beacon = "";
}

bool RoverMap::clearTempBeacons( std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res )
{

    for( auto it = beacon_heap.begin(); it != beacon_heap.end(); ++it )
    {
        if( (*it).isTemporary() )
            beacon_heap.erase( it );
    }

    std::make_heap( beacon_heap.begin(), beacon_heap.end() );

    beacon_map.clear();
    for( uint32_t x = 0; x < beacon_heap.size(); x++ )
        beacon_map.insert( std::pair<std::string,uint32_t>(beacon_heap[x].getIdentifier(), x) );

    return true;
}