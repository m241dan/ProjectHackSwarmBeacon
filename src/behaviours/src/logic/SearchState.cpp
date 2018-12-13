#include <iostream>
#include "SearchState.h"
#include "../TagUtilities.h"
#include "../waypoints/SimpleWaypoint.h"

std::tuple<int,int> coord_key[4] = { std::make_tuple( -1, 1 ), std::make_tuple( 1, 1 ), std::make_tuple( 1, -1 ), std::make_tuple( -1, -1 ) };

void SearchState::setupPublishers()
{
    outputs->new_beacon_pub      = node_handle->advertise<swarmie_msgs::Beacon>( "new_beacon", 10, true );
    outputs->beacon_rover_pub    = node_handle->advertise<swarmie_msgs::BeaconUpdate>( "rover_interested_update", 10, true );
    outputs->beacon_cube_pub     = node_handle->advertise<swarmie_msgs::BeaconUpdate>( "cubes_seen_update", 10, true );
}

void SearchState::setupSubscribers()
{
    inputs->new_beacon_sub      = node_handle->subscribe( "new_beacon", 10, &SearchState::newBeaconHandler, this );
    inputs->beacon_rover_sub    = node_handle->subscribe( "rover_interested_update", 10, &SearchState::roverInterestedHandler, this );
    inputs->beacon_cube_sub     = node_handle->subscribe( "cubes_seen_update", 10, &SearchState::cubesSeenHandler, this );
    show_heap                   = node_handle->subscribe( (inputs->rover_name + "/showHeap"), 1, &SearchState::showHeap, this );
}

void SearchState::newBeaconHandler( const swarmie_msgs::Beacon::ConstPtr &beacon )
{
    if( inputs->beacon_map.find( beacon->identifier ) == inputs->beacon_map.end() )
    {
        RoverBeacon new_beacon( *beacon, dummy );
        BeaconUtilities::heapPush( inputs->beacon_heap, inputs->beacon_map, new_beacon );
    }
    else
    {
        ROS_INFO( "newBeaconHandler: attempting to add beacon with identifier %s but it already exists!", beacon->identifier.c_str() );
    }
}

void SearchState::roverInterestedHandler( const swarmie_msgs::BeaconUpdate::ConstPtr &message )
{
    if( inputs->beacon_map.find( message->identifier ) != inputs->beacon_map.end() )
    {
        if( message->value > 0 )
        {
            inputs->beacon_heap[inputs->beacon_map[message->identifier]].addRoverInterested();
            if( inputs->beacon_heap.size() > 1 )
                BeaconUtilities::heapWeightDown( inputs->beacon_heap, inputs->beacon_map, message->identifier );
        }
        else
        {
            inputs->beacon_heap[inputs->beacon_map[message->identifier]].remRoverInterested();
            if( inputs->beacon_heap.size() > 1 )
              BeaconUtilities::heapWeightUp( inputs->beacon_heap, inputs->beacon_map, message->identifier );

        }
    }
    else
    {
        ROS_INFO( "roverInterestedHandler: attempting to update a beacon with identifier %s that does not exist", message->identifier.c_str() );
    }
}

void SearchState::cubesSeenHandler( const swarmie_msgs::BeaconUpdate::ConstPtr &message )
{
    if( inputs->beacon_map.find( message->identifier ) != inputs->beacon_map.end() )
    {
        uint16_t initial_cubes = inputs->beacon_heap[inputs->beacon_map[message->identifier]].getCubes();
        inputs->beacon_heap[inputs->beacon_map[message->identifier]].setCubes( static_cast<uint16_t>(message->value) );

        if( message->value > initial_cubes )
            BeaconUtilities::heapWeightUp( inputs->beacon_heap, inputs->beacon_map, message->identifier );
        else if( message->value < initial_cubes )
            BeaconUtilities::heapWeightDown( inputs->beacon_heap, inputs->beacon_map, message->identifier );
    }
    else
    {
        ROS_INFO( "cubeSeenHandler: attempting to update a beacon with identifier %s that does not exist", message->identifier.c_str() );
    }
}

void SearchState::action()
{
    forceTransition( internalTransition() );
    internalAction();
}

void SearchState::onEnter( std::string prev_state )
{
    if(this->inputs->goalInObst){
        if( waypoints.size() > 0 )
        {
            delete waypoints.front();
            waypoints.erase( waypoints.begin() );
        }

        if( waypoints.size() != 0 )
        {
            SimpleWaypoint *wp = dynamic_cast<SimpleWaypoint*>( waypoints.front() );
            if( wp )
            {
                inputs->goal_x = wp->simple_params.goal_x;
                inputs->goal_y = wp->simple_params.goal_y;
            }
            else
            {
                inputs->goal_x = 0;
                inputs->goal_y = 0;
            }

            this->outputs->current_waypoint = waypoints.front();
        }
        else
            this->outputs->current_waypoint = 0;
        this->inputs->goalInObst = false;
    }
    if( waypoints.size() > 0 )
    {
       // outputs->current_waypoint = waypoints.front();
        if( !inputs->beacon_map.empty() && prev_state == "dropoff_state" )
        {
            geometry_msgs::Pose2D pose = waypoints.front()->getGoalPose();
            double waypoint_value = ( arena_size - hypot( pose.x, pose.y ) ) * meter_value;
            double beacon_value = inputs->beacon_heap.front().getWeight();

            ROS_INFO( "R: %s WV: %f BV %f", inputs->rover_name.c_str(), waypoint_value, beacon_value );
            if( waypoint_value < beacon_value )
            {
                SimpleWaypoint *wp;
                SimpleParams params;

                params.skid_steer_threshold = M_PI/6;
                params.linear_max = 40;
                params.rotational_max = 80;
                params.skid_max = 60;
                params.arrived_threshold = 0.05;
                params.goal_x = inputs->beacon_heap.front().getPosition().x;
                params.goal_y = inputs->beacon_heap.front().getPosition().y;
                wp = new SimpleWaypoint( inputs, params );
                waypoints.insert( waypoints.begin(), dynamic_cast<Waypoint*>( wp ) );
                inputs->present_beacon = inputs->beacon_heap.front();

                swarmie_msgs::BeaconUpdate update;
                update.identifier = inputs->present_beacon.getIdentifier();
                update.value = 1;
                outputs->beacon_rover_pub.publish( update );
            }
        }
        outputs->current_waypoint = waypoints.front();

    }
    else
        forceTransition( SEARCHSTATE_INIT );
}

void SearchState::onExit( std::string next_state )
{
    //insert new beacons here
    if( next_state == "pickup_state" && inputs->present_beacon.getIdentifier() != "dummy" )
    {
        SimpleWaypoint *wp;
        SimpleParams params;

        params.skid_steer_threshold = M_PI/6;

        params.linear_max = 40;
        params.rotational_max = 80;
        params.skid_max = 60;

        /* in sim */
        params.arrived_threshold = 0.05;
        params.goal_x = inputs->odom_accel.x == NAN ? 1 : inputs->odom_accel.x;
        params.goal_y = inputs->odom_accel.y == NAN ? 1 : inputs->odom_accel.y;

       /* in irl */
//       params.arrived_threshold = 0.25;
  //     params.goal_x = inputs->odom_accel_gps.x;
    //   params.goal_y = inputs->odom_accel_gps.y;

        wp = new SimpleWaypoint( inputs, params );
        waypoints.insert( waypoints.begin(), dynamic_cast<Waypoint*>( wp ) );

    }
}

std::string SearchState::transition()
{
    std::string transition_to = getIdentifier();

    if( TagUtilities::hasTag( &this->inputs->tags, 0 ) && !TagUtilities::hasTag( &this->inputs->tags, 256 ) )
    {
        transition_to = "pickup_state";
    }
    else if( TagUtilities::hasTag( &this->inputs->tags, 0 ) && TagUtilities::hasTag( &this->inputs->tags, 256 ) )
    {
        Tag closest_tag = TagUtilities::getClosestTag( &this->inputs->tags, 256 );
        Cube closest_cube = TagUtilities::getClosestCube( &this->inputs->cubes );

        if( closest_tag.getGroundDistance( 256 ) > closest_cube.getGroundDistance() )
            transition_to = "pickup_state";
    } else if( this->inputs->us_center < .35 || this->inputs->us_left < .35 ||  this->inputs->us_right < .35 ) {
        transition_to = "avoid_state";
    } else if ( TagUtilities::hasTag(&this->inputs->tags, 256)){
        transition_to = "avoid_state";
    }

    return transition_to;
}

SSState SearchState::internalTransition()
{
    SSState transition_to = internal_state;

    switch( internal_state )
    {
        default: break;
        case SEARCHSTATE_INIT:
            if( waypoints.size() != 0 )
                transition_to = SEARCHSTATE_DRIVE;
            break;
        case SEARCHSTATE_DRIVE:
            if( waypoints.size() == 0 )
                transition_to = SEARCHSTATE_RESET;
            break;
    }

    return transition_to;
}

void SearchState::internalAction()
{
    switch( internal_state )
    {
        default: break;
        case SEARCHSTATE_INIT:
        {
            SimpleWaypoint *waypoint = 0;
            SimpleParams params;

            /* in irl */
//            params.skid_steer_threshold = M_PI/6;
//            params.arrived_threshold = 0.15;
            /* in sim */
            params.skid_steer_threshold = M_PI/6;
            params.arrived_threshold = 0.05;

            int num_rovers = inputs->infoVector.size();
            /* get your "spot" */
            int spot = 0;
            for( int i = 0; i < num_rovers; i++ )
            {
                if( inputs->rover_name == inputs->infoVector[i].name )
                {
                    spot = i + 1;
                    break;
                }
            }


            /* figure out what you're closest to */
            int closest_starting_point = 0;
            double closest_distance = 1000; //a dumby number, you'll never be that far away
            for( int i = 0; i < 4; i++ )
            {
                double x = std::get<0>( coord_key[i] );
                double y = std::get<1>( coord_key[i] );
                /* in sim */
                double curr_distance = hypot( x - inputs->odom_accel.x, y - inputs->odom_accel.y );
                /* in irl */
//                double curr_distance = hypot( x - inputs->odom_accel_gps.x, y - inputs->odom_accel_gps.y );

                if( curr_distance <= closest_distance )
                {
                     closest_starting_point = i;
                     closest_distance = curr_distance;
                }
            }
            /* generate the waypoints, 5 at a time */
            int ring_multiplier = 0;
            if( num_rovers <= 3 )
            {
                ring_multiplier = 1;
            }
            else
            {
                ring_multiplier = 1.5;
            }
            for( int ring = 0 + spot; ring < ( MAX_RINGS + 1 ); ring++ )
            {
                int count = 0;
                int i = closest_starting_point;
                while( count < 5 )
                {
                     params.goal_x = std::get<0>( coord_key[i] ) * ring * ring_multiplier;
                     params.goal_y = std::get<1>( coord_key[i] ) * ring * ring_multiplier;

                     waypoint = new SimpleWaypoint( inputs, params );
                     waypoints.push_back( dynamic_cast<Waypoint *>( waypoint ) );

                     count++;
                     if( ++i == 4 )
                        i = 0;
                }
            }
            this->outputs->current_waypoint = waypoints.front();

            break;
        }
        case SEARCHSTATE_DRIVE:
        {
            Waypoint *waypoint = this->waypoints.front();
            if( waypoint )
            {
                if( waypoint->hasArrived() )
                {
                    delete waypoint;
                    waypoints.erase( waypoints.begin() );
                    if( waypoints.size() != 0 )
                        this->outputs->current_waypoint = waypoints.front();
                    else
                        this->outputs->current_waypoint = 0;
                }
            }
            outputs->gripper_position = Gripper::HOVER_OPEN;
            break;
        }
    }
}

void SearchState::forceTransition( SSState transition_to )
{
    SSState prev_state = internal_state;

    internal_state = transition_to;

    if( internal_state != prev_state )
    {
        /* onExit bits */
        switch( prev_state )
        {
            default: break;
        }

        /* onEnter bits */
        switch( internal_state )
        {
            default: break;
        }

    }
}

void SearchState::showHeap( const std_msgs::UInt8::ConstPtr &msg )
{
    for( auto &beacon : inputs->beacon_heap )
    {
        ROS_INFO( "ID: %s Weight: %d", beacon.getIdentifier().c_str(), beacon.getWeight() );
    }
}