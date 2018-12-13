#ifndef searchstate_h
#define searchstate_h

#include "../state_machine/State.h"
#include "LogicMachine.h"
#include "../RoverBeacon.h"
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>


#define SEARCH_SIZE 20.0
#define MAX_RINGS 7
typedef enum
{
    SEARCHSTATE_INIT,
    SEARCHSTATE_DRIVE,
    SEARCHSTATE_RESET
} SSState;

class SearchState : public State
{
    public:
        SearchState( IOTable *io ) : State( "search_state" ), inputs(io->inputs), outputs(io->outputs), internal_state( SEARCHSTATE_INIT )
        {
            inputs->beacon_heap.reserve( sizeof( RoverBeacon ) * 30 );
            inputs->beacon_counter = 0;
            RoverBeacon beacon( "dummy", dummy, dummy );
        }
        virtual void action( void );
        virtual void onEnter( std::string prev_state );
        virtual void onExit( std::string next_state );
        virtual std::string transition();
        ros::NodeHandle* node_handle;
        void setupPublishers();
        void setupSubscribers();

    private:
        SSState internalTransition();
        void internalAction();
        void forceTransition( SSState transition_to );

        void newBeaconHandler           ( const swarmie_msgs::Beacon::ConstPtr& beacon );
        void roverInterestedHandler     ( const swarmie_msgs::BeaconUpdate::ConstPtr& message );
        void cubesSeenHandler           ( const swarmie_msgs::BeaconUpdate::ConstPtr& message );
        void showHeap                   ( const std_msgs::UInt8::ConstPtr& msg );
        geometry_msgs::Pose2D dummy;

        std::vector<Waypoint *> waypoints;
        LogicInputs *inputs;
        LogicOutputs *outputs;
        SSState internal_state;

        ros::Subscriber show_heap;
};

#endif
