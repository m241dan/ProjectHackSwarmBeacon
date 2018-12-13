#ifndef pickupstate_h
#define pickupstate_h

#include <angles/angles.h>
#include "../state_machine/State.h"
#include "../waypoints/ApproachCube.h"
#include "../waypoints/LinearWaypoint.h"
#include "../waypoints/RotationalWaypoint.h"
#include "LogicTypes.h"

#define MAX_ATTEMPTS 20
#define MAX_TRIES 4
#define CLOSE_TIME 1.25
#define UP_TIME 1.25
#define CONFIRM_TIME 2.00
#define MAX_DISTANCE_CHANGE 0.02
#define APPROACH_TIME 15

typedef enum
{
    PICKUP_INIT,
    PICKUP_COMPLETE_FAILURE,
    PICKUP_APPROACH,
    PICKUP_FINAL_APPROACH,
    PICKUP_CLAW_CLOSE,
    PICKUP_CLAW_UP,
    PICKUP_CONFIRM,
    PICKUP_BACKUP,
    PICKUP_ROTATE,
    PICKUP_COMPLETE,
    PICKUP_FAIL
} PUState;

class PickUpState : public State
{
    public:
        PickUpState( IOTable *io ) : State( "pickup_state" ), internal_state(PICKUP_INIT), inputs(io->inputs), outputs(io->outputs) {}
        virtual void action( void );
        virtual void onEnter( std::string prev_state );
        virtual void onExit( std::string next_state );
        virtual std::string transition();

    private:
        PUState internalTransition();
        void internalAction();
        void forceTransition( PUState transition_to );

        ApproachCube *approach;
        LinearWaypoint *linear;
        LinearWaypoint *backup;
        RotationalWaypoint *rotate;

        double timer;

        LogicInputs *inputs;
        LogicOutputs *outputs;
        PUState internal_state;
        double prev_distance;
        int attempts;
        int num_tries;
        bool cube_secured;

        int cubes_seen;
        geometry_msgs::Pose2D confirm_position;
        bool home_seen;
};

#endif
