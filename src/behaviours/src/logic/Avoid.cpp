#include "Avoid.h"

void Avoid::action( void )
{
    forceTransition( internalTransition() );
    internalAction();
}

void Avoid::onEnter( std::string prev_state )
{
    attempts = 0;
    previous_state = prev_state;
    forceTransition( AVOID_INIT );

    if( entrance_goal )
    {
        delete entrance_goal;
        entrance_goal = 0;
    }

    SimpleParams params;

    params.skid_steer_threshold = M_PI/6;
    params.arrived_threshold = 0.25;
    params.goal_x = inputs->goal_x;
    params.goal_y = inputs->goal_y;

    entrance_goal = new SimpleWaypoint( inputs, params );
}

void Avoid::onExit( std::string next_state )
{

}

std::string Avoid::transition()
{
    std::string transition_to = getIdentifier();

    if( TagUtilities::hasTag( &inputs->tags, 0 ) && previous_state == "search_state" )
    {
        if( !TagUtilities::hasTag( &this->inputs->tags, 256 ) )
        {
            transition_to = "pickup_state";
        }
        else
        {
            Tag closest_tag = TagUtilities::getClosestTag( &this->inputs->tags, 256 );
            Cube closest_cube = TagUtilities::getClosestCube( &this->inputs->cubes );

            if( closest_tag.getGroundDistance( 256 ) > closest_cube.getGroundDistance() )
                transition_to = "pickup_state";
        }

    }
    else if( TagUtilities::hasTag( &inputs->tags, 256 ) && previous_state == "findhome_state" )
    {
        transition_to = "dropoff_state";
    }
    else if( internal_state == AVOID_EXIT_SUCCESS )
        transition_to = previous_state;
    else if( internal_state == AVOID_EXIT_FAILURE )
    {
        transition_to = previous_state;
        inputs->goalInObst = true;
    }

    return transition_to;
}

ASState Avoid::internalTransition()
{
    ASState transition_to = internal_state;

    switch( internal_state )
    {
        case AVOID_INIT:
            if( rotate )
                transition_to = AVOID_ROTATE;
            break;
        case AVOID_ROTATE:
            if( inputs->us_left > .75 && inputs->us_center > .75 && inputs->us_right > .75 && !TagUtilities::hasTag( &inputs->tags, 256 ) && !TagUtilities::hasTagInRange( &inputs->tags, 0, .21, .4 ) )
            {
                transition_to = AVOID_DRIVE;
            }
            break;
        case AVOID_DRIVE:
            if( inputs->us_left < 0.35 || inputs->us_right < 0.35 || inputs->us_center < .035 || TagUtilities::hasTag( &inputs->tags, 256 ) || TagUtilities::hasTagInRange( &inputs->tags, 0, .21, .4 ) )
            {
                outputs->current_waypoint = 0;
                delete drive;
                drive = 0;

                transition_to = AVOID_ROTATE;
            }
            else if( drive && drive->hasArrived() )
            {
                outputs->current_waypoint = 0;
                delete drive;
                drive = 0;

                transition_to = AVOID_ATTEMPT_EXIT;
            }
            break;
        case AVOID_ATTEMPT_EXIT:
            if( inputs->us_left < 0.35 || inputs->us_right < 0.35 || inputs->us_center < .035 || TagUtilities::hasTag( &inputs->tags, 256 ) || TagUtilities::hasTagInRange( &inputs->tags, 0, .21, .4 ) )
            {
                if( ++attempts > MAX_EXIT_ATTEMPTS )
                    transition_to = AVOID_EXIT_FAILURE;
                else
                    transition_to = AVOID_ROTATE;
            }
            else if( entrance_goal && entrance_goal->hasArrived() || ( inputs->time.toSec() - timer > RESUME_ATTEMPT_TIME ) )
            {
                transition_to = AVOID_EXIT_SUCCESS;
            }
            break;
        case AVOID_EXIT_FAILURE:
            break;
        case AVOID_EXIT_SUCCESS:
            break;
    }

    return transition_to;
}

void Avoid::internalAction()
{
    switch( internal_state )
    {
        case AVOID_INIT:
        {
            RawOutputParams r_params;

            r_params.left_output = -AVOID_ROTATION_SPEED;
            r_params.right_output = AVOID_ROTATION_SPEED;

            rotate = new RawOutputWaypoint( inputs, r_params );
            outputs->current_waypoint = rotate;
            break;
        }
        case AVOID_ROTATE:
            break;
        case AVOID_DRIVE:
            break;
        case AVOID_ATTEMPT_EXIT:
            break;
        case AVOID_EXIT_FAILURE:
            break;
        case AVOID_EXIT_SUCCESS:
            break;
    }
}

void Avoid::forceTransition( ASState transition_to )
{
    ASState prev_state = internal_state;

    internal_state = transition_to;

    if( internal_state != prev_state )
    {
        /* onExit */
        switch( prev_state )
        {
            default:break;
        }

        /* onEnter */
        switch( internal_state )
        {
            default:break;
            case AVOID_ROTATE:
                outputs->current_waypoint = rotate;
                break;
            case AVOID_DRIVE:
            {
                LinearParams l_params;

                l_params.distance = 0.75;
                l_params.max_output = 60;

                drive = new LinearWaypoint( inputs, l_params );
                outputs->current_waypoint = drive;
                break;
            }
            case AVOID_ATTEMPT_EXIT:
                outputs->current_waypoint = entrance_goal;
                timer = inputs->time.toSec();
                break;
        }
    }
}


