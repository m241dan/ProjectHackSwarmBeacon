#include "PickUpState.h"
#include "../Gripper.h"

/********************
 * Basic Operations *
 ********************/

void PickUpState::action()
{
    forceTransition( internalTransition() );
    internalAction();
}

void PickUpState::onEnter()
{
    forceTransition( PICKUP_INIT );
    num_tries = 0;
    attempts = 0;
    cube_secured = false;
}

void PickUpState::onExit()
{

}

std::string PickUpState::transition()
{
    std::string transition_to = getIdentifier();
    return transition_to;
}

/***********************
 * Internal Operations *
 ***********************/
PUState PickUpState::internalTransition()
{
    PUState transition_to = internal_state;

    switch( internal_state )
    {
        case PICKUP_INIT:
            if( approach )
            {
                transition_to = PICKUP_APPROACH;
            }
            else
            {
                if( this->attempts > MAX_ATTEMPTS )
                    transition_to = PICKUP_FAIL;

            }
            break;
        case PICKUP_FAILED_INIT:
            //complete do nothing state
            break;
        case PICKUP_APPROACH:
            if( approach && approach->hasArrived() )
            {
                outputs->current_waypoint = 0;
                delete this->approach;
                this->approach = 0;

                transition_to = PICKUP_FINAL_APPROACH;
                /* on Enter */
                LinearParams l_params;

                l_params.distance = 0.11;
                l_params.deccel_point = 0;
                l_params.max_vel = 5;

                this->linear = new LinearWaypoint( this->inputs, l_params );
                this->outputs->current_waypoint = this->linear;
            }
            else if( attempts > MAX_ATTEMPTS )
                transition_to = PICKUP_FAIL;
            break;
        case PICKUP_FINAL_APPROACH:
            if( linear && linear->hasArrived() )
            {
                outputs->current_waypoint = 0;
                delete this->linear;
                this->linear = 0;

                transition_to = PICKUP_CLAW_CLOSE;
                this->timer = this->inputs->time.toSec();
            }
            else if( this->attempts > MAX_ATTEMPTS )
                transition_to = PICKUP_FAIL;
            break;
        case PICKUP_CLAW_CLOSE:
            if( ( this->inputs->time.toSec() - this->timer ) >= CLOSE_TIME )
            {
                transition_to = PICKUP_CLAW_UP;
                this->timer = this->inputs->time.toSec();
            }
            break;
        case PICKUP_CLAW_UP:
            if( ( this->inputs->time.toSec() - this->timer ) >= UP_TIME )
            {
                transition_to = PICKUP_CONFIRM;
                this->timer = this->inputs->time.toSec();
            }
            break;
        case PICKUP_CONFIRM:
            if( ( this->inputs->time.toSec() - this->timer ) >= CONFIRM_TIME )
            {
                if( cube_secured )
                    transition_to = PICKUP_COMPLETE;
                else
                {
                    transition_to = PICKUP_FAIL;
                    /* on Enter */
                    this->num_tries++;

                    LinearParams l_params;

                    l_params.distance = .1;
                    l_params.deccel_point = 0.05;
                    l_params.max_vel = 10;
                    l_params.reverse = true;

                    this->linear = new LinearWaypoint( this->inputs, l_params );
                    this->outputs->current_waypoint = this->linear;
                }
            }
        case PICKUP_HOVER_CLOSE:
        case PICKUP_COMPLETE:
        case PICKUP_FAIL:
            if( linear && linear->hasArrived() )
            {
                transition_to = PICKUP_INIT;
            }
            break;
    }
    return transition_to;
}

void PickUpState::internalAction()
{
    switch( internal_state )
    {
        case PICKUP_INIT:
            if( TagUtilities::hasTag( &inputs->tags, 0 ) )
            {
                TagParams t_params;

                t_params.desired_tag = 0;

                t_params.dist_deccel = 0.05;
                t_params.dist_goal = 0.24;
                t_params.dist_max_output = 10;

                t_params.yaw_deccel = 0.15;
                t_params.yaw_goal = 0.0;
                t_params.yaw_max_output = 80;

                t_params.type = CLOSEST;

                approach = new ApproachTagWaypoint( inputs, t_params );
                outputs->current_waypoint = approach;
            }
            else
            {
                this->attempts++;
            }
            break;
        case PICKUP_FAILED_INIT:
            //complete do nothing state
            break;
        case PICKUP_APPROACH:
            outputs->gripper_position = Gripper::DOWN_OPEN;
            if( !TagUtilities::hasTag( &inputs->tags, 0 ) )
                this->attempts++;
            else
                this->attempts = 0;
            break;
        case PICKUP_FINAL_APPROACH:
            if( !TagUtilities::hasTag( &inputs->tags, 0 ) )
                this->attempts++;
            else
                this->attempts = 0;
            outputs->gripper_position = Gripper::DOWN_OPEN;
            break;
        case PICKUP_CLAW_CLOSE:
            outputs->gripper_position = Gripper::DOWN_CLOSED;
            break;
        case PICKUP_CLAW_UP:
            outputs->gripper_position = Gripper::UP_CLOSED;
            break;
        case PICKUP_CONFIRM:
            if( this->inputs->us_center < 0.13 )
                cube_secured = true;
            else if( TagUtilities::hasTag( &this->inputs->tags, 0 ) && TagUtilities::getDistance( this->inputs->tags.back() ) < 0.15 )
                cube_secured = true;
            break;
        case PICKUP_HOVER_CLOSE:
            outputs->gripper_position = Gripper::HOVER_CLOSED;
            break;
        case PICKUP_COMPLETE:
            break;
        case PICKUP_FAIL:
            outputs->gripper_position = Gripper::DOWN_OPEN;
            break;

    }
}

void PickUpState::forceTransition( PUState transition_to )
{
    internal_state = transition_to;
}
