#include "DropOffState.h"
#include "../Gripper.h"

void DropOffState::action()
{
    forceTransition( internalTransition() );
    internalAction();
}

void DropOffState::onEnter( std::string prev_state )
{
    if( prev_state == "findhome_state" )
    {
        forceTransition( DROPOFF_INIT );
    }
}

void DropOffState::onExit( std::string nexst_state )
{

}

std::string DropOffState::transition()
{
    std::string transition_to = getIdentifier();

    switch( internal_state )
    {
        default:break;
        case DROPOFF_COMPLETE:
            transition_to = "search_state";
    }

    return transition_to;
}

DOState DropOffState::internalTransition()
{
    DOState transition_to = internal_state;

    switch( internal_state )
    {
        default:break;
        case DROPOFF_INIT:
            if( this->approach )
                transition_to = DROPOFF_APPROACH;
            break;
        case DROPOFF_APPROACH:
            if( this->approach && this->approach->hasArrived() )
            {
                delete this->approach;
                this->approach = 0;
                this->outputs->current_waypoint = 0;

                transition_to = DROPOFF_ADJUST;
            }
            break;
    }

    return transition_to;
}

void DropOffState::internalAction()
{
    switch( internal_state )
    {
        default:break;
        case DROPOFF_INIT:
        {
            TagParams t_params;

            t_params.desired_tag = 256;

            t_params.dist_deccel = 0.05;
            t_params.dist_goal = 0.24;
            t_params.dist_max_output = 10;

            t_params.yaw_deccel = 0.10;
            t_params.yaw_goal = 0.0;
            t_params.yaw_max_output = (80/3);

            t_params.type = CLOSEST;

            this->approach = new ApproachTagWaypoint( this->inputs, t_params );
            this->outputs->current_waypoint = this->approach;
            break;
        }

        case DROPOFF_APPROACH:
            break;
        case DROPOFF_ADJUST:
            std::cout << "Made it to adjust" << std::endl;
            break;
    }
}

void DropOffState::forceTransition( DOState transition_to )
{
    DOState prev_state = internal_state;

    internal_state = transition_to;

    if( internal_state != prev_state )
    {
        /* on Exit */
        switch( prev_state )
        {
            default:break;
        }

        /* on Enter */
        switch( internal_state )
        {
            default: break;
            case DROPOFF_INIT:
                if( this->approach )
                {
                    delete this->approach;
                    this->approach = 0;
                }
                break;
        }
    }
}
