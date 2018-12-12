#ifndef simplewaypoint_h
#define simplewaypoint_h

#include "Waypoint.h"
#include "../RadRotPID.h"
#include "../LinearPID.h"

//SimpleWaypointStates seperate .cpp/.h

typedef enum
{
    SIMPLE_INIT,
    SIMPLE_ROTATE,
    SIMPLE_SKID,
    SIMPLE_ARRIVED
} SWState;

typedef struct simple_params
{
    double skid_steer_threshold = M_PI/6;
    double arrived_threshold = 0.05;

    double goal_x = 0;
    double goal_y = 0;

    int linear_max = 40;
    int rotational_max = 80;
    int skid_max = 60;

} SimpleParams;


class SimpleWaypoint : public Waypoint
{
    public:
        SimpleWaypoint( LogicInputs *i, SimpleParams sp ) : Waypoint( i ), simple_params(sp), internal_state(SIMPLE_INIT)
        {
            driving_params.goal_x = simple_params.goal_x;
            driving_params.goal_y = simple_params.goal_y;
            /* in sim */
            driving_params.current_x = &inputs->odom_accel.x; // _gps.x;
            driving_params.current_y = &inputs->odom_accel.y; // _gps.y;
            driving_params.current_theta = &inputs->odom_accel.theta; // _gps.theta;
            /* in irl */
//            driving_params.current_x = &inputs->odom_accel_gps.x;
  //          driving_params.current_y = &inputs->odom_accel_gps.y;
    //        driving_params.current_theta = &inputs->odom_accel_gps.theta;

            linear_pid = LinearPID( WaypointUtilities::getDistSkidBasedPIDParams() );
            rotational_pid = RadRotPID( WaypointUtilities::getRadianRotSkidBasedPIDParams() );

        }
        virtual void run();
        virtual geometry_msgs::Pose2D getGoalPose();
        SimpleParams simple_params;
    private:
        SWState internalTransition();
        void internalAction();
        void forceTransition( SWState transition_to );

        WaypointUtilities::DrivingParams driving_params;
        SWState internal_state;

        LinearPID linear_pid;
        RadRotPID rotational_pid;
 };

#endif
