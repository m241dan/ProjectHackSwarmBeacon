//
// Created by korisd on 11/23/18.
//

#include "RoverUtilities.h"

double RoverUtilities::Math::distance( const geometry_msgs::Pose2D &from, const geometry_msgs::Pose2D &to )
{
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return sqrt( (dx*dx) + (dy*dy) );
}

double RoverUtilities::Math::distance( const geometry_msgs::Pose &pose )
{
    return hypot( hypot( pose.position.x, pose.position.y ), pose.position.z );
}

double RoverUtilities::Math::angularDistance( const geometry_msgs::Pose2D &from, double to_theta )
{
    return angles::shortest_angular_distance( from.theta, to_theta );
}

double RoverUtilities::Math::calculateDesiredVelocity( double distance, double deceleration_distance,
                                                              double max_velocity, double minimum_velocity )
{
    if( deceleration_distance > 0 )
    {
        double velocity = distance * (fabs( max_velocity ) / deceleration_distance);
        return std::min( fabs( max_velocity ), std::max( velocity, minimum_velocity ));
    }
    return max_velocity;
}

double RoverUtilities::Math::getCorrectionBetweenRoverAndPoint( const geometry_msgs::Pose2D &rover,
                                                                const geometry_msgs::Pose2D &goal_point )
{
    //make the rover the origin, and generate the "new" coordinate position of the goal point based on that.
    geometry_msgs::Pose2D new_goal_point;
    new_goal_point.x = goal_point.x - rover.x;
    new_goal_point.y = goal_point.y - rover.y;

    //arctan the point get the angle of it from "origin"
    double angle_to_point = atan2( new_goal_point.y, new_goal_point.x );

    //shortest angular distance between the Rover's theta (ie, which way it's facing) and the correction angle to the point
    return angles::shortest_angular_distance( rover.theta, angle_to_point );
}

bool RoverUtilities::Math::closerPose( const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2 )
{
    return distance( p1 ) < distance( p2 );
}