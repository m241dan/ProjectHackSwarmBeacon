//
// Created by korisd on 11/23/18.
//

#ifndef BEHAVIOURS_ROVERUTILITIES_H
#define BEHAVIOURS_ROVERUTILITIES_H

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>
#include <math.h>
#include <vector>
#include <map>
#include "RoverBeacon.h"
#include <algorithm>

namespace RoverUtilities
{
    namespace Math
    {
        double  distance                            ( const geometry_msgs::Pose2D &from, const geometry_msgs::Pose2D &to );
        double  distance                            ( const geometry_msgs::Pose& pose );
        double  angularDistance                     ( const geometry_msgs::Pose2D &from, double to_theta );
        double  calculateDesiredVelocity            ( double distance, double deceleration_distance, double max_velocity, double min_velocity);
        double  getCorrectionBetweenRoverAndPoint   ( const geometry_msgs::Pose2D& rover, const geometry_msgs::Pose2D& point );
        bool    closerPose                          ( const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2 );
    };

};

#endif //BEHAVIOURS_ROVERUTILITIES_H
