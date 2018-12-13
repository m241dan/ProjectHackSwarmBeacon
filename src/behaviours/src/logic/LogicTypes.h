#ifndef logictypes_h
#define logictypes_h

#include <vector>
#include <geometry_msgs/Pose2D.h>
#include "../Tag.h"
#include "../TagUtilities.h"
#include "../Gripper.h"
#include "../TagExaminer.h"
#include "../Cube.h"
#include <ros/ros.h>
#include "../RoverBeacon.h"
#include "../RoverUtilities.h"
#include <swarmie_msgs/Beacon.h>
#include <swarmie_msgs/BeaconUpdate.h>
#include <vector>
#include <map>

class Waypoint;

typedef struct calibration_nums
{
    int motor_min = 12;
    int rotational_min = 15;
} CalibNums;

struct roverInfo {
    int id;
    string name;
    float x;
    float y;
    float sonar_left;
    float sonar_right;
    float sonar_center;
    string state;
    int number_of_cubes;
    int number_of_base_tags;

    bool operator < ( const roverInfo& sec ) const
    {
        return ( name < sec.name );
    }
};

typedef struct logic_inputs
{
    geometry_msgs::Pose2D	raw_odom;
    geometry_msgs::Pose2D	odom_accel;
    geometry_msgs::Pose2D	odom_accel_gps;
    std::vector<Tag> 		tags;
    std::vector<Cube>           cubes;
    double          goal_x;
    double          goal_y;
    bool            goalInObst = false;
    bool            rotationFlag = false;
    int             avoidCounter = 0;
    double          initialAvoidAngle;
    double          initialAvoidX;
    double          initialAvoidY;
    std::string          prevState;
    double			us_left = 0.0;
    double			us_right = 0.0;
    double			us_center = 0.0;
    double			linear_vel_odom_accel = 0.0;
    double			linear_vel_odom_accel_gps = 0.0;
    double			angular_vel_odom_accel = 0.0;
    double			angular_vel_odom_accel_gps = 0.0;
    ros::Time			time;
    CalibNums			calibration;
    TagExaminer                 examiner;
    std::vector<roverInfo>	infoVector;
    std::string			rover_name;
    ros::Subscriber new_beacon_sub;
    ros::Subscriber beacon_cube_sub;
    ros::Subscriber beacon_rover_sub;
    BeaconUtilities::BeaconMap beacon_map;
    BeaconUtilities::BeaconHeap beacon_heap;
    int beacon_counter;
    RoverBeacon present_beacon;
    logic_inputs() : present_beacon( "dummy", raw_odom, raw_odom )
    {
    }
} LogicInputs;

typedef struct logic_outputs
{
    Gripper::Position gripper_position = Gripper::HOVER_OPEN;
    Waypoint *current_waypoint = 0;
    double offset_x = 0.;
    double offset_y = 0.;
    ros::Publisher new_beacon_pub;
    ros::Publisher beacon_cube_pub;
    ros::Publisher beacon_rover_pub;

} LogicOutputs;

typedef struct io_table
{
    LogicInputs *inputs;
    LogicOutputs *outputs;
} IOTable;

bool shouldAvoidCube( LogicInputs *inputs );

#endif
