#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "swarmie_msgs/Waypoint.h"
#include "swarmie_msgs/InfoMessage.h"

// Include Controllers
#include <vector>

#include "Point.h"
#include "Tag.h"

/****************
 * New Includes *
 ****************/
#include "state_machine/StateMachine.h"
#include "waypoints/SimpleWaypoint.h"
#include "logic/LogicMachine.h"
#include "logic/SearchState.h"
#include "logic/PickUpState.h"
#include "logic/FindHomeState.h"
//#include "logic/AvoidState.h"
#include "logic/AvoidHomeState.h"
#include "logic/AvoidCubeState.h"
#include "logic/DropOffState.h"
#include "logic/InitState.h"
#include "Gripper.h"
#include "MotorController.h"
#include "TagUtilities.h"
#include "logic/Avoid.h"
// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;


/************************
 * Global Alphabet Soup *
 ************************/

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create logic controller

void humanTime();

// Behaviours Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void sendGripperPosition( Gripper::Position pos );

int currentMode = 0;
const float state_machines_loop = 0.1; // time between state machines function call
const float status_publish_interval = 1;
const float heartbeat_publish_interval = 2;
const float info_publish_interval = 1;
const float waypoint_tolerance = 0.1; //10 cm tolerance.

bool publish_info = false;

float prevWrist = 0;
float prevFinger = 0;
long int startTime = 0;
float minutesTime = 0;
float hoursTime = 0;

float drift_tolerance = 0.5; // meters

std_msgs::String msg;


string roverName = "";

char host[128];
char prev_state_machine[128];
// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int startDelayInSeconds = 30;
float timerTimeElapsed = 0;

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

/****************************
 * END ALPHABET GLOBAL SOUP *
 ****************************/


/******************
 * ROS Publishers *
 ******************/
ros::Publisher state_machine_publish;
ros::Publisher status_publisher;
ros::Publisher finger_angle_publish;
ros::Publisher wrist_angle_publish;
ros::Publisher info_log_publisher;
ros::Publisher drive_control_publish;
ros::Publisher heartbeat_publisher;
ros::Publisher rover_info_publisher;
ros::Publisher rover_info_timer_publisher;

void setupPublishers( ros::NodeHandle &ros_handle, string published_name )
{
    status_publisher = ros_handle.advertise<std_msgs::String>((published_name + "/status"), 1, true);
    state_machine_publish = ros_handle.advertise<std_msgs::String>((published_name + "/state_machine"), 1, true);
    finger_angle_publish = ros_handle.advertise<std_msgs::Float32>((published_name + "/fingerAngle/cmd"), 1, true);
    wrist_angle_publish = ros_handle.advertise<std_msgs::Float32>((published_name + "/wristAngle/cmd"), 1, true);
    info_log_publisher = ros_handle.advertise<std_msgs::String>("/infoLog", 1, true);
    drive_control_publish = ros_handle.advertise<geometry_msgs::Twist>((published_name + "/driveControl"), 10);
    heartbeat_publisher = ros_handle.advertise<std_msgs::String>((published_name + "/behaviour/heartbeat"), 1, true);
    rover_info_timer_publisher = ros_handle.advertise<std_msgs::String>((published_name + "/infoTimer"), 1, true);
    rover_info_publisher = ros_handle.advertise<swarmie_msgs::InfoMessage>(("roverInfo"), 6, true);

    roverName = published_name;
}

/*******************
 * ROS Subscribers *
 *******************/
ros::Subscriber joy_subscriber;
ros::Subscriber mode_subscriber;
ros::Subscriber target_subscriber;
ros::Subscriber raw_odom_subscriber;
ros::Subscriber odometry_subscriber;
ros::Subscriber map_subscriber;
ros::Subscriber rover_info_subscriber;

/******************************************
 * ROS Callback Functions for Subscribers *
 ******************************************/
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void odomHandler(const nav_msgs::Odometry::ConstPtr& message);
void odomAndAccelHandler(const nav_msgs::Odometry::ConstPtr& message);
void odomAccelAndGPSHandler(const nav_msgs::Odometry::ConstPtr& message);
void manualWaypointHandler(const swarmie_msgs::Waypoint& message);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);
void roverInfoHandler(const swarmie_msgs::InfoMessage& message);

void setupSubscribers( ros::NodeHandle &ros_handle, string published_name )
{
    joy_subscriber = ros_handle.subscribe((published_name + "/joystick"), 10, joyCmdHandler);
    mode_subscriber = ros_handle.subscribe((published_name + "/mode"), 1, modeHandler);
    target_subscriber = ros_handle.subscribe((published_name + "/targets"), 10, targetHandler);
    raw_odom_subscriber = ros_handle.subscribe((published_name + "/odom/"), 10, odomHandler);
    odometry_subscriber = ros_handle.subscribe((published_name + "/odom/filtered"), 10, odomAndAccelHandler);
    map_subscriber = ros_handle.subscribe((published_name + "/odom/ekf"), 10, odomAccelAndGPSHandler);
    rover_info_subscriber = ros_handle.subscribe("/roverInfo", 10, roverInfoHandler);

}

/**************
 * ROS Timers *
 **************/
ros::Timer state_machine_timer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;
ros::Timer publish_info_timer;

/***********************
 * ROS Timer Functions *
 ***********************/
void runStateMachines(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void publishRoverInfoTimerEventHandler(const ros::TimerEvent& event);

void setupTimerCallbacks( ros::NodeHandle &ros_handle )
{
    publish_status_timer = ros_handle.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    state_machine_timer = ros_handle.createTimer(ros::Duration(state_machines_loop), runStateMachines);
    publish_heartbeat_timer = ros_handle.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);
    publish_info_timer = ros_handle.createTimer(ros::Duration(info_publish_interval), publishRoverInfoTimerEventHandler);

}

/******************
 * SIGINT Handler *
 ******************/
void sigintEventHandler(int signal);

/*****************
 * Sensor Inputs *
 *****************/
LogicInputs inputs;
LogicOutputs outputs;
IOTable iotable = { &inputs, &outputs };
//iotable.inputs = &inputs;
//iotable.outputs = &outputs;
/***********************
 * Logic State Machine *
 ***********************/
LogicMachine logic_machine( &iotable );
    SearchState search_state( &iotable );
    PickUpState pickup_state( &iotable );
    FindHomeState findhome_state( &iotable );
    Avoid avoid_state( &iotable );
    AvoidHomeState avoidhome_state( &iotable );
    AvoidCubeState avoidcube_state( &iotable );
//    MotorCalibState motorcalib_state( &iotable );
//    RotationalCalibState rotationalcalib_state( &iotable );
    DropOffState dropoff_state( &iotable );
    InitState init_state( &iotable );

void setupLogicMachine( ros::NodeHandle* nh )
{
    /* add States */
    logic_machine.addState( init_state.getIdentifier(), dynamic_cast<State *>(&init_state) );
    logic_machine.addState( search_state.getIdentifier(), dynamic_cast<State *>(&search_state) );
    logic_machine.addState( pickup_state.getIdentifier(), dynamic_cast<State *>(&pickup_state) );
    logic_machine.addState( findhome_state.getIdentifier(), dynamic_cast<State *>(&findhome_state) );
    logic_machine.addState( dropoff_state.getIdentifier(), dynamic_cast<State *>(&dropoff_state) );
    logic_machine.addState( avoid_state.getIdentifier(), dynamic_cast<State *>(&avoid_state) );
    logic_machine.addState( avoidhome_state.getIdentifier(), dynamic_cast<State *>(&avoidhome_state) );
    logic_machine.addState( avoidcube_state.getIdentifier(), dynamic_cast<State *>(&avoidcube_state) );
    search_state.node_handle = nh;
    search_state.setupSubscribers();
    search_state.setupPublishers();

    return;
}





/*****************
 * MAIN FUNCTION *
 *****************/

int main(int argc, char **argv)
{
    gethostname(host, sizeof (host));
    string hostname(host);
    string published_name;

    //Determine this Rover's name
    if (argc >= 2)
    {
        published_name = argv[1];
        cout << "Welcome to the world of tomorrow " << published_name
             << "!  Behaviour turnDirectionule started." << endl;
    }
    else
    {
        published_name = hostname;
        cout << "No Name Selected. Default is: " << published_name << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (published_name + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
    ros::NodeHandle ros_handle;

    iotable.inputs->rover_name = published_name;
    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);

    setupSubscribers( ros_handle, published_name );

    //Sonar Stuff
    message_filters::Subscriber<sensor_msgs::Range> sonar_left_subscriber(ros_handle, (published_name + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonar_center_subscriber(ros_handle, (published_name + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonar_right_subscriber(ros_handle, (published_name + "/sonarRight"), 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonar_left_subscriber, sonar_center_subscriber, sonar_right_subscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));


    setupPublishers( ros_handle, published_name );
    setupTimerCallbacks( ros_handle );
    setupLogicMachine( &ros_handle );

    //TBD How to wrap this section up
    std_msgs::String msg;
    msg.data = "Log Started";
    info_log_publisher.publish(msg);

    stringstream ss;
    ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
    msg.data = ss.str();
    info_log_publisher.publish(msg);
    timerStartTime = time(0);


    swarmie_msgs::InfoMessage infoMsg;
    infoMsg.name = roverName;
    infoMsg.x = inputs.odom_accel_gps.x;
    infoMsg.y = inputs.odom_accel_gps.y;
    infoMsg.sonar_left = inputs.us_left;
    infoMsg.sonar_right = inputs.us_right;
    infoMsg.sonar_center = inputs.us_center;
    infoMsg.state = logic_machine.getCurrentIdentifier();
    infoMsg.number_of_cubes = TagUtilities::numberOfTags(&inputs.tags, 0);
    infoMsg.number_of_base_tags = TagUtilities::numberOfTags(&inputs.tags, 256);
    rover_info_publisher.publish(infoMsg);

    inputs.rover_name = published_name;

    ros::spin();

    return EXIT_SUCCESS;
}








/************************
 * Subsequent Functions *
 ************************/

// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void runStateMachines(const ros::TimerEvent&)
{
    // time since timerStartTime was set to current time
    //timerTimeElapsed = time(0) - timerStartTime;
    // Robot is in automode
    inputs.time = ros::Time::now();
    if (currentMode == 2 || currentMode == 3)
    {
        /***************************
         * State Machine Execution *
         ***************************/
        logic_machine.run();
        //std::cout << "current state is..." << logic_machine.getCurrentIdentifier() << std::endl;

        /*****************
         * Drive Portion *
         *****************/
        if( outputs.current_waypoint )
        {
            int left = 0;
            int right = 0;
            std::tuple<int,int> output;

            outputs.current_waypoint->run();

            output = outputs.current_waypoint->getOutput();
            left = std::get<0>( output );
            right = std::get<1>( output );

//            std::cout << "Left is " << left << std::endl;
//            std::cout << "Right is " << right << std::endl;
//            std::cout << "Goal X, Y: " << inputs.goal_x << ", " << inputs.goal_y << std::endl;
            /* TODO: add else messaging */
            sendDriveCommand( left, right );
        }
        else
        {
//            std::cout << "Current Waypoint is null" << std::endl;
            sendDriveCommand( 0, 0 );
        }
        /*******************
         * Gripper Portion *
         *******************/
        sendGripperPosition( outputs.gripper_position );
    }
    else
    {
        sendGripperPosition( Gripper::HOVER_OPEN );
        /* some output about manual mode? */
    }
/*
    for( int i = 0; i < inputs.infoVector.size(); i++ )
    {
        std::cout << i << ":" << inputs.infoVector.at(i).name << std::endl;
    }
*/
}

void sendDriveCommand(double left, double right)
{
    geometry_msgs::Twist velocity;

    velocity.linear.x = left,
    velocity.angular.z = right;

    // publish the drive commands
    drive_control_publish.publish(velocity);
}

void sendGripperPosition( Gripper::Position pos )
{
    std_msgs::Float32 wrist;
    std_msgs::Float32 fingers;
    std::tuple<double,double> values = Gripper::getWristFingerValuesForPosition( pos );

    wrist.data = std::get<0>( values );
    fingers.data = std::get<1>( values );

    wrist_angle_publish.publish( wrist );
    finger_angle_publish.publish( fingers );
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void roverInfoHandler(const swarmie_msgs::InfoMessage& message){
        roverInfo messageInfo;
        messageInfo.name = message.name;
        messageInfo.number_of_cubes = message.number_of_cubes;
        messageInfo.number_of_base_tags = message.number_of_base_tags;
        messageInfo.x = message.x;
        messageInfo.y = message.y;
        messageInfo.state = message.state;
        messageInfo.sonar_left = message.sonar_left;
        messageInfo.sonar_right = message.sonar_right;
        messageInfo.sonar_center = message.sonar_center;

        bool roverExists = false;
        for(int i = 0; i < inputs.infoVector.size(); i++){
            if(messageInfo.name == inputs.infoVector.at(i).name){
                inputs.infoVector.at(i) = messageInfo;
                roverExists = true;
                break;
            }
        }
        if(!roverExists){
            inputs.infoVector.push_back(messageInfo);
        }
        std::sort( inputs.infoVector.begin(), inputs.infoVector.end() );
}

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
    // Don't pass April tag data to the logic controller if the robot is not in autonomous mode.
    // This is to make sure autonomous behaviours are not triggered while the rover is in manual mode.
//    if(currentMode == 0 || currentMode == 1)
  //      return;
    std_msgs::String tag_output;
    std::stringstream ss;

    inputs.tags.clear();
    inputs.cubes.clear();
    inputs.examiner.clear();
    if (message->detections.size() > 0)
    {
        //std::cout << "Round of Tags" << std::endl;
        for (int i = 0; i < message->detections.size(); i++)
        {
            // Package up the ROS AprilTag data into our own type that does not rely on ROS.
            Tag loc;
            loc.setID( message->detections[i].id );

            // Pass the position of the AprilTag
            geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
            loc.setPosition( make_tuple( tagPose.pose.position.x,
                                         tagPose.pose.position.y,
                                         tagPose.pose.position.z ) );

            // Pass the orientation of the AprilTag
            loc.setOrientation( ::boost::math::quaternion<float>( tagPose.pose.orientation.x,
                                                                  tagPose.pose.orientation.y,
                                                                  tagPose.pose.orientation.z,
                                                                  tagPose.pose.orientation.w ) );
            inputs.tags.push_back( loc );
//            std::cout << loc << std::endl;
        }
/*        double average_yaw = 0.0;
        for( int i = 0; i < inputs.tags.size(); i++ )
        {
            average_yaw += inputs.tags.at(i).calcYaw();
        }
        average_yaw /= inputs.tags.size();

        double average_roll = 0.0;
        for( int i = 0; i < inputs.tags.size(); i++ )
        {
            average_roll += inputs.tags.at(i).calcRoll();
        }
        average_roll /= inputs.tags.size();

        double average_pitch = 0.0;
        for( int i = 0; i < inputs.tags.size(); i++ )
        {
            average_pitch += inputs.tags.at(i).calcPitch();
        }
        average_pitch /= inputs.tags.size();

        std::cout << "Average R/P/Y:" << average_roll << "/" << average_pitch << "/" << average_yaw << std::endl; */
        if( TagUtilities::hasTag( &inputs.tags, 0 ) )
        {
            for( int i = 0; i < inputs.tags.size(); i++ )
            {
                bool found = false;
                for( int j = 0; j < inputs.cubes.size(); j++ )
                {
                    found = inputs.cubes.at(j).checkTag( inputs.tags.at(i) );
                    if( found )
                    {
                        //std::cout << "same cube!" << std::endl;
                        break;
                    }
                }
                if( !found )
                    inputs.cubes.push_back( Cube( inputs.tags.at(i) ) );

            }
        }
        //std::cout << "Number of cubes " << inputs.cubes.size() << std::endl;
        //std::cout << "Closest Tag Dist:" << TagUtilities::getDistance( TagUtilities::getClosestTag( &inputs.tags, 256 ) ) << std::endl;
        inputs.examiner.loadTags( inputs.tags );
//        double z = inputs.cubes.at(0).getPositionZ();

  //      std::cout << "Distance to Cube:" << sqrt( ( z * z ) - ( 0.145 * 0.145 ) ) << std::endl;
    }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

/* this is awkward... do nothing for now*/
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonar_left, const sensor_msgs::Range::ConstPtr& sonar_center, const sensor_msgs::Range::ConstPtr& sonar_right)
{
    inputs.us_left = sonar_left->range;
    inputs.us_right = sonar_right->range;
    inputs.us_center = sonar_center->range;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& message)
{
    inputs.raw_odom.x = message->pose.pose.position.x;
    inputs.raw_odom.y = message->pose.pose.position.x;
    inputs.odom_accel.x = message->pose.pose.position.x;
    inputs.odom_accel.y = message->pose.pose.position.y;
}
void odomAndAccelHandler(const nav_msgs::Odometry::ConstPtr& message)
{
    //Get (x,y) location directly from pose
  //  inputs.odom_accel.x = message->pose.pose.position.x - outputs.offset_x;
  //  inputs.odom_accel.y = message->pose.pose.position.y - outputs.offset_y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    inputs.odom_accel.theta = yaw;

    inputs.linear_vel_odom_accel = message->twist.twist.linear.x;
    inputs.angular_vel_odom_accel = message->twist.twist.angular.z;
}

void odomAccelAndGPSHandler(const nav_msgs::Odometry::ConstPtr& message)
{
  //Get (x,y) location directly from pose
  inputs.odom_accel_gps.x = message->pose.pose.position.x - outputs.offset_x;
  inputs.odom_accel_gps.y = message->pose.pose.position.y - outputs.offset_y;

  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  inputs.odom_accel_gps.theta = yaw;

  inputs.linear_vel_odom_accel_gps = message->twist.twist.linear.x;
  inputs.angular_vel_odom_accel_gps = message->twist.twist.angular.z;
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message)
{
    const int max_motor_cmd = 255;
  std_msgs::String msg;
  msg.data = "";

    if (currentMode == 0 || currentMode == 1)
    {
        float linear  = abs(message->axes[4]) >= 0.1 ? message->axes[4]*max_motor_cmd : 0.0;
        float angular = abs(message->axes[3]) >= 0.1 ? message->axes[3]*max_motor_cmd : 0.0;

        float left = linear - angular;
        float right = linear + angular;

        if(left > max_motor_cmd)
            left = max_motor_cmd;
        else if(left < -max_motor_cmd)
            left = -max_motor_cmd;

        if(right > max_motor_cmd)
            right = max_motor_cmd;
        else if(right < -max_motor_cmd)
            right = -max_motor_cmd;

        sendDriveCommand(left, right);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
    std_msgs::String msg;
    msg.data = "DTCC";
    status_publisher.publish(msg);
}

void sigintEventHandler(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "";
  heartbeat_publisher.publish(msg);
}
void publishRoverInfoTimerEventHandler(const ros::TimerEvent&)
{
    swarmie_msgs::InfoMessage infoMsg;
    infoMsg.name = roverName;
    infoMsg.x = inputs.odom_accel_gps.x;
    infoMsg.y = inputs.odom_accel_gps.y;
    infoMsg.sonar_left = inputs.us_left;
    infoMsg.sonar_right = inputs.us_right;
    infoMsg.sonar_center = inputs.us_center;
    infoMsg.state = logic_machine.getCurrentIdentifier();
    infoMsg.number_of_cubes = TagUtilities::numberOfTags(&inputs.tags, 0);
    infoMsg.number_of_base_tags = TagUtilities::numberOfTags(&inputs.tags, 256);
    rover_info_publisher.publish(infoMsg);

    publish_info = true;
}

long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();

  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec*1e3 + t.nsec/1e6;
}


void humanTime() {
  float timeDiff = (getROSTimeInMilliSecs()-startTime)/1e3;
  if (timeDiff >= 60) {
    minutesTime++;
    startTime += 60  * 1e3;
    if (minutesTime >= 60) {
      hoursTime++;
      minutesTime -= 60;
    }
  }
  timeDiff = floor(timeDiff*10)/10;

  double intP, frac;
  frac = modf(timeDiff, &intP);
  timeDiff -= frac;
  frac = round(frac*10);
  if (frac > 9) {
    frac = 0;
  }
  //cout << "System has been Running for :: " << hoursTime << " : hours " << minutesTime << " : minutes " << timeDiff << "." << frac << " : seconds" << endl; //you can remove or comment this out it just gives indication something is happening to the log file
}
