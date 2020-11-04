#define LOG_ERROR 0
#define LOG_NORMAL 1
#define LOG_EMERGENCY 2
#define LOG_IMPORTANT 3

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

//Pix message
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandHome.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"

// UAV Mission Service
#include "drone_contest/mission_start.h"



class UAV_control_class
{
    private:

    ////////////////////////////////////////////////////////////////////////////
    //////                                                                 /////
    //////                                                                 /////
    //////                                                                 /////
    //////                         mission                                 /////
    //////                                                                 /////
    //////                                                                 /////
    ////////////////////////////////////////////////////////////////////////////
    
    bool mission_flag = false;
    double mission_waypoint_lon;
    double mission_waypoint_lat;
    int return_time;

    ///UAV variable
    ///state representing flag
    bool offboard_flag;
    bool armed_flag;

    ///Pix state/position/velocity/target
    mavros_msgs::State current_state;
    nav_msgs::Odometry pix_poseLocal;
    geometry_msgs::Quaternion current_orientation;

    geometry_msgs::PoseStamped tracker_result;

    //Pix pub
    ros::Publisher setpoint_raw_pub;

    //UAV pub
    ros::Publisher pub_log_data;

    //Pix sub
    ros::Subscriber sub_pix_state;
    ros::Subscriber sub_pix_poseLocal;
    ros::Subscriber sub_pix_GPSLocal;
    ros::Subscriber sub_tracker_result;

    //Pix service client
    ros::ServiceClient client_cmdArming;
    ros::ServiceClient set_mode_client;

    sub_pix_GPSLocal = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/mavros/local_position/odom", 1, &UAV_control_class::cb_pix_GPSLocal,this);
    void cb_pix_GPSLocal(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    bool fn_mission_start(drone_contest::mission_start::Request &req,
                          drone_contest::mission_start::Response &res);

    srv_mission_start = nh.advertiseService("take_off", &fn_mission_start,this);








    ////////////////////////////////////////////////////////////////////////////
    //////                                                                 /////
    //////                                                                 /////
    //////                                                                 /////
    //////                         /mission                                /////
    //////                                                                 /////
    //////                                                                 /////
    ////////////////////////////////////////////////////////////////////////////



//    //Pix pub
//    ros::Publisher setpoint_raw_pub;

//    //UAV pub
//    ros::Publisher pub_log_data;

//    //Pix sub
//    ros::Subscriber sub_pix_state;
//    ros::Subscriber sub_pix_poseLocal;
//    ros::Subscriber sub_tracker_result;

//    //Pix service client
//    ros::ServiceClient client_cmdArming;
//    ros::ServiceClient set_mode_client;



    //UAV service server
    bool fn_take_off(drone_contest::take_off::Request &req,
                     drone_contest::take_off::Response &res);
    bool fn_landing(drone_contest::landing::Request &req,
                    drone_contest::landing::Response &res);
    bool fn_send_mission_info(drone_contest::ui_mission_request::Request &req,
                              drone_contest::ui_mission_request::Response &res);
    bool fn_automatic_mission_start(drone_contest::automatic_mission_start::Request &req,
                                    drone_contest::automatic_mission_start::Response &res);
    bool fn_next_mission(drone_contest::next_mission::Request &req,
                         drone_contest::next_mission::Response &res);
    bool fn_previous_mission(drone_contest::previous_mission::Request &req,
                             drone_contest::previous_mission::Response &res);
    bool fn_pause_mission(drone_contest::pause_mission::Request &req,
                          drone_contest::pause_mission::Response &res);
    bool fn_return_home(drone_contest::return_home::Request &req,
                        drone_contest::return_home::Response &res);
    bool fn_emergency_landing(drone_contest::emergency_landing::Request &req,
                              drone_contest::emergency_landing::Response &res);




    /*
    * UAV_control_mode
    * * not ready         : -1
    * * standby           :  0
    * * take off          :  1
    * * landing           :  2
    * * move to waypoint  :  3
    * * hover on waypoint :  4
    * * pause             :  5
    * * return home       :  6
    * * emergency         :  7
    */

    ///Pix state/position/velocity/target
    mavros_msgs::State current_state;
    nav_msgs::Odometry pix_poseLocal;
    geometry_msgs::Quaternion current_orientation;

    ///LOS variable
    float xy_dist;
    float xyz_dist;
    float xyz_total_dist;
    float yaw_dist;
    geometry_msgs::Point start_wp;
    geometry_msgs::Point end_wp;
    mavros_msgs::PositionTarget pause_target;
    mavros_msgs::PositionTarget target_position;
    geometry_msgs::Vector3 leftover;
    geometry_msgs::Vector3 total_distance;

    ///position target control type arranged
    int velocity_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                mavros_msgs::PositionTarget::IGNORE_AFY|
                                mavros_msgs::PositionTarget::IGNORE_AFZ|
                                mavros_msgs::PositionTarget::IGNORE_PX|
                                mavros_msgs::PositionTarget::IGNORE_PY|
                                mavros_msgs::PositionTarget::IGNORE_PZ|
                                mavros_msgs::PositionTarget::IGNORE_YAW;

    int position_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                mavros_msgs::PositionTarget::IGNORE_AFY|
                                mavros_msgs::PositionTarget::IGNORE_AFZ|
                                mavros_msgs::PositionTarget::IGNORE_VX|
                                mavros_msgs::PositionTarget::IGNORE_VY|
                                mavros_msgs::PositionTarget::IGNORE_VZ|
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    int yaw_align_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                 mavros_msgs::PositionTarget::IGNORE_AFY|
                                 mavros_msgs::PositionTarget::IGNORE_AFZ|
                                 mavros_msgs::PositionTarget::IGNORE_VX|
                                 mavros_msgs::PositionTarget::IGNORE_VY|
                                 mavros_msgs::PositionTarget::IGNORE_VZ|
                                 mavros_msgs::PositionTarget::IGNORE_YAW;

    int waypoint_control_type = mavros_msgs::PositionTarget::IGNORE_AFX|
                                mavros_msgs::PositionTarget::IGNORE_AFY|
                                mavros_msgs::PositionTarget::IGNORE_AFZ|
                                mavros_msgs::PositionTarget::IGNORE_PX|
                                mavros_msgs::PositionTarget::IGNORE_PY|
                                mavros_msgs::PositionTarget::IGNORE_PZ|
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    ///UAV log
    bool log_cnt;
    std::string loglog;
    std::string loglog_old;

///UAV control mode variable

    ///take off
    double landing_cur; // current altitude in landing seq.
    ros::Time last_request; // for loop time
    ///landing
    mavros_msgs::SetMode offb_set_mode; // for OFFBOARD
    double landing_old; // previous altitude in landing seq.
    ros::Time landing_time; // for landing loop time
    ///ui_mission_request
    int wp_size;
    std::vector <geometry_msgs::Point> uav_wp_local;
    std::vector <double> uav_heading;
    ///mission
    int wp_idx = -1;
    ///pause_mode
    bool pause_toggle;
    ///auto mode
    ros::Time arrive_time;
    bool auto_arrive_cnt;

    ///Px4 state feedback
    void cb_pix_state(const mavros_msgs::State::ConstPtr& msg);
    void cb_pix_local(const nav_msgs::Odometry::ConstPtr &msg);
    void cb_tracker_result(const geometry_msgs::PoseStamped::ConstPtr &msg);


    public:
    ros::NodeHandle nh;

    /// For main function
    void spinOnce();

    ///additional function
    geometry_msgs::Vector3 Quat2Angle(geometry_msgs::Quaternion quat);
    geometry_msgs::Vector3 euclidean_xyz_distance(geometry_msgs::Point start, geometry_msgs::Point end);
    float yaw_distance(float start, float end);
    void integrated_distance_calculator();
    void set_start_end_wp(int idx);
    void log_pub_ros_info(int color, std::string log);
    void LOS_move();
//    void move_wo_LoS();
//    void move_vel_wo_cond();
//    void move_pos_wo_cond();

    int UAV_control_mode;

    ///velocity & take off height & threshold for detecting arrive(ROS parameter)
    double wp_vel;
    double height_vel;
    double landing_vel;
    double yaw_vel;
    double take_off_height;
    double threshold_distance;
    double arrive_distance;
    double threshold_yaw;
    double LOS_radius;
    double auto_arrive_delay;
    int test_mode;

    UAV_control_class()
    {
        ///control topic pub
        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        ///log topic pub
        pub_log_data = nh.advertise<drone_contest::log_data>("/drone_contest/log_data", 30);
        pub_log_data = nh.advertise<drone_contest::log_data>("/drone_contest/log_data", 30);

        ///pixhawk data subs
        sub_pix_state = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &UAV_control_class::cb_pix_state,this);
        sub_pix_poseLocal = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &UAV_control_class::cb_pix_local,this);
        sub_tracker_result = nh.subscribe<geometry_msgs::PoseStamped>("/tracker_result", 1, &UAV_control_class::cb_tracker_result,this);


        ///px4 service client
        client_cmdArming = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ///UAV service client : read wp line
        client_request_waypoint = nh.serviceClient<drone_contest::linenumin_waypointout>("/wp_reader/request");


        ///UAV control service server
        srv_take_off = nh.advertiseService("take_off", &UAV_control_class::fn_take_off,this);
        srv_landing = nh.advertiseService("landing", &UAV_control_class::fn_landing,this);
        srv_send_mission_info = nh.advertiseService("send_mission_info", &UAV_control_class::fn_send_mission_info,this);
        srv_auto_mission = nh.advertiseService("auto_mission", &UAV_control_class::fn_automatic_mission_start,this);
        srv_next_mission = nh.advertiseService("next_mission", &UAV_control_class::fn_next_mission,this);
        srv_previous_mission = nh.advertiseService("previous_mission", &UAV_control_class::fn_previous_mission,this);
        srv_pause_mission = nh.advertiseService("pause_mission", &UAV_control_class::fn_pause_mission,this);
        srv_emergency_landing = nh.advertiseService("emergency_landing", &UAV_control_class::fn_emergency_landing,this);
        srv_return_home = nh.advertiseService("return_home", &UAV_control_class::fn_return_home,this);

        ///initializing variable
        armed_flag = false;
        UAV_control_mode = not_ready_mode;
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pause_toggle = false;
        last_request = ros::Time::now();
        offb_set_mode.request.custom_mode = "OFFBOARD";
        log_cnt = false;
        auto_flag = false;
        auto_arrive_cnt = false;
    }

    ~UAV_control_class()
    {

    }
};
