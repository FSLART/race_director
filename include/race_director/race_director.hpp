#ifndef RACE_DIRECTOR_HPP
#define RACE_DIRECTOR_HPP

#include <cstdio>
#include <chrono>
#include <thread>
#include <cstdlib> 
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "lart_common.h"
#include "topics.h"

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "lart_msgs/srv/heartbeat.hpp"

#include "lart_msgs/msg/acu.hpp"
#include "lart_msgs/msg/vcu_rpm.hpp"
#include "lart_msgs/msg/res.hpp"
#include "lart_msgs/msg/jetson.hpp"
#include "lart_msgs/msg/dv_dynamics1.hpp"
#include "lart_msgs/msg/dv_dynamics2.hpp"
#include "lart_msgs/msg/slam_stats_can.hpp"
#include "lart_msgs/msg/cubemars_position_loop.hpp"
#include "lart_msgs/msg/cubemars_feedback.hpp"
#include "lart_msgs/msg/vcu_torque_target.hpp"
#include "lart_msgs/msg/vcu_rpm_target.hpp"


#define TIMESTAMP_MARGIN 1.0 // seconds

using namespace std::placeholders;


class RaceDirector : public rclcpp::Node {
    public:
        RaceDirector();
        ~RaceDirector();
        
        void acu_callback(const lart_msgs::msg::Acu::SharedPtr msg);
        int get_current_state();

    private:

    /* Variables*/
        int current_state = lart_msgs::msg::State::OFF;
        int current_mission = lart_msgs::msg::Mission::MANUAL;
        std::chrono::steady_clock::time_point ready_change;
        std::chrono::steady_clock::time_point last_state_change;
        std::chrono::steady_clock::time_point last_steering_timestamp {};
        bool ready_change_set = false;
        bool bag_recording = false;

        int asms_state = 0;

        int emergency_cause = lart_msgs::msg::Jetson::EMERGENCY_CAUSE_NONE;

        lart_msgs::msg::Jetson jetson_msg;

        //Handbook msgs
        lart_msgs::msg::DvDynamics1 dv_dynamics1_msg;
        lart_msgs::msg::DvDynamics2 dv_dynamics2_msg;
        lart_msgs::msg::SlamStatsCan slam_stats_can_msg;
        rclcpp::TimerBase::SharedPtr handbook_msgs_timer;

        std::mutex state_mutex;
        std::thread state_thread;

        rclcpp::TimerBase::SharedPtr steering_timestamp_timer;
        rclcpp::TimerBase::SharedPtr perception_timestamp_timer;
        rclcpp::TimerBase::SharedPtr send_jetson_msg_timer;
        rclcpp::TimerBase::SharedPtr bag_stop_timer;

    /* Functions */

        void vcu_control_feedback_callback(const lart_msgs::msg::VcuRpm::SharedPtr msg);

        void res_callback(const lart_msgs::msg::Res::SharedPtr msg);

        void nodes_state_callback(const lart_msgs::msg::State::SharedPtr msg);

        void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
        
        void slam_callback(const lart_msgs::msg::SlamStats::SharedPtr msg);

        void combined_imu_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& acc_msg, 
                            const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& gyro_msg);
        
        void torque_control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg);

        void rpm_control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg);

        void cubemars_feedback_callback(const lart_msgs::msg::CubemarsFeedback::SharedPtr msg);

        void change_state(int new_state);

        void schedule_bag_stop();

        void stop_bag_recording();

        void send_state_to_nodes();

        void send_handbook_msgs();

        void check_steering_timestamp();

        /* Perception Service Related*/
        void request_perception_timestamp();

        void handle_perception_timestamp_response(rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedFuture future);

    /* Service Clients */
        rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedPtr steering_timestamp;

        rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedPtr perception_timestamp;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_bag_recording_client;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_bag_recording_client;

    /* Publishers */
        rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher;
        rclcpp::Publisher<lart_msgs::msg::Jetson>::SharedPtr jetson_publisher;
        rclcpp::Publisher<lart_msgs::msg::SlamStatsCan>::SharedPtr slam_stats_can_publisher;
        rclcpp::Publisher<lart_msgs::msg::DvDynamics1>::SharedPtr dv_dynamics1_publisher;
        rclcpp::Publisher<lart_msgs::msg::DvDynamics2>::SharedPtr dv_dynamics2_publisher;
        rclcpp::Publisher<lart_msgs::msg::CubemarsPositionLoop>::SharedPtr cubemars_position_loop_publisher;
        rclcpp::Publisher<lart_msgs::msg::VcuTorqueTarget>::SharedPtr vcu_torque_target_publisher;
        rclcpp::Publisher<lart_msgs::msg::VcuRpmTarget>::SharedPtr vcu_rpm_target_publisher;
        rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr control_feedback_publisher;
        
        /* Subscribers */
        rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
        rclcpp::Subscription<lart_msgs::msg::Acu>::SharedPtr acu_subscriber;
        rclcpp::Subscription<lart_msgs::msg::VcuRpm>::SharedPtr vcu_rpm_subscriber;
        rclcpp::Subscription<lart_msgs::msg::Res>::SharedPtr res_subscriber;
        rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr slam_subscriber;
        rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr nodes_state_subscriber;
        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> imu_acc_sub_;
        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> imu_gyro_sub_;
        std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::msg::Vector3Stamped, geometry_msgs::msg::Vector3Stamped>> imu_sync_;
        rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr control_torque_subscriber;
        rclcpp::Subscription<lart_msgs::msg::DynamicsCMD>::SharedPtr control_rpm_subscriber;
        rclcpp::Subscription<lart_msgs::msg::CubemarsFeedback>::SharedPtr cubemars_feedback_subscriber;

};
#endif //RACE_DIRECTOR_HPP