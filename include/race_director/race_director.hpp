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

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "lart_msgs/srv/heartbeat.hpp"

#include "lart_msgs/msg/acu.hpp"
#include "lart_msgs/msg/res.hpp"
#include "lart_msgs/msg/jetson.hpp"
#include "lart_msgs/msg/dv_dynamics1.hpp"
#include "lart_msgs/msg/dv_dynamics2.hpp"
#include "lart_msgs/msg/slam_stats_can.hpp"



#define TIMESTAMP_MARGIN 3.0 // seconds

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
        std::chrono::steady_clock::time_point ready_change;
        bool ready_change_set = false;
        bool bag_recording = false;

        int asms_state = 0;

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

    /* Functions */

        void res_callback(const lart_msgs::msg::Res::SharedPtr msg);

        void nodes_state_callback(const lart_msgs::msg::State::SharedPtr msg);

        void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
        
        void slam_callback(const lart_msgs::msg::SlamStats::SharedPtr msg);

        void combined_imu_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& acc_msg, 
                            const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& gyro_msg);

        void change_state(int new_state);

        void send_state_to_nodes();

        void send_handbook_msgs();

        /* Steering Service Related*/
        void request_steering_timestamp();

        void handle_steering_timestamp_response(rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedFuture future);

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
        
        /* Subscribers */
        rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber;
        rclcpp::Subscription<lart_msgs::msg::Acu>::SharedPtr acu_subscriber;
        rclcpp::Subscription<lart_msgs::msg::Res>::SharedPtr res_subscriber;
        rclcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr slam_subscriber;
        rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr nodes_state_subscriber;
        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> imu_acc_sub_;
        message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> imu_gyro_sub_;
        std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::msg::Vector3Stamped, geometry_msgs::msg::Vector3Stamped>> sync_;

};
#endif //RACE_DIRECTOR_HPP