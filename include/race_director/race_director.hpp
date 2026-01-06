#ifndef RACE_DIRECTOR_HPP
#define RACE_DIRECTOR_HPP

#include <cstdio>
#include <chrono>
#include <thread>
#include <cstdlib> 
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/srv/heartbeat.hpp"

#define TIMESTAMP_MARGIN 3.0 // seconds

using namespace std::placeholders;


class RaceDirector : public rclcpp::Node {
    public:
        RaceDirector();
        ~RaceDirector();
        
        void acu_state_callback(const lart_msgs::msg::State::SharedPtr msg);
        int get_current_state();


    private:

    /* Variables*/
        int previous_state = lart_msgs::msg::State::OFF;
        int current_state = lart_msgs::msg::State::OFF;
        std::chrono::steady_clock::time_point ready_change;
        bool ready_change_set = false;
        
        std::mutex state_mutex;
        std::thread state_thread;

        rclcpp::TimerBase::SharedPtr steering_timestamp_timer;
        rclcpp::TimerBase::SharedPtr perception_timestamp_timer;

    /* Functions */

        void nodes_state_callback(const lart_msgs::msg::State::SharedPtr msg);

        void change_state(int new_state);

        void send_state_to_nodes();

        /* Steering Service Related*/
        void request_steering_timestamp();

        void handle_steering_timestamp_response(rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedFuture future);

        /* Perception Service Related*/
        void request_perception_timestamp();

        void handle_perception_timestamp_response(rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedFuture future);

    /* Service Clients */
        rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedPtr steering_timestamp;

        rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedPtr perception_timestamp;

    /* Publishers */
        rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_publisher;

    /* Subscribers */
        rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr acu_state_subscriber;
        rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr nodes_state_subscriber;
};
#endif //RACE_DIRECTOR_HPP