#include "race_director/race_director.hpp"

RaceDirector::RaceDirector() : Node("race_director"){

    bool is_unit_test = false;
    if (const char *env = std::getenv("UNIT_TEST")) {
      is_unit_test = (std::string(env) == "1");
    }

    /* Publishers*/
    this->state_publisher = this->create_publisher<lart_msgs::msg::State>("/state", 10);

    /* Subscribers */
    this->acu_state_subscriber = this->create_subscription<lart_msgs::msg::State>("/acu/state", 10, std::bind(&RaceDirector::acu_state_callback, this, _1));
    this->nodes_state_subscriber = this->create_subscription<lart_msgs::msg::State>("/nodes/state", 10, std::bind(&RaceDirector::nodes_state_callback, this, _1));
    
    /* Services */
    if (!is_unit_test){
        this->steering_timestamp = this->create_client<lart_msgs::srv::Heartbeat>("steering/last_timestamp");
    
        while (!this->steering_timestamp->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for steering/last_timestamp service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for steering/last_timestamp service...");
        }
    
        this->steering_timestamp_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&RaceDirector::request_steering_timestamp, this));
    
        this->perception_timestamp = this->create_client<lart_msgs::srv::Heartbeat>("zed/last_timestamp");
        
        while (!this->perception_timestamp->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for zed/last_timestamp service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for zed/last_timestamp service...");
        }
        this->perception_timestamp_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&RaceDirector::request_perception_timestamp, this));
    }

    
    /* Threads */
    this->state_thread = std::thread([this]() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            this->send_state_to_nodes();
            rate.sleep();
        }
    });
    this->state_thread.detach();

}
RaceDirector::~RaceDirector(){
    if (this->state_thread.joinable()) {
        this->state_thread.join();
    }
}

void RaceDirector::acu_state_callback(const lart_msgs::msg::State::SharedPtr msg) {
    auto received_state = msg->data;
    if (received_state == current_state) {
        return;
    }

    switch (received_state) {
        case lart_msgs::msg::State::OFF:
            RCLCPP_INFO(this->get_logger(), "State changed to OFF");
            this->change_state(lart_msgs::msg::State::OFF);
            break;

        case lart_msgs::msg::State::READY:
            RCLCPP_INFO(this->get_logger(), "State changed to READY");
            this->ready_change = std::chrono::steady_clock::now();
            this->change_state(lart_msgs::msg::State::READY);
            break;
        case lart_msgs::msg::State::DRIVING:{
            std::chrono::duration<double> time_in_ready = std::chrono::steady_clock::now() - this->ready_change;
            if ( current_state == lart_msgs::msg::State::READY && time_in_ready > std::chrono::seconds(6)) {
                RCLCPP_INFO(this->get_logger(), "State changed to DRIVING");
                this->change_state(lart_msgs::msg::State::DRIVING);
            } else {
                RCLCPP_WARN(this->get_logger(), "Cannot change to DRIVING, not in READY state");
                return;
            }
            break;
        }
        case lart_msgs::msg::State::EMERGENCY:
            RCLCPP_INFO(this->get_logger(), "State changed to EMERGENCY");
            this->change_state(lart_msgs::msg::State::EMERGENCY);
            break;
    }

}

void RaceDirector::nodes_state_callback(const lart_msgs::msg::State::SharedPtr msg){
    if (current_state == lart_msgs::msg::State::EMERGENCY) return; // if the current state is emergency it is not possible to change
    
    auto received_state = msg->data;
    switch (received_state){
        case lart_msgs::msg::State::FINISH:
            if (current_state == lart_msgs::msg::State::DRIVING)
            this->change_state(lart_msgs::msg::State::FINISH);
        break;
            case lart_msgs::msg::State::EMERGENCY:
            this->change_state(lart_msgs::msg::State::EMERGENCY);
        break;
    }
}

#pragma region Steering Service

void RaceDirector::request_steering_timestamp(){
    auto request = std::make_shared<lart_msgs::srv::Heartbeat::Request>();
    auto future = this->steering_timestamp->async_send_request(
        request,
        std::bind(&RaceDirector::handle_steering_timestamp_response, this, _1));
}

void RaceDirector::handle_steering_timestamp_response(rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedFuture future){
    try {
        auto response = future.get();
        auto last_steering_timestamp = response->timestamp;
        auto now = this->now();

        if ((now - last_steering_timestamp).seconds() > TIMESTAMP_MARGIN) {
            this->change_state(lart_msgs::msg::State::EMERGENCY);
        }
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}
#pragma endregion

#pragma region Perception Service
void RaceDirector::request_perception_timestamp(){
    auto request = std::make_shared<lart_msgs::srv::Heartbeat::Request>();
    auto future = this->perception_timestamp->async_send_request(
        request,
        std::bind(&RaceDirector::handle_perception_timestamp_response, this, _1));
}

void RaceDirector::handle_perception_timestamp_response(rclcpp::Client<lart_msgs::srv::Heartbeat>::SharedFuture future){
    try {
        auto response = future.get();
        auto last_perception_timestamp = response->timestamp;
        auto now = this->now();

        if ((now - last_perception_timestamp).seconds() > TIMESTAMP_MARGIN) {
            this->change_state(lart_msgs::msg::State::EMERGENCY);
        }
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}
#pragma endregion

int RaceDirector::get_current_state(){
    return this->current_state;
}

void RaceDirector::change_state(int new_state) {
    std::lock_guard<std::mutex> lock(state_mutex);
    this->current_state = new_state;
}

void RaceDirector::send_state_to_nodes() {
    lart_msgs::msg::State msg;
    msg.header.stamp = this->now();

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        msg.data = this->current_state;
    }

    this->state_publisher->publish(msg);
}

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<RaceDirector>());
        rclcpp::shutdown();


    return 0;
}