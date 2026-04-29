#include "race_director/race_director.hpp"

RaceDirector::RaceDirector() : Node("race_director"){

    bool is_unit_test = false;
    if (const char *env = std::getenv("UNIT_TEST")) {
      is_unit_test = (std::string(env) == "1");
    }

    /* Publishers*/
    this->state_publisher = this->create_publisher<lart_msgs::msg::State>("/state", 10);
    this->jetson_publisher = this->create_publisher<lart_msgs::msg::Jetson>("/jetson", 10);
    this->slam_stats_can_publisher = this->create_publisher<lart_msgs::msg::SlamStatsCan>("/dv/slam_stats", 10);
    this->dv_dynamics1_publisher = this->create_publisher<lart_msgs::msg::DvDynamics1>("/dv/dynamics1", 10);
    this->dv_dynamics2_publisher = this->create_publisher<lart_msgs::msg::DvDynamics2>("/dv/dynamics2", 10);
    this->cubemars_position_loop_publisher = this->create_publisher<lart_msgs::msg::CubemarsPositionLoop>("/cubemars/position_loop", 10);
    this->vcu_torque_target_publisher = this->create_publisher<lart_msgs::msg::VcuTorqueTarget>("/vcu/torque_target", 10);

    /* Subscribers */
    this->acu_subscriber = this->create_subscription<lart_msgs::msg::Acu>("/acu", 10, std::bind(&RaceDirector::acu_callback, this, _1));
    this->res_subscriber = this->create_subscription<lart_msgs::msg::Res>("/res", 10, std::bind(&RaceDirector::res_callback, this, _1));
    this->nodes_state_subscriber = this->create_subscription<lart_msgs::msg::State>("/state/nodes", 10, std::bind(&RaceDirector::nodes_state_callback, this, _1));
    this->mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>("/mission", 10, std::bind(&RaceDirector::mission_callback, this, _1));
    imu_acc_sub_.subscribe(this, "/imu/acceleration");
    imu_gyro_sub_.subscribe(this, "/imu/angular_velocity");
    
    //sync imu acc and gyro subs
    imu_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::msg::Vector3Stamped, geometry_msgs::msg::Vector3Stamped>>(imu_acc_sub_, imu_gyro_sub_, 10);
    imu_sync_->registerCallback(std::bind(&RaceDirector::combined_imu_callback, this, _1, _2));
    
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

    this->start_bag_recording_client = this->create_client<std_srvs::srv::Trigger>("/start_recording");
    this->stop_bag_recording_client = this->create_client<std_srvs::srv::Trigger>("/stop_recording");

    this->handbook_msgs_timer = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&RaceDirector::send_handbook_msgs, this));

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

void RaceDirector::acu_callback(const lart_msgs::msg::Acu::SharedPtr msg) {
    this->asms_state = msg->asms;
    if (this->asms_state == 0){
        return;
    }

    //start recording bag if ign is 1, stop recording if ign is 0
    if (msg->ign == 1){
        if (!this->bag_recording){
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            this->start_bag_recording_client->async_send_request(request);
            this->bag_recording = true;
        }
    }else{
        if (this->bag_recording){
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            this->stop_bag_recording_client->async_send_request(request);
            this->bag_recording = false;
        }
    }

    auto received_state = msg->as_state;

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
            this->ready_change_set = true;
            this->change_state(lart_msgs::msg::State::READY);
            break;
        case lart_msgs::msg::State::EMERGENCY:
            RCLCPP_INFO(this->get_logger(), "State changed to EMERGENCY");
            this->change_state(lart_msgs::msg::State::EMERGENCY);
            break;
    }
}

void RaceDirector::res_callback(const lart_msgs::msg::Res::SharedPtr msg){
    int res_signal = msg->signal;
    if (this->current_state == lart_msgs::msg::State::READY){
        std::chrono::duration<double> time_in_ready = std::chrono::steady_clock::now() - this->ready_change;
        if((res_signal == 5 || res_signal == 7) && time_in_ready > std::chrono::seconds(6)) {
            this->change_state(lart_msgs::msg::State::DRIVING);
        }
    }

    if (res_signal == 0){
        this->change_state(lart_msgs::msg::State::EMERGENCY);
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

void RaceDirector::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg){
    //Get jetson temperature
    std::ifstream temp_file("/sys/devices/virtual/thermal/thermal_zone1/temp");
    int temp;
    temp_file >> temp;
    float final_temp = temp / 1000.0 ;

    //Get CPU usage


    //Get GPU usage


    lart_msgs::msg::Jetson jetson_msg;
    jetson_msg.header.stamp = this->now();
    jetson_msg.as_mission = msg->data;
    jetson_msg.as_state = this->current_state;
    jetson_msg.temperature = static_cast<int8_t>(final_temp);
    jetson_msg.cpu = 0;
    jetson_msg.gpu = 0;

    this->jetson_publisher->publish(jetson_msg);
}

void RaceDirector::slam_callback(const lart_msgs::msg::SlamStats::SharedPtr msg){
    this->slam_stats_can_msg.lap_counter = msg->lap_count;
    this->slam_stats_can_msg.cones_count_actual = msg->cones_count_current;
    this->slam_stats_can_msg.cones_count_all = msg->cones_count_all;
}

void RaceDirector::combined_imu_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& acc_msg, 
                            const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& gyro_msg){
    this->dv_dynamics2_msg.acceleration_longitudinal = acc_msg->vector.x;
    this->dv_dynamics2_msg.acceleration_lateral= acc_msg->vector.y;
    this->dv_dynamics2_msg.yaw_rate = gyro_msg->vector.z;
}

void RaceDirector::control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){
    float angle = msg->steering_angle;
    float sw_angle = -61.6073*pow(angle, 4)+449.05708*pow(angle, 3)+16.71117*pow(angle, 2)+156.50789*angle;
    float rel_current = msg->acc_cmd*100;
    
    this->dv_dynamics1_msg.steering_angle_target = sw_angle;
    this->dv_dynamics1_msg.steering_angle_actual = 0.0;
    this->dv_dynamics1_msg.speed_target = 0.0;
    this->dv_dynamics1_msg.speed_actual = 0.0;
    this->dv_dynamics1_msg.brake_hydr_target = 0.0;
    this->dv_dynamics1_msg.brake_hydr_actual = 0.0;
    this->dv_dynamics1_msg.motor_moment_target = rel_current;
    this->dv_dynamics1_msg.motor_moment_actual = 0.0;
    
    //prepare control commands for vcu and cubemars
    lart_msgs::msg::VcuTorqueTarget vcu_msg;
    vcu_msg.header.stamp = this->now();
    vcu_msg.torque_target = rel_current;
    this->vcu_torque_target_publisher->publish(vcu_msg);

    lart_msgs::msg::CubemarsPositionLoop cubemars_msg;
    cubemars_msg.header.stamp = this->now();
    cubemars_msg.steering_angle_target = sw_angle;
    this->cubemars_position_loop_publisher->publish(cubemars_msg);


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
    if(this->current_state == lart_msgs::msg::State::EMERGENCY){
        return;
    }
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

void RaceDirector::send_handbook_msgs() {
    this->dv_dynamics1_msg.header.stamp = this->now();
    this->dv_dynamics2_msg.header.stamp = this->now();
    this->slam_stats_can_msg.header.stamp = this->now();

    this->dv_dynamics1_publisher->publish(this->dv_dynamics1_msg);
    this->dv_dynamics2_publisher->publish(this->dv_dynamics2_msg);
    this->slam_stats_can_publisher->publish(this->slam_stats_can_msg);
}

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<RaceDirector>());
        rclcpp::shutdown();

    return 0;
}