#include "race_director/race_director.hpp"

RaceDirector::RaceDirector() : Node("race_director"){

    bool is_unit_test = false;
    if (const char *env = std::getenv("UNIT_TEST")) {
      is_unit_test = (std::string(env) == "1");
    }

    /* Publishers*/
    this->state_publisher = this->create_publisher<lart_msgs::msg::State>(TOPIC_STATE_PC, 10);
    this->jetson_publisher = this->create_publisher<lart_msgs::msg::Jetson>(TOPIC_JETSON, 10);
    this->slam_stats_can_publisher = this->create_publisher<lart_msgs::msg::SlamStatsCan>(TOPIC_DV_SLAM_STATS, 10);
    this->dv_dynamics1_publisher = this->create_publisher<lart_msgs::msg::DvDynamics1>(TOPIC_DV_DYNAMICS1, 10);
    this->dv_dynamics2_publisher = this->create_publisher<lart_msgs::msg::DvDynamics2>(TOPIC_DV_DYNAMICS2, 10);
    this->cubemars_position_loop_publisher = this->create_publisher<lart_msgs::msg::CubemarsPositionLoop>(TOPIC_CUBEMARS_POSITION_LOOP, 10);
    this->vcu_torque_target_publisher = this->create_publisher<lart_msgs::msg::VcuTorqueTarget>(TOPIC_VCU_TORQUE_TARGET, 10);
    this->vcu_rpm_target_publisher = this->create_publisher<lart_msgs::msg::VcuRpmTarget>(TOPIC_VCU_RPM_TARGET, 10);
    this->control_feedback_publisher = this->create_publisher<lart_msgs::msg::Dynamics>(TOPIC_CONTROL_FEEDBACK, 10);

    /* Subscribers */
    this->acu_subscriber = this->create_subscription<lart_msgs::msg::Acu>(TOPIC_CAN_ACU, 10, std::bind(&RaceDirector::acu_callback, this, _1));
    this->vcu_rpm_subscriber = this->create_subscription<lart_msgs::msg::VcuRpm>(TOPIC_CAN_VCU_RPM, 10, std::bind(&RaceDirector::vcu_control_feedback_callback, this, _1));
    this->res_subscriber = this->create_subscription<lart_msgs::msg::Res>(TOPIC_CAN_RES, 10, std::bind(&RaceDirector::res_callback, this, _1));
    this->nodes_state_subscriber = this->create_subscription<lart_msgs::msg::State>(TOPIC_STATE_NODES, 10, std::bind(&RaceDirector::nodes_state_callback, this, _1));
    this->mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(TOPIC_MISSION_PC, 10, std::bind(&RaceDirector::mission_callback, this, _1));
    imu_acc_sub_.subscribe(this, TOPIC_IMU_ACCELERATION);
    imu_gyro_sub_.subscribe(this, TOPIC_IMU_ANGULAR_VELOCITY);

    this->control_torque_subscriber = this->create_subscription<lart_msgs::msg::DynamicsCMD>(TOPIC_CONTROL_TORQUE_TARGET, 10, std::bind(&RaceDirector::torque_control_callback, this, _1));
    this->control_rpm_subscriber = this->create_subscription<lart_msgs::msg::DynamicsCMD>(TOPIC_CONTROL_RPM_TARGET, 10, std::bind(&RaceDirector::rpm_control_callback, this, _1));

    //sync imu acc and gyro subs
    imu_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::msg::Vector3Stamped, geometry_msgs::msg::Vector3Stamped>>(imu_acc_sub_, imu_gyro_sub_, 10);
    imu_sync_->registerCallback(std::bind(&RaceDirector::combined_imu_callback, this, _1, _2));

    this->cubemars_feedback_subscriber = this->create_subscription<lart_msgs::msg::CubemarsFeedback>(TOPIC_CAN_CUBEMARS_FEEDBACK, 10, std::bind(&RaceDirector::cubemars_feedback_callback, this, _1));
    
    /* Services */
    // if (!is_unit_test){
    //     this->perception_timestamp = this->create_client<lart_msgs::srv::Heartbeat>(SERVICE_ZED_TIMESTAMP);
        
    //     // while (!this->perception_timestamp->wait_for_service(std::chrono::seconds(1))) {
    //     //     if (!rclcpp::ok()) {
    //     //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for zed/last_timestamp service.");
    //     //         return;
    //     //     }
    //     //     RCLCPP_INFO(this->get_logger(), "Waiting for zed/last_timestamp service...");
    //     // }
    //     this->perception_timestamp_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&RaceDirector::request_perception_timestamp, this));
    // }

    this->start_bag_recording_client = this->create_client<rosbag2_interfaces::srv::Resume>(SERVICE_START_BAG_RECORDING);
    this->stop_bag_recording_client = this->create_client<rosbag2_interfaces::srv::Stop>(SERVICE_STOP_BAG_RECORDING);

    this->handbook_msgs_timer = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&RaceDirector::send_handbook_msgs, this));
    // this->steering_timestamp_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RaceDirector::check_steering_timestamp, this));
    
    /* Threads */
    this->state_thread = std::thread([this]() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            this->send_state_to_nodes();
            rate.sleep();
        }
    });
    this->state_thread.detach();

    this->send_jetson_msg_timer = this->create_wall_timer(std::chrono::duration<double>(0.02), [this]() { this->jetson_publisher->publish(this->jetson_msg); });

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
            auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();
            RCLCPP_INFO(this->get_logger(), "Calling start_bag_recording service");
            this->start_bag_recording_client->async_send_request(request);
            this->bag_recording = true;
            this->bag_stop_timer.reset(); // allow a fresh delayed-stop to be scheduled for this session
        }
    }else{
        if (this->bag_recording){
            this->schedule_bag_stop();
        }
    }

    auto received_state = msg->as_state;

    if (received_state == this->get_current_state()) {
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
            this->emergency_cause =lart_msgs::msg::Jetson::EMERGENCY_CAUSE_ACU;
            break;
    }

}

void RaceDirector::vcu_control_feedback_callback(const lart_msgs::msg::VcuRpm::SharedPtr msg){
    lart_msgs::msg::Dynamics control_feedback_msg;
    control_feedback_msg.header.stamp = this->now();
    control_feedback_msg.steering_angle = dv_dynamics1_msg.steering_angle_actual;
    control_feedback_msg.rpm = (msg->motor_rpm_left + msg->motor_rpm_right) / 2.0;

    this->dv_dynamics1_msg.speed_actual = RPM_TO_MS((msg->motor_rpm_left + msg->motor_rpm_right) / 2.0);
    this->dv_dynamics1_msg.motor_moment_actual = (msg->motor_current_left + msg->motor_current_right) / 2.0;//meter em percentagem

    this->control_feedback_publisher->publish(control_feedback_msg);
}


void RaceDirector::res_callback(const lart_msgs::msg::Res::SharedPtr msg){
    int res_signal = msg->signal;
    if(this->ready_change_set){
        if (this->get_current_state() == lart_msgs::msg::State::READY){
            std::chrono::duration<double> time_in_ready = std::chrono::steady_clock::now() - this->ready_change;
            if((res_signal == 5 || res_signal == 7) && time_in_ready > std::chrono::seconds(6)) {
                this->change_state(lart_msgs::msg::State::DRIVING);
            }
        }
    }

    if (res_signal == 0){
        this->change_state(lart_msgs::msg::State::EMERGENCY);
        this->emergency_cause =lart_msgs::msg::Jetson::EMERGENCY_CAUSE_RES;
    }
}

void RaceDirector::nodes_state_callback(const lart_msgs::msg::State::SharedPtr msg){
    int current_state_var = this->get_current_state();
    if (current_state_var == lart_msgs::msg::State::EMERGENCY) return; // if the current state is emergency it is not possible to change
    
    auto received_state = msg->data;
    switch (received_state){
        case lart_msgs::msg::State::FINISH:
            if (current_state_var == lart_msgs::msg::State::DRIVING)
                this->change_state(lart_msgs::msg::State::FINISH);
        break;
            case lart_msgs::msg::State::EMERGENCY:
            this->change_state(lart_msgs::msg::State::EMERGENCY);
            //TODO: Figure out a way to know which node caused the emergency
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
    this->jetson_msg.header.stamp = this->now();
    this->jetson_msg.as_mission = msg->data;
    this->jetson_msg.as_state = this->get_current_state();
    this->jetson_msg.temperature = static_cast<int8_t>(final_temp);
    this->jetson_msg.cpu = 0;
    this->jetson_msg.gpu = 0;
    this->jetson_msg.emergency_cause = this->emergency_cause;

    // this->jetson_publisher->publish(this->jetson_msg);
}

void RaceDirector::slam_callback(const lart_msgs::msg::SlamStats::SharedPtr msg){
    this->slam_stats_can_msg.lap_counter = msg->lap_count;
    this->slam_stats_can_msg.cones_count_actual = msg->cones_count_current;
    this->slam_stats_can_msg.cones_count_all = msg->cones_count_all;
}

void RaceDirector::combined_imu_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& acc_msg, const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& gyro_msg){
    this->dv_dynamics2_msg.acceleration_longitudinal = acc_msg->vector.x;
    this->dv_dynamics2_msg.acceleration_lateral= acc_msg->vector.y;
    this->dv_dynamics2_msg.yaw_rate = gyro_msg->vector.z;
}

void RaceDirector::torque_control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){
    float angle = msg->steering_angle;
    float sw_angle = -49.3021*std::pow(angle,3)+90.5065*std::pow(angle,2)+312.5504*angle;
    float rel_current = msg->acc_cmd*100;

    if (abs(sw_angle) > MAX_STEERING_ANGLE_DEG || abs(rel_current) > 100){
        RCLCPP_ERROR(this->get_logger(), "Commands out of bounds- steering:%f, rel_current:%f", sw_angle, rel_current);
        return;
    }
    
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
    cubemars_msg.position = sw_angle;
    this->cubemars_position_loop_publisher->publish(cubemars_msg);

}

void RaceDirector::rpm_control_callback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg){
    float angle = msg->steering_angle;
    float sw_angle = -49.3021*std::pow(angle,3)+90.5065*std::pow(angle,2)+312.5504*angle;
    float rpm = msg->rpm;

    if (abs(sw_angle) > MAX_STEERING_ANGLE_DEG || rpm < 0 || rpm > 20000){
        RCLCPP_ERROR(this->get_logger(), "Commands out of bounds- steering:%f, rpm:%f", sw_angle, rpm);
        return;
    }
    
    this->dv_dynamics1_msg.steering_angle_target = sw_angle;
    this->dv_dynamics1_msg.steering_angle_actual = 0.0;
    this->dv_dynamics1_msg.speed_target = RPM_TO_MS(rpm);
    this->dv_dynamics1_msg.speed_actual = 0.0;
    this->dv_dynamics1_msg.brake_hydr_target = 0.0;
    this->dv_dynamics1_msg.brake_hydr_actual = 0.0;
    this->dv_dynamics1_msg.motor_moment_target = 0.0;
    this->dv_dynamics1_msg.motor_moment_actual = 0.0;

    //prepare control commands for vcu and cubemars
    lart_msgs::msg::VcuRpmTarget vcu_msg;
    vcu_msg.header.stamp = this->now();
    vcu_msg.rpm_target = rpm;
    this->vcu_rpm_target_publisher->publish(vcu_msg);

    lart_msgs::msg::CubemarsPositionLoop cubemars_msg;
    cubemars_msg.header.stamp = this->now();
    cubemars_msg.position = sw_angle;
    this->cubemars_position_loop_publisher->publish(cubemars_msg);
}

void RaceDirector::cubemars_feedback_callback(const lart_msgs::msg::CubemarsFeedback::SharedPtr msg){
    this->dv_dynamics1_msg.steering_angle_actual = msg->position;
    if (msg->error_code != 0){
        RCLCPP_ERROR(this->get_logger(), "Cubemars error code: %d", msg->error_code);
        this->change_state(lart_msgs::msg::State::EMERGENCY);
        this->emergency_cause =lart_msgs::msg::Jetson::EMERGENCY_CAUSE_STEERING_ERROR;
    }
    this->last_steering_timestamp = std::chrono::steady_clock::now();
}

void RaceDirector::check_steering_timestamp(){
    if(!this->asms_state || this->last_steering_timestamp.time_since_epoch().count() == 0)
        return;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_since_last_steering = now - this->last_steering_timestamp;
    if (time_since_last_steering.count() > TIMESTAMP_MARGIN) {
        RCLCPP_ERROR(this->get_logger(), "Steering feedback timestamp is too old: %f seconds", time_since_last_steering.count());
        this->change_state(lart_msgs::msg::State::EMERGENCY);
        this->emergency_cause =lart_msgs::msg::Jetson::EMERGENCY_CAUSE_STEERING_TIMEOUT;
    }
}

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
            this->emergency_cause = lart_msgs::msg::Jetson::EMERGENCY_CAUSE_ZED;
        }
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}
#pragma endregion

int RaceDirector::get_current_state(){
    std::lock_guard<std::mutex> lock(state_mutex);
    return this->current_state;
}

void RaceDirector::change_state(int new_state) {
    std::lock_guard<std::mutex> lock(state_mutex);
    auto now = std::chrono::steady_clock::now();
    if(new_state == lart_msgs::msg::State::EMERGENCY){
        this->current_state = new_state;
        this->schedule_bag_stop();
    }

    std::chrono::duration<double> time_in_state = std::chrono::steady_clock::now() - this->last_state_change;
    if(time_in_state < std::chrono::seconds(1))
        return;
    if(this->current_state == lart_msgs::msg::State::EMERGENCY){
        return;
    }
    this->current_state = new_state;
    this->last_state_change = std::chrono::steady_clock::now();

    if(new_state == lart_msgs::msg::State::FINISH){
        this->schedule_bag_stop();
    }
}

void RaceDirector::schedule_bag_stop() {
    if (this->bag_stop_timer) {
        return; // already scheduled for this recording session
    }
    this->bag_stop_timer = this->create_wall_timer(std::chrono::seconds(5), [this]() {
        this->bag_stop_timer->cancel();
        this->stop_bag_recording();
    });
}

void RaceDirector::stop_bag_recording() {
    if (!this->bag_recording) {
        return;
    }
    auto request = std::make_shared<rosbag2_interfaces::srv::Stop::Request>();
    RCLCPP_INFO(this->get_logger(), "Calling stop_bag_recording service");
    this->stop_bag_recording_client->async_send_request(request);
    this->bag_recording = false;
}

void RaceDirector::send_state_to_nodes() {
    lart_msgs::msg::State msg;
    msg.header.stamp = this->now();
    msg.data = this->get_current_state();
    // RCLCPP_INFO(this->get_logger(), "Publishing state: %d", msg.data);

    this->state_publisher->publish(msg);
    // this->jetson_publisher->publish(this->jetson_msg);

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