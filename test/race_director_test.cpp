#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "race_director/race_director.hpp"


TEST(RaceDirectorTest, StartingState) {
    RaceDirector race_director = RaceDirector();
    EXPECT_EQ(race_director.get_current_state(), lart_msgs::msg::State::OFF);
}

TEST(RaceDirectorTest, StateTransitionToReady) {
    RaceDirector race_director = RaceDirector();
    auto msg = std::make_shared<lart_msgs::msg::State>();
    msg->data = lart_msgs::msg::State::READY;

    race_director.acu_state_callback(msg);

    EXPECT_EQ(race_director.get_current_state(), lart_msgs::msg::State::READY);
}

TEST(RaceDirectorTest, StateTransitionToDrivingBefore6Seconds) {
    RaceDirector race_director = RaceDirector();
    auto msg = std::make_shared<lart_msgs::msg::State>();
    msg->header.stamp = rclcpp::Clock().now();
    msg->data = lart_msgs::msg::State::READY;

    race_director.acu_state_callback(msg);
    msg->header.stamp = rclcpp::Clock().now();
    msg->data = lart_msgs::msg::State::DRIVING;

    race_director.acu_state_callback(msg);

    EXPECT_EQ(race_director.get_current_state(), lart_msgs::msg::State::READY);
}

TEST(RaceDirectorTest, StateTransitionToDrivingAfter6Seconds) {
    RaceDirector race_director = RaceDirector();
    auto msg = std::make_shared<lart_msgs::msg::State>();
    msg->header.stamp = rclcpp::Clock().now();
    msg->data = lart_msgs::msg::State::READY;

    race_director.acu_state_callback(msg);

    std::this_thread::sleep_for(std::chrono::seconds(7));

    msg->header.stamp = rclcpp::Clock().now();
    msg->data = lart_msgs::msg::State::DRIVING;

    race_director.acu_state_callback(msg);

    EXPECT_EQ(race_director.get_current_state(), lart_msgs::msg::State::DRIVING);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    setenv("UNIT_TEST", "1", 1);

    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}