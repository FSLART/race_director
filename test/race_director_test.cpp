// Calls the node's callbacks directly; nothing is published on /res, /acu or /imu.
// CMake runs this on an isolated, localhost-only domain because the node's state thread publishes /state.
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "race_director/race_director.hpp"

using lart_msgs::msg::State;

namespace {

void send_acu(RaceDirector &rd, int asms, int as_state) {
    auto msg = std::make_shared<lart_msgs::msg::Acu>();
    msg->asms = asms;
    msg->ign = 0; // ign 1 would try to start bag recording
    msg->as_state = as_state;
    rd.acu_callback(msg);
}

void send_res(RaceDirector &rd, int signal) {
    auto msg = std::make_shared<lart_msgs::msg::Res>();
    msg->signal = signal;
    rd.res_callback(msg);
}

void send_aqt1(RaceDirector &rd, int res) {
    auto msg = std::make_shared<lart_msgs::msg::Aqt1>();
    msg->res = res;
    rd.aqt1_callback(msg);
}

// READY, RES go before 6 s is ignored, RES go after 6 s goes to DRIVING
void drive(RaceDirector &rd) {
    send_acu(rd, 1, State::READY);
    ASSERT_EQ(rd.get_current_state(), State::READY);
    send_res(rd, 5);
    ASSERT_EQ(rd.get_current_state(), State::READY);
    std::this_thread::sleep_for(std::chrono::milliseconds(6200));
    send_res(rd, 5);
    ASSERT_EQ(rd.get_current_state(), State::DRIVING);
}

}  // namespace

TEST(RaceDirectorTest, StartsOff) {
    auto rd = std::make_shared<RaceDirector>();
    EXPECT_EQ(rd->get_current_state(), State::OFF);
}

TEST(RaceDirectorTest, ReadyResGoAfter6sDrivingThenResStopEmergency) {
    auto rd = std::make_shared<RaceDirector>();
    drive(*rd);
    // less than 1 s after DRIVING: the stop must not be held back by the 1 s guard
    send_res(*rd, 0);
    EXPECT_EQ(rd->get_current_state(), State::EMERGENCY);
    // latched
    send_acu(*rd, 1, State::READY);
    send_res(*rd, 5);
    EXPECT_EQ(rd->get_current_state(), State::EMERGENCY);
}

TEST(RaceDirectorTest, ResStopInAnyState) {
    auto rd = std::make_shared<RaceDirector>();
    send_res(*rd, 0);
    EXPECT_EQ(rd->get_current_state(), State::EMERGENCY);
}

TEST(RaceDirectorTest, Aqt1ResStopOnlyWhileDriving) {
    auto rd = std::make_shared<RaceDirector>();
    send_acu(*rd, 1, State::READY);
    send_aqt1(*rd, 0);
    EXPECT_EQ(rd->get_current_state(), State::READY);
    send_aqt1(*rd, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(6200));
    send_res(*rd, 5);
    ASSERT_EQ(rd->get_current_state(), State::DRIVING);
    send_aqt1(*rd, 0);
    EXPECT_EQ(rd->get_current_state(), State::EMERGENCY);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
