/**
 * @file ap_ros.cpp
 * @brief AP ROS control
 */

// Standard Libraries
#include <stdint.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// MAVROS Libraries
// Custom Libraries
#include "ardupilot_bt/ap_ros.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;


ROSControl::ROSControl() : rclcpp::Node("ros_control")
{
    takeoff_client_ = this->create_client<ardupilot_msgs::srv::Takeoff>("/ap/experimental/takeoff");
    arm_client_ = this->create_client<ardupilot_msgs::srv::ArmMotors>("/ap/arm_motors");
    switch_mode_client_ = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");
}
/**
 * @brief Service to takeoff the drone
 */
bool ROSControl::takeoff_control(float altitude)
{
    auto request = std::make_shared<ardupilot_msgs::srv::Takeoff::Request>();
    request->alt = altitude;

    // Call takeoff service
    auto request_future = takeoff_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), request_future);

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = request_future.get();
        if (response->status) {
            RCLCPP_INFO(this->get_logger(), "Takeoff Success");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Takeoff Failed");
        }

    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call takeoff service");
    }

    return false;

}
/**
 * @brief Service to arm the drone
 */
bool ROSControl::arm_control()
{
    return false;
}

/**
 * @brief Service to switch the drone's mode
 */
bool ROSControl::mode_switch()
{
    return false;
}
