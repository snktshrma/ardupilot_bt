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
    auto request_future = takeoff_client_->async_send_request(request, std::bind(&ROSControl::handleTakeoff, this, std::placeholders::_1));

    return true;
}

/**
 * @brief Service to arm the drone
 */
bool ROSControl::arm_control(bool arm)
{
    auto request = std::make_shared<ardupilot_msgs::srv::ArmMotors::Request>();
    request->arm = arm;

    // Call arm service
    auto request_future = arm_client_->async_send_request(request, std::bind(&ROSControl::handleArm, this, std::placeholders::_1));
    return true;
}

/**
 * @brief Service to switch the drone's mode
 */
bool ROSControl::mode_switch(uint8_t mode)
{
    auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
    request->mode = mode;

    // Call takeoff service
    auto request_future = switch_mode_client_->async_send_request(request, std::bind(&ROSControl::handleModeSwitch, this, std::placeholders::_1));

    return true;
}







void ROSControl::handleTakeoff(rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedFuture response)
{
    auto result = response.get();

    if (result->status) {
        RCLCPP_INFO(this->get_logger(), "Takeoff Success");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff Failed");
    }
}


void ROSControl::handleArm(rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedFuture response)
{
    auto result = response.get();

    if (result->status) {
        RCLCPP_INFO(this->get_logger(), "Takeoff Success");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff Failed");
    }
}


void ROSControl::handleModeSwitch(rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture response)
{
    result = response.get();

    if (result->status) {
        RCLCPP_INFO(this->get_logger(), "Mode switched successfully to: %u", result-> curr_mode);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Mode switch failed");
    }
}

