/**
 * @file offb_node.cpp
 * @brief ROS control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
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
    takeoff_client = this->create_client<ardupilot_msgs::srv::Takeoff>("ardupilot_msgs/srv/Takeoff")
}
/**
 * @brief Service to takeoff the drone
 */
void ROSControl::takeoff(float32 altitude)
{
    auto request = std::make_shared<ardupilot_msgs::srv:::Takeoff::Request>();
    request->alt = altitude;

    // Call takeoff service
    auto request_future = takeoff_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

    if (result == rclcpp::FutureReturnCode::Success) {
        auto response = result_future.get();
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROSControl>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
