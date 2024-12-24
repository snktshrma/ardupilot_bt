#ifndef APROS_H
#define APROS_H

// Standard Libraries
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS2 Collaborative Drones Libraries
#include "ardupilot_msgs/srv/takeoff.hpp"

// Namespaces
using namespace BT;
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

class ROSControl : public rclcpp::Node {
public:
    ROSControl();

private:
    // Publishers

    // Subscribers

    // Service Clients
    rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedPtr takeoff_client_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr callback_group_1;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Input Trajectory
    // Method Declarations

    void arm();

    void takeoff();

};

#endif /* APROS_H */
