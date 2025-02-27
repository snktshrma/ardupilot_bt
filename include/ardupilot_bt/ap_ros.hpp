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
#include "ardupilot_msgs/srv/mode_switch.hpp"
#include "ardupilot_msgs/srv/arm_motors.hpp"

// Namespaces
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

class ROSControl : public rclcpp::Node {
public:
    ROSControl();

    bool takeoff_control(float altitude);
    bool arm_control(bool arm);
    bool mode_switch(uint8_t mode);

private:
    void handleTakeoff(rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedFuture response);
    void handleModeSwitch(rclcpp::Client<ardupilot_msgs::src::ModeSwitch>::SharedFuture response);
    void handleArm(rclcpp::Client<ardupilot_msgs::src::ArmMotor>::SharedFuture response);

    // Service Clients
    rclcpp::Client<ardupilot_msgs::srv::Takeoff>::SharedPtr takeoff_client_;
    rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr switch_mode_client_;
    rclcpp::Client<ardupilot_msgs::srv::ArmMotors>::SharedPtr arm_client_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr callback_group_1;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;



};

#endif /* APROS_H */
