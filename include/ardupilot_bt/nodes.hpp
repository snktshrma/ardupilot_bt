#ifndef NODE_H
#define NODE_H

#include <behaviortree_ros2/bt_action_node.hpp>
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>


#include "ardupilot_bt/ap_ros.hpp"


// #include "behaviortree_cpp/loggers/bt_file_logger.h"
// #include "ardupilot_bt/nodes.hpp"
#include <math.h>

using namespace BT;
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;


NodeStatus isModeChanged() {
    cout << "Checking if Mode is already set [isModeChanged: FAILURE]" << endl;
    return NodeStatus::FAILURE;
}

NodeStatus checkPrearm() {
    cout << "Prearm Passed [checkPrearm: SUCCESS]" << endl;
    return NodeStatus::SUCCESS;
}

class setMode : public BT::SyncActionNode {
    public:
        setMode(const string& name, const NodeConfig& config)
            : BT::SyncActionNode(name, config), mode_num(4) {}

        static PortsList providedPorts() {
            return {OutputPort<int>("modeNum") };
        }

        NodeStatus tick() override {
            // std::string s = std::to_string(mode_num);
            setOutput("modeNum", mode_num);
            cout << "Mode Set through output port: " << mode_num << endl;
            return NodeStatus::SUCCESS;
        }
    private:
        int mode_num;
};


class setAlt : public BT::SyncActionNode {
    public:
        setAlt(const string& name, const NodeConfig& config)
            : BT::SyncActionNode(name, config), alt_(10) {}

        static PortsList providedPorts() {
            return {OutputPort<float>("alt") };
        }

        NodeStatus tick() override {
            // std::string s = std::to_string(alt);
            setOutput("alt", alt_);
            cout << "Altitude Set through output port: " << alt_ << endl;
            return NodeStatus::SUCCESS;
        }
    private:
        float alt_;
};

class changeMode : public BT::SyncActionNode {
  public:
    changeMode(const string& name, const NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static PortsList providedPorts() {
        return { InputPort<int>("modeNum") };
    }

    NodeStatus tick() override {
        Expected<int> mode_num = getInput<int>("modeNum");

        if (!mode_num) {
            throw BT::RuntimeError("Mode not set: ", mode_num.error() );
        }
        cout << "Mode Changed: " << mode_num.value() << endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class arm : public BT::SyncActionNode {
  // Add the sequence to confirm if arming is success, else a rate controller to
  // arming command until armed
  public:
    arm(const string& name) : BT::SyncActionNode(name, {}) {}
    NodeStatus tick() override {
      cout << "Arming the vehicle: " << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

class takeoff : public BT::SyncActionNode {
  // TODO: Add altitude input for user
  // Add the sequence and fallbacks to confirm if takeoff is success

  public:
    takeoff(const string& name, const NodeConfig& config, std::shared_ptr<ROSControl> ros_control) : BT::SyncActionNode(name, config), ros_control_(ros_control) {}

    static PortsList providedPorts() {
        return { InputPort<float>("alt") };
    }

    NodeStatus tick() override {
        Expected<float> alt_ = getInput<float>("alt");
        float altitude = alt_.value();
        if (!alt_) {
            throw BT::RuntimeError("Alt to be defined: ", alt_.error() );
        }

        if (ros_control_->takeoff_control(altitude)) {
            cout << "Taking Off! Altitude: " << altitude << endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
        
    }
  private:
    std::shared_ptr<ROSControl> ros_control_;
};

#endif /* NODE_H */