#include <cstdio>
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
using namespace std;



class changeMode : public BT::SyncActionNode {
  public:
    changeMode(const string& name) : BT::SyncActionNode(name, {}) {}
    NodeStatus tick() override {
      cout << "Changing Mode" << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus checkPrearm() {
  std::cout << "[ Checking Status ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class arm : public BT::SyncActionNode {
  // Add the sequence to confirm if arming is success, else a rate controller to
  // arming command until armed
  public:
    arm(const string& name) : BT::SyncActionNode(name, {}) {}
    NodeStatus tick() override {
      cout << "Arming the vehicle" << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

class takeoff : public BT::SyncActionNode {
  // TODO: Add altitude input for user
  // Add the sequence and fallbacks to confirm if takeoff is success

  public:
    takeoff(const string& name) : BT::SyncActionNode(name, {}) {}
    NodeStatus tick() override {
      cout << "Taking off the vehicle" << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

class vehicleMode : public BT::SyncActionNode {
  // Changes vehicle modes (like auto, guided, RTL, LAND, etc.)

  public:
    vehicleMode(const string& name) : BT::SyncActionNode(name, {}) {}
    NodeStatus tick() override {
      cout << "Vehicle's mode changes" << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

static const char* xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="main">
    <Sequence name="root_sequence">
      <changeMode name="mode_change" modeNum="1"/>
      <RetryUntilSuccessful num_attempts="10">
        <checkPrearm name="prearm_check"/>
      </RetryUntilSuccessful>
      <arm name="arm"/>
      <Takeoff name="takeoff"/>
    </Sequence>
  </BehaviorTree>

</root>

)";

int main()
{
  BehaviorTreeFactory factory;

  factory.registerNodeType<changeMode>("changeMode");
  factory.registerNodeType<arm>("arm");
  factory.registerNodeType<takeoff>("takeoff");

  factory.registerSimpleCondition("checkPrearm", std::bind(checkPrearm));

  auto tree = factory.createTreeFromText(xml);

  std::chrono::milliseconds ms = (std::chrono::milliseconds) 100;
  // BT::Tree::TickOption ops = BT::Tree::TickOption::EXACTLY_ONCE;
  tree.tickWhileRunning();

  return 0;
}
