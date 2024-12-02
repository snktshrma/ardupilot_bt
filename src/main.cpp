#include <cstdio>
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
using namespace std;



class checkPrearms : public BT::SyncActionNode {
  public:
    checkPrearms(const string& name) : BT::SyncActionNode(name, {}) {}
    NodeStatus tick() override {
      cout << "Checking Prearms" << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

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
      cout << "Vehcile's mode changes" << this->name() << endl;
      return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("ardupilot_bt package\n");
  return 0;
}
