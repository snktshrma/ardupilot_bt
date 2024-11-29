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

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("ardupilot_bt package\n");
  return 0;
}
