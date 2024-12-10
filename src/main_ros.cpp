#include "ardupilot_bt/nodes.hpp"

using namespace BT;
using namespace std;

int main()
{
  BehaviorTreeFactory factory;

  factory.registerNodeType<changeMode>("changeMode");
  factory.registerNodeType<arm>("arm");
  factory.registerNodeType<takeoff>("takeoff");
  factory.registerNodeType<setMode>("setMode");
  factory.registerNodeType<setAlt>("setAlt");

  factory.registerSimpleCondition("isModeChanged", std::bind(isModeChanged));
  factory.registerSimpleCondition("checkPrearm", std::bind(checkPrearm));

  // auto tree = factory.createTreeFromText(xml);
  auto tree = factory.createTreeFromFile("/home/snkt/bt_ws/src/ardupilot_bt/src/tree.xml");

  std::chrono::milliseconds ms = (std::chrono::milliseconds) 100;
  // BT::Tree::TickOption ops = BT::Tree::TickOption::EXACTLY_ONCE;
  tree.tickWhileRunning();

  return 0;
}
