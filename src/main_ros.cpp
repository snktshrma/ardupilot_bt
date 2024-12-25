#include "ardupilot_bt/nodes.hpp"
#include "ardupilot_bt/ap_ros.hpp"

using namespace BT;
using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto ros_control = std::make_shared<ROSControl>();

  BehaviorTreeFactory factory;

  factory.registerNodeType<changeMode>("changeMode");
  factory.registerNodeType<arm>("arm");
  factory.registerNodeType<takeoff>("takeoff", ros_control);
  factory.registerNodeType<setMode>("setMode");
  factory.registerNodeType<setAlt>("setAlt");

  factory.registerSimpleCondition("isModeChanged", std::bind(isModeChanged));
  factory.registerSimpleCondition("checkPrearm", std::bind(checkPrearm));

  // auto tree = factory.createTreeFromText(xml);
  auto tree = factory.createTreeFromFile("/home/snkt/bt_ws/src/ardupilot_bt/src/tree.xml");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_control);

  while (rclcpp::ok()) {
    executor.spin_some();
    tree.tickWhileRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  // BT::Tree::TickOption ops = BT::Tree::TickOption::EXACTLY_ONCE;

  return 0;
}
