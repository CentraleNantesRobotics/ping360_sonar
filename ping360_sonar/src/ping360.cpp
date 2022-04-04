#include <ping360_sonar/ping360_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node{std::make_shared<ping360_sonar::Ping360Sonar>()};
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while(rclcpp::ok())
  {
    node->refresh();
    exec.spin_once();
  }

  rclcpp::shutdown();
  return 0;
}
