// component_container_isolated, but every component gets an EventsExecutor:
// push-driven event queue instead of waitset polling. Measured on this
// fleet's nav2 stacks the waitset wakeups on the 50 Hz /clock dominated
// idle CPU. Same CLI/contract as the stock containers.
#include <memory>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager_isolated.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  using Executor = rclcpp::experimental::executors::EventsExecutor;
  auto exec = std::make_shared<Executor>();
  auto node =
    std::make_shared<rclcpp_components::ComponentManagerIsolated<Executor>>(exec);
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}
