// component_container_isolated, but every component gets an EventsExecutor
// where the distro has one (Iron+): push-driven event queue instead of
// waitset polling. Measured on this fleet's nav2 stacks the waitset wakeups
// on the 50 Hz /clock dominated idle CPU. On Humble (no EventsExecutor) it
// degrades to the stock SingleThreadedExecutor. Same CLI/contract as the
// stock containers.
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager_isolated.hpp>

#if __has_include(<rclcpp/experimental/executors/events_executor/events_executor.hpp>)
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
using BaseExecutor = rclcpp::experimental::executors::EventsExecutor;
#else
using BaseExecutor = rclcpp::executors::SingleThreadedExecutor;
#endif

// ComponentManagerIsolated constructs the per-component executor differently
// across distros: default-constructed (jazzy/kilted) or with
// (ExecutorOptions, thread_count) (lyrical+). Accept both; the options only
// matter for non-default contexts, which the container does not use.
class FleetExecutor : public BaseExecutor
{
public:
  FleetExecutor() = default;
  explicit FleetExecutor(const rclcpp::ExecutorOptions & /*options*/, size_t /*threads*/ = 0)
  : BaseExecutor() {}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<FleetExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManagerIsolated<FleetExecutor>>(exec);
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}
