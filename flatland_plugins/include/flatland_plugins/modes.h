#ifndef FLATLAND_PLUGINS_MODES_H
#define FLATLAND_PLUGINS_MODES_H

#include <flatland_server/model_plugin.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace flatland_plugins
{

/**
 * Fake operating-mode state machine (e.g. idle / cleaning / transporting).
 * Publishes the current mode latched on `mode` and switches on `mode_cmd`
 * or on an `inorbit/custom_command` payload of the form "mode=<name>".
 * It only pretends: no behavior changes, just observable state.
 */
class Modes : public flatland_server::ModelPlugin
{
public:
  void OnInitialize(const YAML::Node & config) override;

private:
  void SetMode(const std::string & mode, const char * source);

  std::vector<std::string> allowed_;  ///< empty = any mode accepted
  std::string mode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr custom_command_sub_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_MODES_H
