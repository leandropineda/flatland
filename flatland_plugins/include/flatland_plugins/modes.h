#ifndef FLATLAND_PLUGINS_MODES_H
#define FLATLAND_PLUGINS_MODES_H

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace flatland_plugins
{

/**
 * Operating-mode state, published latched on `mode`.
 *
 * Automatic (default): derived from what the robot is actually doing —
 * `charging` (BatteryState says so) > `navigating` (body is moving) >
 * `idle`. A `mode_cmd` (or "mode=<name>" on the command channel) sets a
 * manual override (e.g. cleaning, transporting, mission) that sticks until
 * `mode_cmd: auto`. It only pretends: no behavior changes.
 */
class Modes : public flatland_server::ModelPlugin
{
public:
  void OnInitialize(const YAML::Node & config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper) override;

private:
  void HandleCommand(const std::string & mode, const char * source);
  void SetMode(const std::string & mode, const char * source);

  std::vector<std::string> allowed_;  ///< empty = any manual mode accepted
  std::string mode_;
  std::string manual_mode_;   ///< non-empty = override active
  double speed_threshold_;    ///< m/s above which the robot is `navigating`
  bool charging_ = false;     ///< from the model's BatteryState topic
  flatland_server::Body * body_ = nullptr;
  UpdateTimer update_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr custom_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_MODES_H
