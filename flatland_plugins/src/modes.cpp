#include <flatland_plugins/modes.h>
#include <flatland_server/yaml_reader.h>

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

using namespace flatland_server;

namespace flatland_plugins
{

void Modes::OnInitialize(const YAML::Node & config)
{
  YamlReader reader(node_, config);
  allowed_ = reader.GetList<std::string>("modes", {}, -1, -1);
  mode_ = reader.Get<std::string>("initial_mode", "idle");
  std::string topic = reader.Get<std::string>("topic", "mode");
  std::string cmd_topic = reader.Get<std::string>("command_topic", "mode_cmd");
  std::string custom_command_topic =
    reader.Get<std::string>("custom_command_topic", "custom_command");
  reader.EnsureAccessedAllKeys();

  // latched so late joiners (dashboards, agents) see the current mode
  mode_pub_ = node_->create_publisher<std_msgs::msg::String>(
    GetModel()->NameSpaceTopic(topic), rclcpp::QoS(1).transient_local());

  cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
    GetModel()->NameSpaceTopic(cmd_topic), 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { SetMode(msg->data, "mode_cmd"); });

  // "mode=<name>" on the same command channel the Battery plugin listens to
  custom_command_sub_ = node_->create_subscription<std_msgs::msg::String>(
    GetModel()->NameSpaceTopic(custom_command_topic), 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data.rfind("mode=", 0) == 0) {
        SetMode(msg->data.substr(5), "custom_command");
      }
    });

  SetMode(mode_, "initial");
}

void Modes::SetMode(const std::string & mode, const char * source)
{
  if (!allowed_.empty() && std::find(allowed_.begin(), allowed_.end(), mode) == allowed_.end()) {
    RCLCPP_WARN(
      rclcpp::get_logger("Modes"), "%s: rejecting unknown mode '%s' (from %s)",
      GetModel()->GetName().c_str(), mode.c_str(), source);
    return;
  }
  mode_ = mode;
  std_msgs::msg::String msg;
  msg.data = mode_;
  mode_pub_->publish(msg);
  RCLCPP_INFO(
    rclcpp::get_logger("Modes"), "%s: mode -> '%s' (from %s)", GetModel()->GetName().c_str(),
    mode_.c_str(), source);
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Modes, flatland_server::ModelPlugin)
