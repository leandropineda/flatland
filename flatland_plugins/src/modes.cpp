#include <flatland_plugins/modes.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

using namespace flatland_server;

namespace flatland_plugins
{

void Modes::OnInitialize(const YAML::Node & config)
{
  YamlReader reader(node_, config);
  allowed_ = reader.GetList<std::string>("modes", {}, -1, -1);
  manual_mode_ = reader.Get<std::string>("initial_mode", "");
  std::string body_name = reader.Get<std::string>("body", "");
  speed_threshold_ = reader.Get<double>("speed_threshold", 0.05);
  double update_rate = reader.Get<double>("update_rate", 2.0);
  std::string topic = reader.Get<std::string>("topic", "mode");
  std::string cmd_topic = reader.Get<std::string>("command_topic", "mode_cmd");
  std::string custom_command_topic =
    reader.Get<std::string>("custom_command_topic", "custom_command");
  std::string battery_topic = reader.Get<std::string>("battery_topic", "battery_state");
  reader.EnsureAccessedAllKeys();

  if (!body_name.empty()) {
    body_ = GetModel()->GetBody(body_name);
    if (body_ == nullptr) {
      throw YAMLException("Modes: body with name \"" + body_name + "\" does not exist");
    }
  }
  update_timer_.SetRate(update_rate);

  // latched so late joiners (dashboards, agents) see the current mode
  mode_pub_ = node_->create_publisher<std_msgs::msg::String>(
    GetModel()->NameSpaceTopic(topic), rclcpp::QoS(1).transient_local());

  cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
    GetModel()->NameSpaceTopic(cmd_topic), 10,
    [this](const std_msgs::msg::String::SharedPtr msg) { HandleCommand(msg->data, "mode_cmd"); });

  // "mode=<name>" on the same command channel the Battery plugin listens to
  custom_command_sub_ = node_->create_subscription<std_msgs::msg::String>(
    GetModel()->NameSpaceTopic(custom_command_topic), 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data.rfind("mode=", 0) == 0) {
        HandleCommand(msg->data.substr(5), "custom_command");
      }
    });

  // charging state comes from the model's own BatteryState topic, so Modes
  // stays decoupled from the Battery plugin (and works without it)
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    GetModel()->NameSpaceTopic(battery_topic), 10,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      charging_ =
        msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    });

  SetMode(manual_mode_.empty() ? "idle" : manual_mode_, "initial");
}

void Modes::HandleCommand(const std::string & mode, const char * source)
{
  if (mode == "auto" || mode.empty()) {
    manual_mode_.clear();
    RCLCPP_INFO(
      rclcpp::get_logger("Modes"), "%s: automatic mode (from %s)",
      GetModel()->GetName().c_str(), source);
    return;  // next update publishes the derived mode
  }
  if (!allowed_.empty() && std::find(allowed_.begin(), allowed_.end(), mode) == allowed_.end()) {
    RCLCPP_WARN(
      rclcpp::get_logger("Modes"), "%s: rejecting unknown mode '%s' (from %s)",
      GetModel()->GetName().c_str(), mode.c_str(), source);
    return;
  }
  manual_mode_ = mode;
  SetMode(mode, source);
}

void Modes::BeforePhysicsStep(const Timekeeper & timekeeper)
{
  if (!update_timer_.CheckUpdate(timekeeper)) return;
  if (!manual_mode_.empty()) return;  // manual override active

  std::string mode = "idle";
  if (charging_) {
    mode = "charging";
  } else if (body_ != nullptr) {
    b2Vec2 v = body_->physics_body_->GetLinearVelocity();
    double speed = std::sqrt(v.x * v.x + v.y * v.y);
    if (speed > speed_threshold_ ||
        std::fabs(body_->physics_body_->GetAngularVelocity()) > 4 * speed_threshold_) {
      mode = "navigating";
    }
  }
  if (mode != mode_) SetMode(mode, "auto");
}

void Modes::SetMode(const std::string & mode, const char * source)
{
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
