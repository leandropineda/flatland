#include <flatland_plugins/battery.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

using namespace flatland_server;

namespace flatland_plugins {

namespace {

std::string ToLower(const std::string &s) {
  std::string out(s);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return out;
}

// Returns the id token immediately after the case-insensitive prefix
// "Charger " in `name` (e.g. "Charger A (Office)" -> "A"), or
// std::nullopt if the name doesn't begin with that prefix.
std::optional<std::string> ExtractChargerId(const std::string &name) {
  static const std::string kPrefix = "charger ";
  if (name.size() <= kPrefix.size()) return std::nullopt;
  for (size_t i = 0; i < kPrefix.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(name[i])) != kPrefix[i]) {
      return std::nullopt;
    }
  }
  std::string id;
  for (size_t i = kPrefix.size(); i < name.size(); ++i) {
    char c = name[i];
    if (std::isspace(static_cast<unsigned char>(c)) || c == '(' || c == ')') {
      break;
    }
    id.push_back(c);
  }
  if (id.empty()) return std::nullopt;
  return id;
}

}  // namespace

void Battery::OnInitialize(const YAML::Node &config) {
  YamlReader reader(node_, config);

  std::string body_name = reader.Get<std::string>("body");
  std::string topic = reader.Get<std::string>("topic", "battery_state");
  double pub_rate = reader.Get<double>("pub_rate", 1.0);

  capacity_ah_ = reader.Get<double>("capacity_ah", 5.0);
  voltage_full_ = reader.Get<double>("voltage_full", 12.6);
  voltage_empty_ = reader.Get<double>("voltage_empty", 10.0);
  base_current_ = reader.Get<double>("base_current", 0.5);
  linear_current_coeff_ = reader.Get<double>("linear_current_coeff", 2.0);
  angular_current_coeff_ = reader.Get<double>("angular_current_coeff", 0.5);
  charge_current_ = reader.Get<double>("charge_current", 2.0);
  double initial_charge = reader.Get<double>("initial_charge", 1.0);

  // Parse charging zones: list of {x, y, radius, name}
  if (config["charging_zones"]) {
    for (const auto &zone_node : config["charging_zones"]) {
      YamlReader zone_reader(node_, zone_node);
      ChargingZone zone;
      zone.x = zone_reader.Get<double>("x");
      zone.y = zone_reader.Get<double>("y");
      zone.radius = zone_reader.Get<double>("radius", 0.5);
      zone.name = zone_reader.Get<std::string>("name", "charger");
      zone_reader.EnsureAccessedAllKeys();
      charging_zones_.push_back(zone);
    }
    // Mark key as accessed so EnsureAccessedAllKeys doesn't complain
    reader.accessed_keys_.insert("charging_zones");
  }

  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name \"" + body_name + "\" does not exist");
  }

  charge_ah_ = capacity_ah_ * initial_charge;
  depleted_ = false;
  charging_override_ = false;
  in_charging_zone_ = false;
  zones_published_ = false;

  update_timer_.SetRate(pub_rate);

  pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
      GetModel()->NameSpaceTopic(topic), 1);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      GetModel()->NameSpaceTopic("battery_marker"), 1);
  // Use transient local QoS so late-joining subscribers (rviz) receive the zone markers
  auto latched_qos = rclcpp::QoS(1).transient_local();
  zone_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      GetModel()->NameSpaceTopic("charging_zones"), latched_qos);
  // Same topic RViz's 2D Goal Pose tool uses; namespaced per model so each
  // robot's dock command reaches its own nav stack.
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      GetModel()->NameSpaceTopic("goal_pose"), 10);

  // Service: enable/disable charging manually
  set_charging_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      GetModel()->NameSpaceTopic("set_charging"),
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        charging_override_ = req->data;
        res->success = true;
        if (req->data) {
          res->message = "Charging enabled";
          RCLCPP_INFO(node_->get_logger(), "Battery: manual charging enabled");
        } else {
          res->message = "Charging disabled";
          RCLCPP_INFO(node_->get_logger(), "Battery: manual charging disabled");
        }
      });

  // Service: reset battery to full
  reset_battery_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      GetModel()->NameSpaceTopic("reset_battery"),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        charge_ah_ = capacity_ah_;
        depleted_ = false;
        charging_override_ = false;
        res->success = true;
        res->message = "Battery reset to 100%";
        RCLCPP_INFO(node_->get_logger(), "Battery: reset to full charge");
      });

  // Topic command interface: std_msgs/String with a case-insensitive verb.
  // Same semantics as the two services above, intended for UI integrations
  // (e.g. InOrbit) that send simple string commands. Namespaced per model so
  // one command channel exists per robot.
  command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      GetModel()->NameSpaceTopic("inorbit/custom_command"), 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = ToLower(msg->data);
        if (cmd == "charge") {
          charging_override_ = true;
          RCLCPP_INFO(node_->get_logger(),
                      "Battery: charging enabled via battery_command");
        } else if (cmd == "discharge") {
          charging_override_ = false;
          RCLCPP_INFO(node_->get_logger(),
                      "Battery: charging disabled via battery_command");
        } else if (cmd == "reset") {
          charge_ah_ = capacity_ah_;
          depleted_ = false;
          charging_override_ = false;
          RCLCPP_INFO(node_->get_logger(),
                      "Battery: reset to full charge via battery_command");
        } else if (cmd == "dock") {
          DispatchDock("");
        } else if (cmd.rfind("dock=", 0) == 0) {
          // Preserve caller casing in msg->data for warning output,
          // DispatchDock lower-cases internally for matching.
          DispatchDock(msg->data.substr(5));
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "Battery: unknown battery_command '%s' "
                      "(expected 'charge', 'discharge', 'reset', "
                      "'dock', or 'dock={id}')",
                      msg->data.c_str());
        }
      });

  RCLCPP_INFO(node_->get_logger(),
              "Battery plugin: capacity=%.1fAh voltage=%.1f-%.1fV "
              "base_current=%.2fA charge_current=%.2fA zones=%zu",
              capacity_ah_, voltage_empty_, voltage_full_,
              base_current_, charge_current_, charging_zones_.size());
  for (const auto &z : charging_zones_) {
    RCLCPP_INFO(node_->get_logger(),
                "  Charging zone '%s' at (%.1f, %.1f) radius=%.1fm",
                z.name.c_str(), z.x, z.y, z.radius);
  }
}

bool Battery::IsInChargingZone(double rx, double ry) const {
  for (const auto &zone : charging_zones_) {
    double dx = rx - zone.x;
    double dy = ry - zone.y;
    if (dx * dx + dy * dy <= zone.radius * zone.radius) {
      return true;
    }
  }
  return false;
}

bool Battery::DispatchDock(const std::string &id_or_empty) {
  if (charging_zones_.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Battery: dock requested but no charging_zones are configured");
    return false;
  }

  const ChargingZone *target = nullptr;

  if (id_or_empty.empty()) {
    // Closest zone to current robot position.
    b2Vec2 pos = body_->physics_body_->GetPosition();
    double best = std::numeric_limits<double>::infinity();
    for (const auto &z : charging_zones_) {
      double dx = z.x - pos.x;
      double dy = z.y - pos.y;
      double d2 = dx * dx + dy * dy;
      if (d2 < best) {
        best = d2;
        target = &z;
      }
    }
  } else {
    std::string wanted = ToLower(id_or_empty);
    for (const auto &z : charging_zones_) {
      auto extracted = ExtractChargerId(z.name);
      if (extracted && ToLower(*extracted) == wanted) {
        target = &z;
        break;
      }
    }
    if (target == nullptr) {
      std::ostringstream known;
      bool first = true;
      for (const auto &z : charging_zones_) {
        auto extracted = ExtractChargerId(z.name);
        if (!extracted) continue;
        if (!first) known << ", ";
        known << *extracted;
        first = false;
      }
      RCLCPP_WARN(node_->get_logger(),
                  "Battery: dock id '%s' not found (available: %s)",
                  id_or_empty.c_str(),
                  first ? "<none>" : known.str().c_str());
      return false;
    }
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = node_->now();
  goal.pose.position.x = target->x;
  goal.pose.position.y = target->y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  goal_pub_->publish(goal);

  RCLCPP_INFO(node_->get_logger(),
              "Battery: dispatching dock to '%s' at (%.2f, %.2f)",
              target->name.c_str(), target->x, target->y);
  return true;
}

void Battery::BeforePhysicsStep(const Timekeeper &timekeeper) {
  double dt = timekeeper.GetStepSize();
  b2Body *physics = body_->physics_body_;
  b2Vec2 pos = physics->GetPosition();

  // Check if robot is in a charging zone
  in_charging_zone_ = IsInChargingZone(pos.x, pos.y);
  bool is_charging = charging_override_ || in_charging_zone_;

  if (is_charging) {
    // Charge the battery
    charge_ah_ += charge_current_ * dt / 3600.0;
    if (charge_ah_ >= capacity_ah_) {
      charge_ah_ = capacity_ah_;
    }
    if (depleted_ && charge_ah_ > 0.0) {
      depleted_ = false;
      RCLCPP_INFO(node_->get_logger(), "Battery: charging resumed, robot can move again");
    }
  } else if (!depleted_) {
    // Drain the battery based on velocity
    b2Vec2 linear_vel = physics->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
    float angular_vel = physics->GetAngularVelocity();

    double speed = std::sqrt(linear_vel.x * linear_vel.x +
                             linear_vel.y * linear_vel.y);
    double current_draw = base_current_ +
                          linear_current_coeff_ * speed +
                          angular_current_coeff_ * std::fabs(angular_vel);

    charge_ah_ -= current_draw * dt / 3600.0;

    if (charge_ah_ <= 0.0) {
      charge_ah_ = 0.0;
      depleted_ = true;
      RCLCPP_WARN(node_->get_logger(), "Battery depleted! Robot stopped.");
    }
  }

  // Stop robot when depleted
  if (depleted_) {
    physics->SetLinearVelocity(b2Vec2(0, 0));
    physics->SetAngularVelocity(0);
  }

  // Publish at configured rate
  if (update_timer_.CheckUpdate(timekeeper)) {
    bool is_charging_now = charging_override_ || in_charging_zone_;
    double percentage = charge_ah_ / capacity_ah_;
    double voltage = voltage_empty_ +
                     (voltage_full_ - voltage_empty_) * percentage;

    // Compute instantaneous current
    double current;
    if (is_charging_now) {
      current = -charge_current_;  // positive = charging in BatteryState convention
    } else {
      current = base_current_;
      if (!depleted_) {
        b2Vec2 vel = physics->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));
        double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        current += linear_current_coeff_ * speed +
                   angular_current_coeff_ * std::fabs(physics->GetAngularVelocity());
      }
    }

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = timekeeper.GetSimTime();
    msg.header.frame_id = body_->name_;
    msg.voltage = static_cast<float>(voltage);
    msg.current = static_cast<float>(is_charging_now ? current : -current);
    msg.charge = static_cast<float>(charge_ah_);
    msg.capacity = static_cast<float>(capacity_ah_);
    msg.design_capacity = static_cast<float>(capacity_ah_);
    msg.percentage = static_cast<float>(percentage);
    if (depleted_) {
      msg.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else if (is_charging_now) {
      msg.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else {
      msg.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
    msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    pub_->publish(msg);

    // Publish text marker above robot
    char text_buf[128];
    const char *status_str = is_charging_now ? "CHG" : (depleted_ ? "DEAD" : "");
    std::snprintf(text_buf, sizeof(text_buf),
                  "Batt: %.0f%%  %.1fV  %.2fA %s",
                  percentage * 100.0, voltage,
                  is_charging_now ? current : -current,
                  status_str);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = timekeeper.GetSimTime();
    marker.header.frame_id = "map";
    marker.ns = "battery";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    marker.pose.position.z = 1.0;
    marker.scale.z = 0.3;
    if (depleted_) {
      marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    } else if (is_charging_now) {
      marker.color.r = 0.0; marker.color.g = 0.8; marker.color.b = 1.0;
    } else if (percentage < 0.2) {
      marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0;
    } else {
      marker.color.r = 0.2; marker.color.g = 1.0; marker.color.b = 0.2;
    }
    marker.color.a = 1.0;
    marker.text = text_buf;
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    marker_pub_->publish(marker);

    // Publish charging zone markers periodically (transient local QoS also latches for late joiners)
    if (!charging_zones_.empty()) {
      visualization_msgs::msg::MarkerArray zone_markers;
      for (size_t i = 0; i < charging_zones_.size(); ++i) {
        const auto &z = charging_zones_[i];

        // Circle marker for the zone
        visualization_msgs::msg::Marker circle;
        circle.header.stamp = timekeeper.GetSimTime();
        circle.header.frame_id = "map";
        circle.ns = "charging_zones";
        circle.id = static_cast<int>(i * 2);
        circle.type = visualization_msgs::msg::Marker::CYLINDER;
        circle.action = visualization_msgs::msg::Marker::ADD;
        circle.pose.position.x = z.x;
        circle.pose.position.y = z.y;
        circle.pose.position.z = 0.01;
        circle.scale.x = z.radius * 2.0;
        circle.scale.y = z.radius * 2.0;
        circle.scale.z = 0.02;
        circle.color.r = 0.0; circle.color.g = 0.6; circle.color.b = 1.0;
        circle.color.a = 0.4;
        circle.lifetime = rclcpp::Duration::from_seconds(0);  // forever
        zone_markers.markers.push_back(circle);

        // Label
        visualization_msgs::msg::Marker label;
        label.header.stamp = timekeeper.GetSimTime();
        label.header.frame_id = "map";
        label.ns = "charging_zone_labels";
        label.id = static_cast<int>(i * 2 + 1);
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = z.x;
        label.pose.position.y = z.y;
        label.pose.position.z = 0.5;
        label.scale.z = 0.25;
        label.color.r = 0.0; label.color.g = 0.8; label.color.b = 1.0;
        label.color.a = 1.0;
        label.text = z.name;
        label.lifetime = rclcpp::Duration::from_seconds(0);
        zone_markers.markers.push_back(label);
      }
      zone_marker_pub_->publish(zone_markers);
    }
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Battery,
                       flatland_server::ModelPlugin)
