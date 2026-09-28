/*
 * @name     imu.cpp
 * @brief    Simulated 2D IMU: orientation, yaw rate and linear acceleration of a point on a body
 */

#include <Box2D/Box2D.h>
#include <flatland_plugins/imu.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace flatland_plugins
{

void Imu::OnInitialize(const YAML::Node & config)
{
  YamlReader reader(node_, config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "imu");
  frame_id_ = reader.Get<std::string>("frame", GetName());
  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);
  update_rate_ = reader.Get<double>("update_rate", std::numeric_limits<double>::infinity());
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));
  orientation_noise_std_dev_ = reader.Get<double>("orientation_noise_std_dev", 0.0);
  angular_velocity_noise_std_dev_ = reader.Get<double>("angular_velocity_noise_std_dev", 0.0);
  linear_acceleration_noise_std_dev_ = reader.Get<double>("linear_acceleration_noise_std_dev", 0.0);
  // !(x >= 0) also rejects NaN, which would put NaN in the covariances
  if (!(orientation_noise_std_dev_ >= 0.0 && angular_velocity_noise_std_dev_ >= 0.0 &&
    linear_acceleration_noise_std_dev_ >= 0.0))
  {
    throw YAMLException("Imu noise std devs must be >= 0");
  }
  int seed = reader.Get<int>("seed", 0);
  reader.EnsureAccessedAllKeys();

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + Q(body_name));
  }
  rng_ = std::mt19937(seed != 0 ? static_cast<unsigned>(seed) : std::random_device{}());
  update_timer_.SetRate(update_rate_);
  publisher_ =
    node_->create_publisher<sensor_msgs::msg::Imu>(GetModel()->NameSpaceTopic(topic_), 1);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  imu_msg_.header.frame_id = GetModel()->NameSpaceTF(frame_id_);
  double var_o = orientation_noise_std_dev_ * orientation_noise_std_dev_;
  double var_w = angular_velocity_noise_std_dev_ * angular_velocity_noise_std_dev_;
  double var_a = linear_acceleration_noise_std_dev_ * linear_acceleration_noise_std_dev_;
  // 2D: roll and pitch are exactly 0
  imu_msg_.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, var_o};
  imu_msg_.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, var_w};
  imu_msg_.linear_acceleration_covariance = {var_a, 0, 0, 0, var_a, 0, 0, 0, var_a};

  imu_tf_.header.frame_id = GetModel()->NameSpaceTF(body_->GetName());
  imu_tf_.child_frame_id = imu_msg_.header.frame_id;
  imu_tf_.transform.translation.x = origin_.x;
  imu_tf_.transform.translation.y = origin_.y;
  imu_tf_.transform.rotation.z = sin(0.5 * origin_.theta);
  imu_tf_.transform.rotation.w = cos(0.5 * origin_.theta);
}

void Imu::AfterPhysicsStep(const Timekeeper & timekeeper)
{
  UpdateImu(timekeeper.GetStepSize());
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }
  if (publisher_->get_subscription_count() > 0) {
    imu_msg_.header.stamp = timekeeper.GetSimTime();
    publisher_->publish(imu_msg_);
  }
  if (broadcast_tf_) {
    imu_tf_.header.stamp = timekeeper.GetSimTime();
    tf_broadcaster_->sendTransform(imu_tf_);
  }
}

void Imu::UpdateImu(double dt)
{
  b2Body * b = body_->GetPhysicsBody();
  b2Vec2 velocity = b->GetLinearVelocityFromLocalPoint(b2Vec2(origin_.x, origin_.y));
  b2Vec2 acceleration = (1.0f / static_cast<float>(dt)) * (velocity - previous_velocity_);  // world
  previous_velocity_ = velocity;

  double yaw = b->GetAngle() + origin_.theta;  // of the IMU frame, in the world
  double c = cos(yaw), s = sin(yaw);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw + Noise(orientation_noise_std_dev_));
  imu_msg_.orientation = tf2::toMsg(q);
  imu_msg_.angular_velocity.z = b->GetAngularVelocity() + Noise(angular_velocity_noise_std_dev_);
  imu_msg_.linear_acceleration.x =
    c * acceleration.x + s * acceleration.y + Noise(linear_acceleration_noise_std_dev_);
  imu_msg_.linear_acceleration.y =
    -s * acceleration.x + c * acceleration.y + Noise(linear_acceleration_noise_std_dev_);
  imu_msg_.linear_acceleration.z = GRAVITY + Noise(linear_acceleration_noise_std_dev_);
}
}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Imu, flatland_server::ModelPlugin)
