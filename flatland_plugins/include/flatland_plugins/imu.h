/*
 * @name     imu.h
 * @brief    Simulated 2D IMU: orientation, yaw rate and linear acceleration of a point on a body
 */

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <tf2_ros/transform_broadcaster.h>

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#ifndef FLATLAND_PLUGINS_IMU_H
#define FLATLAND_PLUGINS_IMU_H

using namespace flatland_server;

namespace flatland_plugins
{

/**
 * Publishes sensor_msgs/Imu for the frame at `origin` on a body: its yaw as orientation, the
 * body's yaw rate, and the linear acceleration of that point (from its velocity between steps, so a
 * turn reads as centripetal acceleration) plus gravity on z, all in the IMU frame, with optional
 * gaussian noise.
 */
class Imu : public ModelPlugin
{
public:
  static constexpr double GRAVITY = 9.80665;  ///< m/s^2, reported on z

  std::string topic_;     ///< topic to publish on
  std::string frame_id_;  ///< IMU frame id
  Body * body_;           ///< body the IMU is mounted on
  Pose origin_;           ///< IMU frame w.r.t. the body
  double update_rate_;    ///< publish rate, Hz
  bool broadcast_tf_;     ///< whether to broadcast the body -> IMU transform
  double orientation_noise_std_dev_;          ///< rad
  double angular_velocity_noise_std_dev_;     ///< rad/s
  double linear_acceleration_noise_std_dev_;  ///< m/s^2

  std::mt19937 rng_;                                ///< seeded by the seed parameter (0 = random)
  std::normal_distribution<double> unit_{0.0, 1.0};
  b2Vec2 previous_velocity_{0.0f, 0.0f};  ///< world velocity of the IMU point at the last step

  sensor_msgs::msg::Imu imu_msg_;  ///< the latest reading, updated every step
  geometry_msgs::msg::TransformStamped imu_tf_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  UpdateTimer update_timer_;

  void OnInitialize(const YAML::Node & config) override;

  /// Updates the reading after every physics step, from the velocity the solver produced (so it
  /// does not depend on plugin order, and contacts show); publishes at update_rate
  void AfterPhysicsStep(const Timekeeper & timekeeper) override;

  /// Updates imu_msg_ from the body's current state; dt is the time since the previous update
  void UpdateImu(double dt);

private:
  double Noise(double std_dev) { return std_dev * unit_(rng_); }
};
}  // namespace flatland_plugins

#endif
