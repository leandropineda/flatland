/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	diff_drive.h
 * @brief   Diff drive plugin
 * @author  Mike Brousseau
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <random>

#ifndef FLATLAND_PLUGINS_DIFFDRIVE_H
#define FLATLAND_PLUGINS_DIFFDRIVE_H

using namespace flatland_server;

namespace flatland_plugins
{

class DiffDrive : public flatland_server::ModelPlugin
{
public:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  Body * body_;
  geometry_msgs::msg::Twist twist_msg_;  ///< last commanded velocity, zeroed once stale
  double cmd_vel_timeout_ = 0.0;  ///< s; a command older than this reads as zero, 0 disables
  double cmd_age_ = 0.0;          ///< s of sim time the last command has been applied for
  double max_linear_acceleration_ = 0.0;   ///< m/s^2 while speeding up, 0 = unlimited
  double max_linear_deceleration_ = 0.0;   ///< m/s^2 slowing down or reversing, 0 = unlimited
  double max_angular_acceleration_ = 0.0;  ///< rad/s^2 either way, 0 = unlimited
  geometry_msgs::msg::Twist applied_twist_;  ///< velocity applied last step, within the limits
  bool twist_in_body_frame_ = false;  ///< odom and ground truth twist in the body (child) frame
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  bool paused_ = false;  ///< while true the body is held still and cmd_vel is ignored
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Odometry ground_truth_msg_;
  UpdateTimer update_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< For publish ROS TF
  bool enable_odom_pub_;   ///< YAML parameter to enable odom publishing
  bool enable_twist_pub_;  ///< YAML parameter to enable twist publishing

  geometry_msgs::msg::TwistStamped twist_pub_msg_;  ///< the last simulated encoder reading

  std::mt19937 rng_;  ///< seeded by the seed parameter (0 = random)
  std::array<double, 6> noise_std_dev_;  ///< odom pose x, y, yaw, then twist x, y, yaw
  std::array<double, 2> twist_pub_scale_error_{};  ///< forward speed, yaw rate; drawn once per run
  // a zero-sigma normal_distribution is undefined behavior (it aborts under _GLIBCXX_ASSERTIONS)
  std::normal_distribution<double> unit_{0.0, 1.0};
  double Noise(int i) { return noise_std_dev_[i] * unit_(rng_); }

  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node & config) override;
  /**
   * @name          BeforePhysicsStep
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void BeforePhysicsStep(const Timekeeper & timekeeper) override;

  /**
   * @name          Command
   * @brief         set the commanded velocity, as a message on the twist topic does
   * @param[in]     cmd Forward (linear.x) and rotation (angular.z) velocity in the body frame
   */
  void Command(const geometry_msgs::msg::Twist & cmd);
};
}  // namespace flatland_plugins

#endif
