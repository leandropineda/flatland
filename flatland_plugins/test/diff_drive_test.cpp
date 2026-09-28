/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  diff_drive_test.cpp
 * @brief test diff drive plugin
 * @author Chunshang Li
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

#include <flatland_plugins/diff_drive.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

using flatland_plugins::DiffDrive;
using flatland_server::Timekeeper;
using flatland_server::World;

TEST(DiffDrivePluginTest, load_test)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_diff_drive_plugin");
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
    "flatland_server", "flatland_server::ModelPlugin");

  try {
    std::shared_ptr<flatland_server::ModelPlugin> plugin =
      loader.createSharedInstance("flatland_plugins::DiffDrive");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Failed to load diff drive Drive plugin. " << e.what();
  }
}

/// Loads diff_drive_tests/<name>.world.yaml, 0.1 s steps; its model's only plugin is a DiffDrive
class DiffDriveWorld
{
public:
  explicit DiffDriveWorld(const std::string & name)
  : node_(rclcpp::Node::make_shared("test_diff_drive_" + name)), timekeeper_(node_)
  {
    timekeeper_.SetMaxStepSize(0.1);
    auto dir = boost::filesystem::path(__FILE__).parent_path() / "diff_drive_tests";
    world_ = World::MakeWorld(node_, (dir / (name + ".world.yaml")).string());
    drive_ = dynamic_cast<DiffDrive *>(world_->plugin_manager_.model_plugins_[0].get());
  }
  ~DiffDriveWorld() { delete world_; }
  void Step(int n)
  {
    for (int i = 0; i < n; i++) {
      world_->Update(timekeeper_);
    }
  }
  double Speed() { return drive_->body_->physics_body_->GetLinearVelocity().Length(); }
  double Turn() { return drive_->body_->physics_body_->GetAngularVelocity(); }
  const geometry_msgs::msg::Vector3 & GroundTruthLinear()
  {
    return drive_->ground_truth_msg_.twist.twist.linear;
  }
  const geometry_msgs::msg::Vector3 & OdomLinear() { return drive_->odom_msg_.twist.twist.linear; }
  void Drive(double v, double w = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    drive_->Command(cmd);
  }
  void Pause(bool paused) { drive_->paused_ = paused; }

private:
  rclcpp::Node::SharedPtr node_;
  Timekeeper timekeeper_;
  World * world_;
  DiffDrive * drive_;
};

TEST(DiffDriveCmdTimeoutTest, a_command_older_than_the_timeout_stops_the_body)
{
  DiffDriveWorld w("timeout");  // cmd_vel_timeout: 0.5
  w.Drive(1.0, 1.0);
  w.Step(6);  // the last step applies it at exactly 0.5 s old: still fresh
  EXPECT_NEAR(w.Speed(), 1.0, 1e-3);
  EXPECT_NEAR(w.Turn(), 1.0, 1e-3);
  w.Step(1);  // 0.6 s old: stale, the whole twist reads as zero
  EXPECT_NEAR(w.Speed(), 0.0, 1e-3);
  EXPECT_NEAR(w.Turn(), 0.0, 1e-3);
}

TEST(DiffDriveCmdTimeoutTest, a_command_sent_during_a_pause_is_stale_after_it)
{
  DiffDriveWorld w("timeout");
  w.Pause(true);
  w.Drive(1.0);  // the controller sends one command, then dies
  w.Step(6);
  w.Pause(false);
  w.Step(1);
  EXPECT_NEAR(w.Speed(), 0.0, 1e-3);
}

TEST(DiffDriveCmdTimeoutTest, a_negative_timeout_fails_the_model_load)
{
  EXPECT_THROW(DiffDriveWorld w("bad_timeout"), flatland_server::PluginException);
}

TEST(DiffDriveCmdTimeoutTest, each_new_command_restarts_the_timeout)
{
  DiffDriveWorld w("timeout");
  for (int i = 0; i < 4; i++) {  // a command every 0.4 s: fresher than the 0.5 s timeout
    w.Drive(1.0);
    w.Step(4);
  }
  EXPECT_NEAR(w.Speed(), 1.0, 1e-3);
}

TEST(DiffDriveCmdTimeoutTest, no_timeout_by_default_keeps_the_last_command)
{
  DiffDriveWorld w("no_timeout");
  w.Drive(1.0);
  w.Step(20);  // 2 s
  EXPECT_NEAR(w.Speed(), 1.0, 1e-3);
}

TEST(DiffDriveAccelTest, speeding_up_is_limited_by_max_linear_acceleration)
{
  DiffDriveWorld w("accel");  // 1.0 m/s2 up, 2.0 m/s2 down
  w.Drive(1.0);
  w.Step(5);  // 0.5 s at 1 m/s2
  EXPECT_NEAR(w.Speed(), 0.5, 1e-3);
  w.Drive(1.0);
  w.Step(5);
  EXPECT_NEAR(w.Speed(), 1.0, 1e-3);
}

TEST(DiffDriveAccelTest, slowing_down_is_limited_by_max_linear_deceleration)
{
  DiffDriveWorld w("accel");
  for (int i = 0; i < 3; i++) {  // up to 1 m/s
    w.Drive(1.0);
    w.Step(4);
  }
  w.Drive(0.0);
  w.Step(2);  // 0.2 s at 2 m/s2
  EXPECT_NEAR(w.Speed(), 0.6, 1e-3);
  w.Step(3);
  EXPECT_NEAR(w.Speed(), 0.0, 1e-3);
}

TEST(DiffDriveAccelTest, reversing_counts_as_slowing_down)
{
  DiffDriveWorld w("accel");
  w.Drive(1.0);
  w.Step(5);  // 0.5 m/s
  w.Drive(-1.0);
  w.Step(1);  // 0.1 s at the 2 m/s2 deceleration, not the 1 m/s2 acceleration
  EXPECT_NEAR(w.Speed(), 0.3, 1e-3);
}

TEST(DiffDriveAccelTest, the_deceleration_limit_defaults_to_the_acceleration_limit)
{
  DiffDriveWorld w("accel_only");  // max_linear_acceleration: 1.0 only
  w.Drive(1.0);
  w.Step(5);  // 0.5 m/s
  w.Drive(-1.0);
  w.Step(1);  // reversing at 1 m/s2, not an instant 2 m/s jump
  EXPECT_NEAR(w.Speed(), 0.4, 1e-3);
}

TEST(DiffDriveAccelTest, a_negative_limit_fails_the_model_load)
{
  // Nav2's velocity_smoother writes deceleration as a negative number: here it would mean unlimited
  EXPECT_THROW(DiffDriveWorld w("bad_accel"), flatland_server::PluginException);
}

TEST(DiffDriveAccelTest, a_timed_out_command_ramps_down_instead_of_stopping_dead)
{
  DiffDriveWorld w("accel");  // cmd_vel_timeout: 0.5
  for (int i = 0; i < 3; i++) {
    w.Drive(1.0);
    w.Step(4);
  }
  // the last command becomes 0.7 s old: stale at 0.6 and 0.7 s, 0.2 m/s shed per step at 2 m/s2
  w.Step(4);
  EXPECT_NEAR(w.Speed(), 0.6, 1e-3);
  w.Step(3);
  EXPECT_NEAR(w.Speed(), 0.0, 1e-3);
}

TEST(DiffDriveAccelTest, resuming_from_a_pause_speeds_up_within_the_limit_again)
{
  DiffDriveWorld w("accel");  // 1.0 m/s2 up
  for (int i = 0; i < 3; i++) {  // up to 1 m/s
    w.Drive(1.0);
    w.Step(4);
  }
  w.Pause(true);
  w.Step(1);
  EXPECT_NEAR(w.Speed(), 0.0, 1e-3);
  w.Pause(false);
  w.Drive(1.0);
  w.Step(3);  // from 0, not from the 1 m/s before the pause
  EXPECT_NEAR(w.Speed(), 0.3, 1e-3);
}

TEST(DiffDriveAccelTest, turning_is_limited_by_max_angular_acceleration)
{
  DiffDriveWorld w("accel");  // 3.0 rad/s2, apart from the linear limits
  w.Drive(0.0, 1.0);
  w.Step(2);  // 0.2 s
  EXPECT_NEAR(w.Turn(), 0.6, 1e-3);
}

TEST(DiffDriveAccelTest, no_limits_by_default_apply_the_command_at_once)
{
  DiffDriveWorld w("no_timeout");
  w.Drive(1.0, 1.0);
  w.Step(1);
  EXPECT_NEAR(w.Speed(), 1.0, 1e-3);
  EXPECT_NEAR(w.Turn(), 1.0, 1e-3);
}

TEST(DiffDriveTwistFrameTest, by_default_the_odometry_twist_is_in_the_world_frame)
{
  DiffDriveWorld w("twist_world");  // facing +y
  w.Drive(1.0);
  w.Step(2);  // the message reports the previous step's velocity
  EXPECT_NEAR(w.GroundTruthLinear().x, 0.0, 1e-3);
  EXPECT_NEAR(w.GroundTruthLinear().y, 1.0, 1e-3);
  EXPECT_NEAR(w.OdomLinear().x, 0.0, 1e-3);  // no odom noise in this model
  EXPECT_NEAR(w.OdomLinear().y, 1.0, 1e-3);
}

TEST(DiffDriveTwistFrameTest, twist_frame_body_reports_it_in_the_child_frame)
{
  DiffDriveWorld w("twist_body");  // facing +y, twist_frame: body
  w.Drive(1.0);
  w.Step(2);  // the message reports the previous step's velocity
  EXPECT_NEAR(w.GroundTruthLinear().x, 1.0, 1e-3);
  EXPECT_NEAR(w.GroundTruthLinear().y, 0.0, 1e-3);
  EXPECT_NEAR(w.OdomLinear().x, 1.0, 1e-3);
  EXPECT_NEAR(w.OdomLinear().y, 0.0, 1e-3);
}

TEST(DiffDriveTwistFrameTest, an_unknown_twist_frame_fails_the_model_load)
{
  EXPECT_THROW(DiffDriveWorld w("twist_bad"), flatland_server::PluginException);  // "Body"
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
