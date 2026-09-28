#include <flatland_plugins/diff_drive.h>
#include <flatland_plugins/imu.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using flatland_plugins::DiffDrive;
using flatland_plugins::Imu;

static double Yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// imu_tests/imu.world.yaml, 0.1 s steps: a body at yaw 0.5 with a DiffDrive (1 m/s^2 linear
/// limit) and Imu plugins: first (listed before the drive), level, yawed (mounted at +90 deg),
/// offset (0.5 m ahead of the body origin) and noisy (seeded)
class ImuPluginTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_imu");
    timekeeper_ = std::make_unique<flatland_server::Timekeeper>(node_);
    timekeeper_->SetMaxStepSize(0.1);
    auto world = boost::filesystem::path(__FILE__).parent_path() / "imu_tests/imu.world.yaml";
    world_ = flatland_server::World::MakeWorld(node_, world.string());
    auto & plugins = world_->plugin_manager_.model_plugins_;
    first_ = dynamic_cast<Imu *>(plugins[0].get());
    drive_ = dynamic_cast<DiffDrive *>(plugins[1].get());
    level_ = dynamic_cast<Imu *>(plugins[2].get());
    yawed_ = dynamic_cast<Imu *>(plugins[3].get());
    offset_ = dynamic_cast<Imu *>(plugins[4].get());
    noisy_ = dynamic_cast<Imu *>(plugins[5].get());
  }
  void TearDown() override { delete world_; }
  void Step(int n)
  {
    for (int i = 0; i < n; i++) {
      world_->Update(*timekeeper_);
    }
  }
  void Drive(double v, double w)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    drive_->Command(cmd);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<flatland_server::Timekeeper> timekeeper_;
  flatland_server::World * world_;
  DiffDrive * drive_;
  Imu *first_, *level_, *yawed_, *offset_, *noisy_;
};

TEST_F(ImuPluginTest, at_rest_it_reads_gravity_and_its_own_yaw)
{
  Step(3);
  const auto & m = level_->imu_msg_;
  EXPECT_NEAR(m.linear_acceleration.x, 0.0, 1e-6);
  EXPECT_NEAR(m.linear_acceleration.y, 0.0, 1e-6);
  EXPECT_NEAR(m.linear_acceleration.z, 9.80665, 1e-6);
  EXPECT_NEAR(m.angular_velocity.z, 0.0, 1e-6);
  EXPECT_NEAR(Yaw(m.orientation), 0.5, 1e-6);
  EXPECT_NEAR(Yaw(yawed_->imu_msg_.orientation), 0.5 + M_PI / 2, 1e-6);
}

TEST_F(ImuPluginTest, forward_acceleration_lands_on_the_mount_axes)
{
  Drive(1.0, 0.0);  // speeds up at 1 m/s^2 for 1 s
  Step(5);
  EXPECT_NEAR(level_->imu_msg_.linear_acceleration.x, 1.0, 1e-3);
  EXPECT_NEAR(level_->imu_msg_.linear_acceleration.y, 0.0, 1e-3);
  EXPECT_NEAR(yawed_->imu_msg_.linear_acceleration.x, 0.0, 1e-3);  // mounted +90 deg: forward is -y
  EXPECT_NEAR(yawed_->imu_msg_.linear_acceleration.y, -1.0, 1e-3);
}

TEST_F(ImuPluginTest, a_turn_reads_on_the_gyro_and_as_centripetal_acceleration)
{
  Drive(1.0, 1.0);  // turns at 1 rad/s at once, reaches 1 m/s after 1 s
  Step(20);
  EXPECT_NEAR(level_->imu_msg_.angular_velocity.z, 1.0, 1e-3);
  // v * w toward the centre, seen from a frame 1.5 steps ahead of it: the velocities are each
  // step's and the yaw is the step's end, so x reads sin(1.5 w dt) = 0.149 at 0.1 s steps (0.0075
  // at the default 5 ms)
  EXPECT_NEAR(level_->imu_msg_.linear_acceleration.y, std::cos(0.15), 0.005);
  EXPECT_NEAR(level_->imu_msg_.linear_acceleration.x, std::sin(0.15), 0.005);
}

TEST_F(ImuPluginTest, an_offset_mount_reads_the_centripetal_acceleration_of_its_point)
{
  Drive(0.0, 1.0);  // turn in place at 1 rad/s
  Step(3);
  EXPECT_NEAR(offset_->imu_msg_.linear_acceleration.x, -0.5, 0.01);  // -w^2 r, toward the centre
  EXPECT_NEAR(level_->imu_msg_.linear_acceleration.x, 0.0, 1e-6);    // on the axis: none
}

TEST_F(ImuPluginTest, the_reading_does_not_depend_on_the_plugin_order)
{
  Drive(1.0, 1.0);
  Step(5);
  EXPECT_DOUBLE_EQ(first_->imu_msg_.linear_acceleration.x, level_->imu_msg_.linear_acceleration.x);
  EXPECT_DOUBLE_EQ(first_->imu_msg_.linear_acceleration.y, level_->imu_msg_.linear_acceleration.y);
  EXPECT_DOUBLE_EQ(first_->imu_msg_.angular_velocity.z, level_->imu_msg_.angular_velocity.z);
}

TEST_F(ImuPluginTest, noise_has_its_sigma_and_a_known_covariance)
{
  const int n = 2000;  // at rest, so each reading is its noise
  double sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
  for (int i = 0; i < n; i++) {
    Step(1);
    const auto & m = noisy_->imu_msg_;
    double x[3] = {m.angular_velocity.z, m.linear_acceleration.x, Yaw(m.orientation) - 0.5};
    for (int k = 0; k < 3; k++) {
      sum[k] += x[k], sq[k] += x[k] * x[k];
    }
  }
  const double sigma[3] = {0.01, 0.1, 0.02};
  for (int k = 0; k < 3; k++) {
    EXPECT_NEAR(std::sqrt(sq[k] / n - (sum[k] / n) * (sum[k] / n)), sigma[k], 0.1 * sigma[k]) << k;
  }
  const auto & m = noisy_->imu_msg_;
  EXPECT_DOUBLE_EQ(m.angular_velocity_covariance[8], 0.01 * 0.01);
  EXPECT_DOUBLE_EQ(m.linear_acceleration_covariance[0], 0.1 * 0.1);
  EXPECT_DOUBLE_EQ(m.orientation_covariance[8], 0.02 * 0.02);
}

TEST(ImuLoadTest, a_negative_noise_std_dev_fails_the_model_load)
{
  auto node = rclcpp::Node::make_shared("test_imu_bad_noise");
  auto world = boost::filesystem::path(__FILE__).parent_path() / "imu_tests/bad_noise.world.yaml";
  EXPECT_THROW(
    delete flatland_server::World::MakeWorld(node, world.string()),
    flatland_server::PluginException);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
