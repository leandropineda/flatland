#include <flatland_plugins/gps.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <cmath>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(GpsPluginTest, load_test)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_gps_plugin");
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
    "flatland_server", "flatland_server::ModelPlugin");

  try {
    std::shared_ptr<flatland_server::ModelPlugin> plugin =
      loader.createSharedInstance("flatland_plugins::Gps");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Failed to load GPS plugin. " << e.what();
  }
}

// gps_tests/noise.world.yaml: one body at the (0, 0) reference with four Gps plugins: exact, noisy
// (noise_std_dev 0.5 m, seed 42), a second one with the same seed and a third with seed 7
class GpsNoiseTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_gps_noise");
    auto world = boost::filesystem::path(__FILE__).parent_path() / "gps_tests/noise.world.yaml";
    world_ = flatland_server::World::MakeWorld(node_, world.string());
    for (int i = 0; i < 4; i++) {
      auto & plugin = world_->plugin_manager_.model_plugins_[i];
      gps_[i] = dynamic_cast<flatland_plugins::Gps *>(plugin.get());
    }
  }
  void TearDown() override { delete world_; }

  rclcpp::Node::SharedPtr node_;
  flatland_server::World * world_;
  flatland_plugins::Gps * gps_[4];  // exact, noisy, same_seed, other_seed
};

TEST_F(GpsNoiseTest, noise_has_the_configured_sigma_and_a_known_covariance)
{
  gps_[0]->UpdateFix();
  const double lat0 = gps_[0]->gps_fix_.latitude, lon0 = gps_[0]->gps_fix_.longitude;
  const double m_per_deg_lat = 110574.3, m_per_deg_lon = 111319.5;  // WGS84 at the equator
  double sum_e = 0, sum_n = 0, sq_e = 0, sq_n = 0;
  const int n = 4000;
  for (int i = 0; i < n; i++) {
    gps_[1]->UpdateFix();
    double e = (gps_[1]->gps_fix_.longitude - lon0) * m_per_deg_lon;
    double north = (gps_[1]->gps_fix_.latitude - lat0) * m_per_deg_lat;
    sum_e += e, sum_n += north, sq_e += e * e, sq_n += north * north;
  }
  // sigma 0.5, not 1, so a covariance holding sigma instead of sigma^2 fails
  EXPECT_NEAR(sum_e / n, 0.0, 0.05);
  EXPECT_NEAR(sum_n / n, 0.0, 0.05);
  EXPECT_NEAR(std::sqrt(sq_e / n - (sum_e / n) * (sum_e / n)), 0.5, 0.025);
  EXPECT_NEAR(std::sqrt(sq_n / n - (sum_n / n) * (sum_n / n)), 0.5, 0.025);
  const auto & fix = gps_[1]->gps_fix_;
  EXPECT_EQ(
    fix.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
  EXPECT_DOUBLE_EQ(fix.position_covariance[0], 0.25);
  EXPECT_DOUBLE_EQ(fix.position_covariance[4], 0.25);
  EXPECT_DOUBLE_EQ(fix.position_covariance[8], 0.25);
}

TEST_F(GpsNoiseTest, the_same_seed_gives_the_same_noise_and_another_seed_does_not)
{
  for (int i = 0; i < 3; i++) {
    gps_[1]->UpdateFix();
    gps_[2]->UpdateFix();
    EXPECT_EQ(gps_[1]->gps_fix_.latitude, gps_[2]->gps_fix_.latitude);
    EXPECT_EQ(gps_[1]->gps_fix_.longitude, gps_[2]->gps_fix_.longitude);
  }
  gps_[3]->UpdateFix();  // seed 7: fails if the seed is ignored (every plugin the same default)
  EXPECT_NE(gps_[1]->gps_fix_.latitude, gps_[3]->gps_fix_.latitude);
}

TEST_F(GpsNoiseTest, no_noise_by_default_is_exact_with_an_unknown_covariance)
{
  gps_[0]->UpdateFix();
  EXPECT_NEAR(gps_[0]->gps_fix_.latitude, 0.0, 1e-12);
  EXPECT_NEAR(gps_[0]->gps_fix_.longitude, 0.0, 1e-12);
  EXPECT_EQ(
    gps_[0]->gps_fix_.position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
}

TEST(GpsPluginTest, a_negative_noise_std_dev_fails_the_model_load)
{
  auto node = rclcpp::Node::make_shared("test_gps_bad_noise");
  auto world = boost::filesystem::path(__FILE__).parent_path() / "gps_tests/bad_noise.world.yaml";
  EXPECT_THROW(
    delete flatland_server::World::MakeWorld(node, world.string()),
    flatland_server::PluginException);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
