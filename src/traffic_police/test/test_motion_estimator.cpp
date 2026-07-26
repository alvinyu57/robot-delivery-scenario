#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "traffic_police/motion_estimator.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

traffic_police::MotionEstimatorConfig estimator_config()
{
  traffic_police::MotionEstimatorConfig config;
  config.base_from_camera = traffic_police::make_rigid_transform(
    0.36, 0.0, 0.37, -kPi / 2.0, 0.0, -kPi / 2.0);
  config.ransac_threshold = 0.02;
  config.maximum_rigid_rmse = 0.01;
  config.maximum_linear_speed = 5.0;
  config.maximum_angular_speed = 5.0;
  config.maximum_sample_interval = 1.0;
  return config;
}

std::vector<cv::Point3f> synthetic_rgbd_points()
{
  std::vector<cv::Point3f> points;
  points.reserve(36U);
  for (int index = 0; index < 36; ++index) {
    const float x = 0.22F * static_cast<float>((index % 6) - 2);
    const float y = 0.18F * static_cast<float>(((index / 6) % 3) - 1);
    const float z =
      2.0F + 0.17F * static_cast<float>(index % 5) +
      0.11F * static_cast<float>(index / 15);
    points.emplace_back(x, y, z);
  }
  return points;
}

cv::Point3f transform_point(
  const cv::Matx44d & transform, const cv::Point3f & point)
{
  return cv::Point3f(
    static_cast<float>(
      transform(0, 0) * point.x + transform(0, 1) * point.y +
      transform(0, 2) * point.z + transform(0, 3)),
    static_cast<float>(
      transform(1, 0) * point.x + transform(1, 1) * point.y +
      transform(1, 2) * point.z + transform(1, 3)),
    static_cast<float>(
      transform(2, 0) * point.x + transform(2, 1) * point.y +
      transform(2, 2) * point.z + transform(2, 3)));
}

cv::Matx44d camera_scene_transform(
  const cv::Matx44d & previous_base_from_current_base,
  const cv::Matx44d & base_from_camera)
{
  const cv::Matx44d camera_from_base =
    traffic_police::invert_rigid_transform(base_from_camera);
  const cv::Matx44d previous_camera_from_current_camera =
    camera_from_base * previous_base_from_current_base * base_from_camera;
  return traffic_police::invert_rigid_transform(
    previous_camera_from_current_camera);
}

std::vector<cv::Point3f> transformed_points(
  const std::vector<cv::Point3f> & source,
  const cv::Matx44d & transform)
{
  std::vector<cv::Point3f> target;
  target.reserve(source.size());
  for (const auto & point : source) {
    target.push_back(transform_point(transform, point));
  }
  return target;
}

void expect_transform_near(
  const cv::Matx44d & actual,
  const cv::Matx44d & expected,
  double tolerance)
{
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      EXPECT_NEAR(actual(row, column), expected(row, column), tolerance);
    }
  }
}
}  // namespace

TEST(MotionEstimator, RecoversKnownBaseTranslation)
{
  const auto config = estimator_config();
  const traffic_police::MotionEstimator estimator(config);
  const cv::Matx44d base_motion =
    traffic_police::make_rigid_transform(0.18, -0.06, 0.0, 0.0, 0.0, 0.0);
  const auto previous_points = synthetic_rgbd_points();
  const auto current_points = transformed_points(
    previous_points,
    camera_scene_transform(base_motion, config.base_from_camera));

  cv::theRNG().state = 0x4d595df4d0f33173ULL;
  const auto estimate = estimator.estimate_from_correspondences(
    previous_points, current_points, 0.2);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->linear_speed, std::hypot(0.18, 0.06) / 0.2, 1.0e-4);
  EXPECT_NEAR(estimate->angular_speed, 0.0, 1.0e-5);
  EXPECT_EQ(estimate->inlier_count, previous_points.size());
  expect_transform_near(
    estimate->previous_base_from_current_base, base_motion, 1.0e-5);
}

TEST(MotionEstimator, CompensatesCameraLeverArmDuringPureBaseRotation)
{
  const auto config = estimator_config();
  const traffic_police::MotionEstimator estimator(config);
  const cv::Matx44d base_motion =
    traffic_police::make_rigid_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.24);
  const auto previous_points = synthetic_rgbd_points();
  const auto current_points = transformed_points(
    previous_points,
    camera_scene_transform(base_motion, config.base_from_camera));

  cv::theRNG().state = 0x6a09e667f3bcc909ULL;
  const auto estimate = estimator.estimate_from_correspondences(
    previous_points, current_points, 0.12);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->linear_speed, 0.0, 1.0e-4);
  EXPECT_NEAR(estimate->angular_speed, 2.0, 1.0e-4);
  expect_transform_near(
    estimate->previous_base_from_current_base, base_motion, 1.0e-5);
}

TEST(MotionEstimator, RejectsOutliersAndUsesActualInlierCount)
{
  auto config = estimator_config();
  config.minimum_inlier_ratio = 0.6;
  const traffic_police::MotionEstimator estimator(config);
  const cv::Matx44d base_motion =
    traffic_police::make_rigid_transform(0.15, 0.0, 0.0, 0.0, 0.0, -0.05);
  const auto previous_points = synthetic_rgbd_points();
  auto current_points = transformed_points(
    previous_points,
    camera_scene_transform(base_motion, config.base_from_camera));
  for (std::size_t index = 27U; index < current_points.size(); ++index) {
    current_points[index] = cv::Point3f(
      8.0F + static_cast<float>(index),
      -6.0F + 0.3F * static_cast<float>(index),
      1.0F + 0.7F * static_cast<float>(index));
  }

  cv::theRNG().state = 0xbb67ae8584caa73bULL;
  const auto estimate = estimator.estimate_from_correspondences(
    previous_points, current_points, 0.25);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->linear_speed, 0.6, 1.0e-4);
  EXPECT_NEAR(estimate->angular_speed, 0.2, 1.0e-4);
  EXPECT_EQ(estimate->correspondence_count, previous_points.size());
  EXPECT_EQ(estimate->inlier_count, 27U);
  EXPECT_LT(estimate->rigid_rmse, 1.0e-5);
}

TEST(MotionEstimator, RejectsImplausibleMotion)
{
  auto config = estimator_config();
  config.maximum_linear_speed = 1.0;
  const traffic_police::MotionEstimator estimator(config);
  const cv::Matx44d base_motion =
    traffic_police::make_rigid_transform(0.4, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto previous_points = synthetic_rgbd_points();
  const auto current_points = transformed_points(
    previous_points,
    camera_scene_transform(base_motion, config.base_from_camera));

  cv::theRNG().state = 0x3c6ef372fe94f82bULL;
  EXPECT_FALSE(
    estimator.estimate_from_correspondences(
      previous_points, current_points, 0.1).has_value());
}

TEST(MotionEstimator, ValidatesDenseRigidTransformAndDirection)
{
  const auto config = estimator_config();
  const traffic_police::MotionEstimator estimator(config);
  const cv::Matx44d base_motion =
    traffic_police::make_rigid_transform(
    0.08, -0.03, 0.0, 0.0, 0.0, -0.12);
  const cv::Matx44d camera_motion =
    camera_scene_transform(base_motion, config.base_from_camera);
  const cv::Mat camera_motion_matrix(camera_motion, true);

  const auto estimate = estimator.estimate_from_camera_transform(
    camera_motion_matrix, 0.2);

  ASSERT_TRUE(estimate.has_value());
  expect_transform_near(
    estimate->previous_base_from_current_base, base_motion, 1.0e-8);
  EXPECT_EQ(estimate->correspondence_count, 0U);
  EXPECT_EQ(estimate->inlier_count, 0U);
}

TEST(MotionEstimator, RejectsNonRigidDenseTransform)
{
  const traffic_police::MotionEstimator estimator(estimator_config());
  cv::Matx44d non_rigid = cv::Matx44d::eye();
  non_rigid(0, 0) = 1.02;
  const cv::Mat non_rigid_matrix(non_rigid, true);

  EXPECT_FALSE(
    estimator.estimate_from_camera_transform(
      non_rigid_matrix, 0.1).has_value());
}
