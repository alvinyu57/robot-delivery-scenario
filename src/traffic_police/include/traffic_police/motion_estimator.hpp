#ifndef TRAFFIC_POLICE__MOTION_ESTIMATOR_HPP_
#define TRAFFIC_POLICE__MOTION_ESTIMATOR_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace traffic_police
{

struct MotionEstimatorConfig
{
  cv::Matx44d base_from_camera{cv::Matx44d::eye()};
  std::size_t minimum_correspondences{12U};
  std::size_t minimum_inliers{8U};
  double minimum_inlier_ratio{0.5};
  double ransac_threshold{0.08};
  double ransac_confidence{0.99};
  double maximum_rigid_rmse{0.03};
  double maximum_linear_speed{2.0};
  double maximum_angular_speed{3.0};
  double maximum_sample_interval{0.5};
};

struct MotionEstimate
{
  double linear_speed;
  double angular_speed;
  std::size_t correspondence_count;
  std::size_t inlier_count;
  double rigid_rmse;
  cv::Matx44d previous_base_from_current_base;
};

class MotionEstimator
{
public:
  explicit MotionEstimator(MotionEstimatorConfig config);

  std::optional<MotionEstimate> estimate_from_camera_transform(
    const cv::Mat & current_camera_from_previous_camera,
    double elapsed_seconds) const;

  std::optional<MotionEstimate> estimate_from_correspondences(
    const std::vector<cv::Point3f> & previous_camera_points,
    const std::vector<cv::Point3f> & current_camera_points,
    double elapsed_seconds) const;

private:
  std::optional<MotionEstimate> make_motion_estimate(
    const cv::Matx44d & current_camera_from_previous_camera,
    double elapsed_seconds,
    std::size_t correspondence_count,
    std::size_t inlier_count,
    double rigid_rmse) const;

  MotionEstimatorConfig config_;
  cv::Matx44d camera_from_base_;
};

cv::Matx44d make_rigid_transform(
  double x, double y, double z,
  double roll, double pitch, double yaw);

cv::Matx44d invert_rigid_transform(const cv::Matx44d & transform);

}  // namespace traffic_police

#endif  // TRAFFIC_POLICE__MOTION_ESTIMATOR_HPP_
