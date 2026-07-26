#include "traffic_police/motion_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace
{
constexpr double kRigidTolerance = 1.0e-3;
constexpr double kDegenerateTolerance = 1.0e-10;

bool finite_transform(const cv::Matx44d & transform)
{
  for (const double value : transform.val) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

cv::Matx33d rotation_part(const cv::Matx44d & transform)
{
  return cv::Matx33d(
    transform(0, 0), transform(0, 1), transform(0, 2),
    transform(1, 0), transform(1, 1), transform(1, 2),
    transform(2, 0), transform(2, 1), transform(2, 2));
}

cv::Vec3d translation_part(const cv::Matx44d & transform)
{
  return cv::Vec3d(transform(0, 3), transform(1, 3), transform(2, 3));
}

cv::Matx44d compose_transform(
  const cv::Matx33d & rotation, const cv::Vec3d & translation)
{
  cv::Matx44d transform = cv::Matx44d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      transform(row, column) = rotation(row, column);
    }
    transform(row, 3) = translation[row];
  }
  return transform;
}

bool valid_rigid_transform(const cv::Matx44d & transform)
{
  if (!finite_transform(transform) ||
    std::abs(transform(3, 0)) > kRigidTolerance ||
    std::abs(transform(3, 1)) > kRigidTolerance ||
    std::abs(transform(3, 2)) > kRigidTolerance ||
    std::abs(transform(3, 3) - 1.0) > kRigidTolerance)
  {
    return false;
  }

  const cv::Matx33d rotation = rotation_part(transform);
  const cv::Matx33d orthogonality_error =
    rotation.t() * rotation - cv::Matx33d::eye();
  const double error =
    cv::norm(cv::Mat(orthogonality_error), cv::NORM_INF);
  const double determinant = cv::determinant(cv::Mat(rotation));
  return error <= kRigidTolerance &&
         std::abs(determinant - 1.0) <= kRigidTolerance;
}

bool finite_point(const cv::Point3f & point)
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

cv::Vec3d transform_point(
  const cv::Matx44d & transform, const cv::Point3f & point)
{
  return cv::Vec3d(
    transform(0, 0) * point.x + transform(0, 1) * point.y +
    transform(0, 2) * point.z + transform(0, 3),
    transform(1, 0) * point.x + transform(1, 1) * point.y +
    transform(1, 2) * point.z + transform(1, 3),
    transform(2, 0) * point.x + transform(2, 1) * point.y +
    transform(2, 2) * point.z + transform(2, 3));
}

std::optional<cv::Matx44d> fit_rigid_transform(
  const std::vector<cv::Point3f> & source,
  const std::vector<cv::Point3f> & target,
  const std::vector<std::size_t> & indices)
{
  if (indices.size() < 3U) {
    return std::nullopt;
  }

  cv::Vec3d source_centroid(0.0, 0.0, 0.0);
  cv::Vec3d target_centroid(0.0, 0.0, 0.0);
  for (const std::size_t index : indices) {
    source_centroid += cv::Vec3d(
      source[index].x, source[index].y, source[index].z);
    target_centroid += cv::Vec3d(
      target[index].x, target[index].y, target[index].z);
  }
  source_centroid *= 1.0 / static_cast<double>(indices.size());
  target_centroid *= 1.0 / static_cast<double>(indices.size());

  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (const std::size_t index : indices) {
    const cv::Vec3d source_offset =
      cv::Vec3d(source[index].x, source[index].y, source[index].z) -
      source_centroid;
    const cv::Vec3d target_offset =
      cv::Vec3d(target[index].x, target[index].y, target[index].z) -
      target_centroid;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance(row, column) +=
          source_offset[row] * target_offset[column];
      }
    }
  }

  cv::SVD decomposition(cv::Mat(covariance), cv::SVD::FULL_UV);
  if (decomposition.w.total() < 2U ||
    decomposition.w.at<double>(1) <= kDegenerateTolerance)
  {
    return std::nullopt;
  }

  cv::Mat rotation = decomposition.vt.t() * decomposition.u.t();
  if (cv::determinant(rotation) < 0.0) {
    cv::Mat adjusted_v = decomposition.vt.t();
    adjusted_v.col(2) *= -1.0;
    rotation = adjusted_v * decomposition.u.t();
  }

  cv::Matx33d rigid_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rigid_rotation(row, column) = rotation.at<double>(row, column);
    }
  }
  const cv::Vec3d rigid_translation =
    target_centroid - rigid_rotation * source_centroid;
  const cv::Matx44d transform =
    compose_transform(rigid_rotation, rigid_translation);
  if (!valid_rigid_transform(transform)) {
    return std::nullopt;
  }
  return transform;
}

double point_residual(
  const cv::Matx44d & transform,
  const cv::Point3f & source,
  const cv::Point3f & target)
{
  const cv::Vec3d predicted = transform_point(transform, source);
  const cv::Vec3d observed(target.x, target.y, target.z);
  return cv::norm(predicted - observed);
}

std::optional<cv::Matx44d> mat_to_transform(const cv::Mat & matrix)
{
  if (matrix.rows != 4 || matrix.cols != 4 || matrix.channels() != 1) {
    return std::nullopt;
  }

  cv::Mat converted;
  matrix.convertTo(converted, CV_64FC1);
  cv::Matx44d transform;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      transform(row, column) = converted.at<double>(row, column);
    }
  }
  if (!valid_rigid_transform(transform)) {
    return std::nullopt;
  }
  return transform;
}
}  // namespace

namespace traffic_police
{

MotionEstimator::MotionEstimator(MotionEstimatorConfig config)
: config_(std::move(config))
{
  if (!valid_rigid_transform(config_.base_from_camera)) {
    throw std::invalid_argument("base_from_camera must be a finite rigid transform");
  }
  if (config_.minimum_correspondences < 4U ||
    config_.minimum_inliers < 3U ||
    config_.minimum_inliers > config_.minimum_correspondences)
  {
    throw std::invalid_argument("invalid correspondence or inlier minimum");
  }
  if (!std::isfinite(config_.minimum_inlier_ratio) ||
    config_.minimum_inlier_ratio <= 0.0 ||
    config_.minimum_inlier_ratio > 1.0)
  {
    throw std::invalid_argument("minimum_inlier_ratio must be in (0, 1]");
  }
  if (!std::isfinite(config_.ransac_threshold) ||
    config_.ransac_threshold <= 0.0)
  {
    throw std::invalid_argument("ransac_threshold must be positive and finite");
  }
  if (!std::isfinite(config_.ransac_confidence) ||
    config_.ransac_confidence <= 0.0 ||
    config_.ransac_confidence >= 1.0)
  {
    throw std::invalid_argument("ransac_confidence must be in (0, 1)");
  }
  if (!std::isfinite(config_.maximum_rigid_rmse) ||
    config_.maximum_rigid_rmse <= 0.0 ||
    !std::isfinite(config_.maximum_linear_speed) ||
    config_.maximum_linear_speed <= 0.0 ||
    !std::isfinite(config_.maximum_angular_speed) ||
    config_.maximum_angular_speed <= 0.0 ||
    !std::isfinite(config_.maximum_sample_interval) ||
    config_.maximum_sample_interval <= 0.0)
  {
    throw std::invalid_argument("motion validation limits must be positive and finite");
  }
  camera_from_base_ = invert_rigid_transform(config_.base_from_camera);
}

std::optional<MotionEstimate> MotionEstimator::estimate_from_camera_transform(
  const cv::Mat & current_camera_from_previous_camera,
  double elapsed_seconds) const
{
  const auto transform = mat_to_transform(current_camera_from_previous_camera);
  if (!transform) {
    return std::nullopt;
  }
  return make_motion_estimate(
    *transform, elapsed_seconds, 0U, 0U, 0.0);
}

std::optional<MotionEstimate> MotionEstimator::estimate_from_correspondences(
  const std::vector<cv::Point3f> & previous_camera_points,
  const std::vector<cv::Point3f> & current_camera_points,
  double elapsed_seconds) const
{
  if (previous_camera_points.size() != current_camera_points.size() ||
    previous_camera_points.size() < config_.minimum_correspondences)
  {
    return std::nullopt;
  }
  if (!std::all_of(
      previous_camera_points.begin(), previous_camera_points.end(), finite_point) ||
    !std::all_of(
      current_camera_points.begin(), current_camera_points.end(), finite_point))
  {
    return std::nullopt;
  }

  cv::Mat affine_transform;
  cv::Mat affine_inlier_mask;
  const int success = cv::estimateAffine3D(
    previous_camera_points, current_camera_points,
    affine_transform, affine_inlier_mask,
    config_.ransac_threshold, config_.ransac_confidence);
  if (success == 0 ||
    affine_transform.rows != 3 || affine_transform.cols != 4 ||
    affine_inlier_mask.total() != previous_camera_points.size())
  {
    return std::nullopt;
  }

  std::vector<std::size_t> affine_inliers;
  affine_inliers.reserve(affine_inlier_mask.total());
  const auto * mask = affine_inlier_mask.ptr<std::uint8_t>();
  for (std::size_t index = 0; index < affine_inlier_mask.total(); ++index) {
    if (mask[index] != 0U) {
      affine_inliers.push_back(index);
    }
  }
  const double affine_inlier_ratio =
    static_cast<double>(affine_inliers.size()) /
    static_cast<double>(previous_camera_points.size());
  if (affine_inliers.size() < config_.minimum_inliers ||
    affine_inlier_ratio < config_.minimum_inlier_ratio)
  {
    return std::nullopt;
  }

  const auto initial_rigid = fit_rigid_transform(
    previous_camera_points, current_camera_points, affine_inliers);
  if (!initial_rigid) {
    return std::nullopt;
  }

  std::vector<std::size_t> rigid_inliers;
  rigid_inliers.reserve(previous_camera_points.size());
  for (std::size_t index = 0; index < previous_camera_points.size(); ++index) {
    if (point_residual(
        *initial_rigid, previous_camera_points[index],
        current_camera_points[index]) <= config_.ransac_threshold)
    {
      rigid_inliers.push_back(index);
    }
  }
  const double rigid_inlier_ratio =
    static_cast<double>(rigid_inliers.size()) /
    static_cast<double>(previous_camera_points.size());
  if (rigid_inliers.size() < config_.minimum_inliers ||
    rigid_inlier_ratio < config_.minimum_inlier_ratio)
  {
    return std::nullopt;
  }

  const auto refined_rigid = fit_rigid_transform(
    previous_camera_points, current_camera_points, rigid_inliers);
  if (!refined_rigid) {
    return std::nullopt;
  }

  std::vector<std::size_t> final_inliers;
  final_inliers.reserve(rigid_inliers.size());
  for (std::size_t index = 0; index < previous_camera_points.size(); ++index) {
    if (point_residual(
        *refined_rigid, previous_camera_points[index],
        current_camera_points[index]) <= config_.ransac_threshold)
    {
      final_inliers.push_back(index);
    }
  }
  const double final_inlier_ratio =
    static_cast<double>(final_inliers.size()) /
    static_cast<double>(previous_camera_points.size());
  if (final_inliers.size() < config_.minimum_inliers ||
    final_inlier_ratio < config_.minimum_inlier_ratio)
  {
    return std::nullopt;
  }

  double squared_error_sum = 0.0;
  for (const std::size_t index : final_inliers) {
    const double residual = point_residual(
      *refined_rigid, previous_camera_points[index],
      current_camera_points[index]);
    squared_error_sum += residual * residual;
  }
  const double rigid_rmse = std::sqrt(
    squared_error_sum / static_cast<double>(final_inliers.size()));
  if (!std::isfinite(rigid_rmse) ||
    rigid_rmse > config_.maximum_rigid_rmse)
  {
    return std::nullopt;
  }

  return make_motion_estimate(
    *refined_rigid, elapsed_seconds,
    previous_camera_points.size(), final_inliers.size(), rigid_rmse);
}

std::optional<MotionEstimate> MotionEstimator::make_motion_estimate(
  const cv::Matx44d & current_camera_from_previous_camera,
  double elapsed_seconds,
  std::size_t correspondence_count,
  std::size_t inlier_count,
  double rigid_rmse) const
{
  if (!valid_rigid_transform(current_camera_from_previous_camera) ||
    !std::isfinite(elapsed_seconds) ||
    elapsed_seconds <= 0.0 ||
    elapsed_seconds > config_.maximum_sample_interval)
  {
    return std::nullopt;
  }

  const cv::Matx44d previous_camera_from_current_camera =
    invert_rigid_transform(current_camera_from_previous_camera);
  const cv::Matx44d previous_base_from_current_base =
    config_.base_from_camera *
    previous_camera_from_current_camera *
    camera_from_base_;
  if (!valid_rigid_transform(previous_base_from_current_base)) {
    return std::nullopt;
  }

  const double translation =
    cv::norm(translation_part(previous_base_from_current_base));
  const cv::Matx33d rotation = rotation_part(previous_base_from_current_base);
  const double cosine =
    std::clamp((cv::trace(cv::Mat(rotation))[0] - 1.0) / 2.0, -1.0, 1.0);
  const double angle = std::acos(cosine);
  const double linear_speed = translation / elapsed_seconds;
  const double angular_speed = angle / elapsed_seconds;
  if (!std::isfinite(linear_speed) ||
    !std::isfinite(angular_speed) ||
    linear_speed > config_.maximum_linear_speed ||
    angular_speed > config_.maximum_angular_speed)
  {
    return std::nullopt;
  }

  return MotionEstimate{
    linear_speed,
    angular_speed,
    correspondence_count,
    inlier_count,
    rigid_rmse,
    previous_base_from_current_base
  };
}

cv::Matx44d make_rigid_transform(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const cv::Matx33d rotation(
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr);
  return compose_transform(rotation, cv::Vec3d(x, y, z));
}

cv::Matx44d invert_rigid_transform(const cv::Matx44d & transform)
{
  const cv::Matx33d inverse_rotation = rotation_part(transform).t();
  const cv::Vec3d inverse_translation =
    -(inverse_rotation * translation_part(transform));
  return compose_transform(inverse_rotation, inverse_translation);
}

}  // namespace traffic_police
