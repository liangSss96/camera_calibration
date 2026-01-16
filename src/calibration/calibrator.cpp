#include "calibration/calibrator.h"

#include <opencv2/calib3d.hpp>

namespace calibration {

Calibrator::Calibrator() = default;

void Calibrator::reset() {
  // 清空所有历史样本与尺寸信息。
  object_points_.clear();
  image_points_.clear();
  image_size_ = {};
}

void Calibrator::addSample(const std::vector<cv::Point2f>& corners,
                           const cv::Size& board_size, float square_size,
                           const cv::Size& image_size) {
  // 角点为空时直接忽略。
  if (corners.empty()) return;
  // 首次样本记录图像尺寸，后续必须一致。
  if (image_size_.width == 0 || image_size_.height == 0) {
    image_size_ = image_size;
  } else if (image_size_ != image_size) {
    // 尺寸不一致会破坏标定的几何约束，直接丢弃。
    return;
  }
  object_points_.push_back(createObjectPoints(board_size, square_size));
  image_points_.push_back(corners);
}

int Calibrator::sampleCount() const {
  return static_cast<int>(image_points_.size());
}

CalibrationResult Calibrator::calibratePinhole() const {
  CalibrationResult result;
  // 样本不足时直接返回空结果。
  if (image_points_.empty()) return result;

  result.model = CameraModel::kPinhole;
  result.image_size = image_size_;
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

  std::vector<cv::Mat> rvecs, tvecs;
  // 使用 OpenCV 标准针孔模型标定。
  result.rms = cv::calibrateCamera(object_points_, image_points_, image_size_,
                                   result.camera_matrix, result.dist_coeffs,
                                   rvecs, tvecs);
  result.mean_reprojection_error =
      computeReprojectionError(result, rvecs, tvecs);
  return result;
}

CalibrationResult Calibrator::calibrateFisheye() const {
  CalibrationResult result;
  // 样本不足时直接返回空结果。
  if (image_points_.empty()) return result;

  result.model = CameraModel::kFisheye;
  result.image_size = image_size_;
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

  std::vector<cv::Mat> rvecs, tvecs;
  // 鱼眼模型常用标定标志位。
  int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
              cv::fisheye::CALIB_CHECK_COND | cv::fisheye::CALIB_FIX_SKEW;
  result.rms =
      cv::fisheye::calibrate(object_points_, image_points_, image_size_,
                             result.camera_matrix, result.dist_coeffs, rvecs,
                             tvecs, flags);
  result.mean_reprojection_error =
      computeReprojectionError(result, rvecs, tvecs);
  return result;
}

bool Calibrator::saveYaml(const std::string& path,
                          const CalibrationResult& result) const {
  // 输出前检查结果完整性。
  if (result.camera_matrix.empty() || result.dist_coeffs.empty()) {
    return false;
  }

  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if (!fs.isOpened()) return false;

  // YAML 输出结构保持稳定，便于后续解析。
  fs << "model"
     << (result.model == CameraModel::kFisheye ? "fisheye" : "pinhole");
  fs << "image_width" << result.image_size.width;
  fs << "image_height" << result.image_size.height;
  fs << "camera_matrix" << result.camera_matrix;
  fs << "distortion_coefficients" << result.dist_coeffs;
  fs << "rms" << result.rms;
  fs << "mean_reprojection_error" << result.mean_reprojection_error;

  fs.release();
  return true;
}

std::vector<cv::Point3f> Calibrator::createObjectPoints(
    const cv::Size& board_size, float square_size) const {
  // 按行列顺序生成平面棋盘角点（Z=0）。
  std::vector<cv::Point3f> points;
  points.reserve(board_size.width * board_size.height);
  for (int i = 0; i < board_size.height; ++i) {
    for (int j = 0; j < board_size.width; ++j) {
      points.emplace_back(j * square_size, i * square_size, 0.0f);
    }
  }
  return points;
}

double Calibrator::computeReprojectionError(
    const CalibrationResult& result, const std::vector<cv::Mat>& rvecs,
    const std::vector<cv::Mat>& tvecs) const {
  // 统计所有角点的均方根重投影误差。
  double total_error = 0.0;
  size_t total_points = 0;

  for (size_t i = 0; i < object_points_.size(); ++i) {
    std::vector<cv::Point2f> projected;
    if (result.model == CameraModel::kFisheye) {
      // 鱼眼模型投影。
      cv::fisheye::projectPoints(object_points_[i], projected, rvecs[i],
                                 tvecs[i], result.camera_matrix,
                                 result.dist_coeffs);
    } else {
      // 针孔模型投影。
      cv::projectPoints(object_points_[i], rvecs[i], tvecs[i],
                        result.camera_matrix, result.dist_coeffs, projected);
    }

    double err = cv::norm(image_points_[i], projected, cv::NORM_L2);
    size_t n = object_points_[i].size();
    total_error += err * err;
    total_points += n;
  }

  if (total_points == 0) return 0.0;
  return std::sqrt(total_error / total_points);
}

}  // namespace calibration
