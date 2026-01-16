#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace calibration {

enum class CameraModel { kPinhole, kFisheye };

// 标定结果结构体：包含内参、畸变、误差等关键输出。
struct CalibrationResult {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double rms = 0.0;
  double mean_reprojection_error = 0.0;
  cv::Size image_size = {};
  CameraModel model = CameraModel::kPinhole;
};

// 标定器：负责管理样本与执行针孔/鱼眼标定流程。
class Calibrator {
 public:
  Calibrator();

  // 清空样本并重置图像尺寸。
  void reset();
  // 追加一帧样本，内部生成对应三维棋盘角点。
  void addSample(const std::vector<cv::Point2f>& corners,
                 const cv::Size& board_size, float square_size,
                 const cv::Size& image_size);

  // 当前样本数量。
  int sampleCount() const;

  // 执行针孔标定。
  CalibrationResult calibratePinhole() const;
  // 执行鱼眼标定。
  CalibrationResult calibrateFisheye() const;

  // 将结果保存为 YAML。
  bool saveYaml(const std::string& path,
                const CalibrationResult& result) const;

 private:
  // 生成棋盘三维角点坐标（Z=0 平面）。
  std::vector<cv::Point3f> createObjectPoints(const cv::Size& board_size,
                                              float square_size) const;
  // 计算平均重投影误差。
  double computeReprojectionError(const CalibrationResult& result,
                                  const std::vector<cv::Mat>& rvecs,
                                  const std::vector<cv::Mat>& tvecs) const;

  // 3D/2D 角点集合。
  std::vector<std::vector<cv::Point3f>> object_points_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  // 所有样本应保持统一图像尺寸。
  cv::Size image_size_ = {};
};

}  // namespace calibration
