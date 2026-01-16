#include "report/report_writer.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace report {

namespace {

// 将矩阵格式化为多行文本，方便报告展示。
std::string MatToString(const cv::Mat& mat) {
  if (mat.empty()) return "[]";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  for (int r = 0; r < mat.rows; ++r) {
    oss << "[";
    for (int c = 0; c < mat.cols; ++c) {
      double v = mat.at<double>(r, c);
      oss << v;
      if (c + 1 < mat.cols) oss << ", ";
    }
    oss << "]";
    if (r + 1 < mat.rows) oss << "\n";
  }
  return oss.str();
}

// 打印覆盖桶统计。
std::string JoinCounts(const std::vector<int>& counts) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < counts.size(); ++i) {
    oss << counts[i];
    if (i + 1 < counts.size()) oss << ", ";
  }
  oss << "]";
  return oss.str();
}

// 打印筛帧原因统计。
std::string JoinReasonCounts(
    const std::unordered_map<std::string, int>& reasons) {
  std::ostringstream oss;
  for (const auto& kv : reasons) {
    oss << "- " << kv.first << ": " << kv.second << "\n";
  }
  return oss.str();
}

}  // namespace

bool WriteMarkdownReport(const std::string& path,
                         const calibration::CalibrationResult& result,
                         const calibration::FrameSelector& selector,
                         const SessionStats& stats,
                         const std::string& camera_name) {
  std::ofstream out(path);
  if (!out.is_open()) return false;

  out << "# Calibration Report\n\n";
  out << "Camera: **" << camera_name << "**\n\n";

  out << "## Summary\n";
  out << "- Model: " << (result.model == calibration::CameraModel::kFisheye
                            ? "fisheye"
                            : "pinhole")
      << "\n";
  out << "- Image size: " << result.image_size.width << " x "
      << result.image_size.height << "\n";
  out << "- RMS: " << result.rms << "\n";
  out << "- Mean reprojection error: " << result.mean_reprojection_error
      << "\n\n";

  out << "## Session Stats\n";
  out << "- Total frames: " << stats.total_frames << "\n";
  out << "- Detected frames: " << stats.detected_frames << "\n";
  out << "- Accepted frames: " << stats.accepted_frames << "\n";
  out << "- No chessboard frames: " << stats.no_chessboard_frames << "\n";
  out << "- Timeout seconds: " << stats.timeout_seconds << "\n\n";

  out << "## Coverage Stats\n";
  out << "- Scale buckets: " << JoinCounts(selector.scaleCounts()) << "\n";
  out << "- Position buckets: " << JoinCounts(selector.positionCounts()) << "\n";
  out << "- Tilt buckets: " << JoinCounts(selector.tiltCounts()) << "\n\n";

  out << "## Selector Config\n";
  const auto& cfg = selector.config();
  out << "- min_area_ratio: " << cfg.min_area_ratio << "\n";
  out << "- max_area_ratio: " << cfg.max_area_ratio << "\n";
  out << "- min_sharpness: " << cfg.min_sharpness << "\n";
  out << "- min_corner_quality: " << cfg.min_corner_quality << "\n";
  out << "- min_center_distance_ratio: " << cfg.min_center_distance_ratio
      << "\n";
  out << "- min_area_diff_ratio: " << cfg.min_area_diff_ratio << "\n";
  out << "- min_tilt_diff: " << cfg.min_tilt_diff << "\n";
  out << "- max_per_bucket: " << cfg.max_per_bucket << "\n";
  out << "- min_total_samples: " << cfg.min_total_samples << "\n";
  out << "- min_position_buckets: " << cfg.min_position_buckets << "\n";
  out << "- min_scale_buckets: " << cfg.min_scale_buckets << "\n";
  out << "- min_tilt_buckets: " << cfg.min_tilt_buckets << "\n\n";

  out << "## Reject Reasons\n";
  out << JoinReasonCounts(selector.reasonCounts());
  out << "\n";

  out << "## Camera Matrix\n";
  out << "```\n" << MatToString(result.camera_matrix) << "\n```\n\n";

  out << "## Distortion Coefficients\n";
  out << "```\n" << MatToString(result.dist_coeffs) << "\n```\n\n";

  out.close();
  return true;
}

}  // namespace report
