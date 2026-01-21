#include "config/app_config.h"

#include <opencv2/core.hpp>

namespace config {

namespace {

// 读取字符串字段，若缺失则返回默认值。
std::string GetString(const cv::FileNode& node, const std::string& key,
                      const std::string& default_value) {
  cv::FileNode child = node[key];
  if (child.empty()) return default_value;
  std::string value;
  child >> value;
  return value;
}

// 读取整数字段，若缺失则返回默认值。
int GetInt(const cv::FileNode& node, const std::string& key, int default_value) {
  cv::FileNode child = node[key];
  if (child.empty()) return default_value;
  int value = default_value;
  child >> value;
  return value;
}

// 读取浮点字段，若缺失则返回默认值。
double GetDouble(const cv::FileNode& node, const std::string& key,
                 double default_value) {
  cv::FileNode child = node[key];
  if (child.empty()) return default_value;
  double value = default_value;
  child >> value;
  return value;
}

// 读取布尔字段，OpenCV FileStorage 使用 int 表示。
bool GetBool(const cv::FileNode& node, const std::string& key,
             bool default_value) {
  cv::FileNode child = node[key];
  if (child.empty()) return default_value;
  int value = default_value ? 1 : 0;
  child >> value;
  return value != 0;
}

// 读取帧筛选阈值配置。
calibration::FrameSelectorConfig ReadSelectorConfig(const cv::FileNode& node) {
  calibration::FrameSelectorConfig cfg;
  if (node.empty()) return cfg;
  cfg.min_area_ratio = GetDouble(node, "min_area_ratio", cfg.min_area_ratio);
  cfg.max_area_ratio = GetDouble(node, "max_area_ratio", cfg.max_area_ratio);
  cfg.min_sharpness = GetDouble(node, "min_sharpness", cfg.min_sharpness);
  cfg.min_corner_quality =
      GetDouble(node, "min_corner_quality", cfg.min_corner_quality);
  cfg.min_center_distance_ratio =
      GetDouble(node, "min_center_distance_ratio",
                cfg.min_center_distance_ratio);
  cfg.min_area_diff_ratio =
      GetDouble(node, "min_area_diff_ratio", cfg.min_area_diff_ratio);
  cfg.min_tilt_diff = GetDouble(node, "min_tilt_diff", cfg.min_tilt_diff);
  cfg.max_per_bucket = GetInt(node, "max_per_bucket", cfg.max_per_bucket);
  cfg.min_total_samples =
      GetInt(node, "min_total_samples", cfg.min_total_samples);
  cfg.min_position_buckets =
      GetInt(node, "min_position_buckets", cfg.min_position_buckets);
  cfg.min_scale_buckets =
      GetInt(node, "min_scale_buckets", cfg.min_scale_buckets);
  cfg.min_tilt_buckets =
      GetInt(node, "min_tilt_buckets", cfg.min_tilt_buckets);
  return cfg;
}

// 读取单路相机配置，并应用全局输出目录。
CameraConfig ReadCameraConfig(const cv::FileNode& node,
                              const AppConfig& app_defaults) {
  CameraConfig cam;
  cam.name = GetString(node, "name", cam.name);
  cam.device = GetInt(node, "device", cam.device);
  cam.input_path = GetString(node, "input_path", cam.input_path);
  cam.input_type = GetString(node, "input_type", cam.input_type);
  cam.board_size.width = GetInt(node, "board_cols", cam.board_size.width);
  cam.board_size.height = GetInt(node, "board_rows", cam.board_size.height);
  cam.square_size =
      static_cast<float>(GetDouble(node, "square_size", cam.square_size));
  cam.model = GetString(node, "model", cam.model);
  cam.output = GetString(node, "output", cam.output);
  cam.report = GetString(node, "report", cam.report);
  cam.auto_calibrate = GetBool(node, "auto_calibrate", cam.auto_calibrate);
  cam.selector = ReadSelectorConfig(node["frame_selector"]);

  // 若输出路径未包含目录，则拼接全局目录。
  if (!app_defaults.output_dir.empty() &&
      cam.output.find('\\') == std::string::npos &&
      cam.output.find('/') == std::string::npos) {
    cam.output = app_defaults.output_dir + "/" + cam.output;
  }
  if (!app_defaults.report_dir.empty() &&
      cam.report.find('\\') == std::string::npos &&
      cam.report.find('/') == std::string::npos) {
    cam.report = app_defaults.report_dir + "/" + cam.report;
  }
  return cam;
}

}  // namespace

bool LoadConfig(const std::string& path, AppConfig* out_config) {
  if (!out_config) return false;
  AppConfig cfg;

  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) return false;

  cv::FileNode app_node = fs["app"];
  cfg.show_window = GetBool(app_node, "show_window", cfg.show_window);
  cfg.max_seconds = GetInt(app_node, "max_seconds", cfg.max_seconds);
  cfg.max_frames = GetInt(app_node, "max_frames", cfg.max_frames);
  cfg.output_dir = GetString(app_node, "output_dir", cfg.output_dir);
  cfg.report_dir = GetString(app_node, "report_dir", cfg.report_dir);

  cv::FileNode cameras_node = fs["cameras"];
  if (cameras_node.type() == cv::FileNode::SEQ) {
    for (auto it = cameras_node.begin(); it != cameras_node.end(); ++it) {
      cfg.cameras.push_back(ReadCameraConfig(*it, cfg));
    }
  }

  *out_config = cfg;
  return true;
}

}  // namespace config
