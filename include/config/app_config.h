#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "calibration/frame_selector.h"

namespace config {

// 单个相机的标定配置。
// 用于：设备索引、棋盘参数、输出路径、采样阈值等。
struct CameraConfig {
  std::string name = "camera";
  int device = 0;
  std::string input_path;
  std::string input_type = "auto";
  cv::Size board_size = {9, 6};
  float square_size = 25.0f;
  std::string model = "pinhole";
  std::string output = "calibration.yaml";
  std::string report = "report.md";
  bool auto_calibrate = true;
  calibration::FrameSelectorConfig selector{};
};

// 全局应用配置。
// 用于：输出目录、显示窗口、超时控制等。
struct AppConfig {
  bool show_window = true;
  int max_seconds = 0;
  int max_frames = 0;
  std::string output_dir = "outputs";
  std::string report_dir = "reports";
  std::vector<CameraConfig> cameras;
};

// 读取 YAML 配置文件，失败返回 false。
bool LoadConfig(const std::string& path, AppConfig* out_config);

}  // namespace config
