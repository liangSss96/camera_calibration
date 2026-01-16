#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "config/app_config.h"
#include "calibration/calibrator.h"
#include "calibration/frame_selector.h"
#include "report/report_writer.h"

namespace {

struct Options {
  std::string config_path;
  int device = 0;
  cv::Size board_size = {9, 6};
  float square_size = 25.0f;
  std::string model = "pinhole";
  std::string output = "calibration.yaml";
  bool auto_calibrate = true;
  bool show_help = false;
  bool invalid_args = false;
};

void PrintUsage() {
  std::cout << "Camera Calibration Tool (OpenCV)\n"
            << "Usage:\n"
            << "  camera_calibration.exe [options]\n\n"
            << "Options:\n"
            << "  --config <path>       YAML config file\n"
            << "  --device <index>      Camera index (default: 0)\n"
            << "  --cols <int>          Board corners in columns (default: 9)\n"
            << "  --rows <int>          Board corners in rows (default: 6)\n"
            << "  --square <float>      Square size in mm (default: 25)\n"
            << "  --model <pinhole|fisheye>\n"
            << "  --output <path>       Output YAML path\n"
            << "  --manual              Manual save with key 's'\n"
            << "  --help                Show this help\n\n"
            << "Keys:\n"
            << "  q / Esc  Quit\n"
            << "  r        Reset samples\n"
            << "  s        Save when coverage is ready\n";
}

Options ParseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&](std::string* out) {
      if (i + 1 < argc) {
        *out = argv[++i];
        return true;
      }
      return false;
    };
    if (arg == "--config") {
      std::string value;
      if (next(&value)) opt.config_path = value;
      else opt.invalid_args = true;
    } else if (arg == "--device") {
      std::string value;
      if (next(&value)) opt.device = std::stoi(value);
      else opt.invalid_args = true;
    } else if (arg == "--cols") {
      std::string value;
      if (next(&value)) opt.board_size.width = std::stoi(value);
      else opt.invalid_args = true;
    } else if (arg == "--rows") {
      std::string value;
      if (next(&value)) opt.board_size.height = std::stoi(value);
      else opt.invalid_args = true;
    } else if (arg == "--square") {
      std::string value;
      if (next(&value)) opt.square_size = std::stof(value);
      else opt.invalid_args = true;
    } else if (arg == "--model") {
      std::string value;
      if (next(&value)) opt.model = value;
      else opt.invalid_args = true;
    } else if (arg == "--output") {
      std::string value;
      if (next(&value)) opt.output = value;
      else opt.invalid_args = true;
    } else if (arg == "--manual") {
      opt.auto_calibrate = false;
    } else if (arg == "--help" || arg == "-h") {
      opt.show_help = true;
    } else {
      opt.invalid_args = true;
    }
  }
  return opt;
}

// 计算检测前后角点的平均位移，作为亚像素优化效果的质量参考。
//
// 参数：
//   before - 优化前的角点坐标
//   after  - 优化后的角点坐标
// 返回：
//   所有角点的平均像素间距（越小越好）
double MeanCornerShift(const std::vector<cv::Point2f>& before,
                       const std::vector<cv::Point2f>& after) {
  if (before.size() != after.size() || before.empty()) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < before.size(); ++i) {
    sum += cv::norm(before[i] - after[i]);
  }
  return sum / static_cast<double>(before.size());
}


// 在帧上绘制标注信息的辅助函数。
// 参数：
//   frame      - 当前采集的视频帧（将直接在其上绘制）
//   decision   - 当前帧的评估结果（含各类指标与采样理由）
//   samples    - 已采集的样本数
//   sufficient - 是否已满足标定的足够覆盖度（true 时提示"ready"）
void DrawOverlay(cv::Mat& frame, const calibration::FrameDecision& decision,
                 int samples, bool sufficient) {
  int y = 24;
  // 辅助 lambda：在下一行绘制一行文字，并向下移动 y 坐标。
  auto line = [&](const std::string& text) {
    cv::putText(frame, text, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    y += 22;
  };

  // 绘制已采样帧数
  line("samples: " + std::to_string(samples));
  // 绘制当前帧的 area（覆盖比例）
  line("area: " + std::to_string(decision.metrics.area_ratio).substr(0, 5));
  // 绘制棋盘倾斜角
  line("tilt: " + std::to_string(decision.metrics.tilt).substr(0, 5));
  // 绘制清晰度分数
  line("sharp: " + std::to_string(decision.metrics.sharpness).substr(0, 6));
  // 绘制角点质量分数
  line("cornerQ: " +
       std::to_string(decision.metrics.corner_quality).substr(0, 5));
  // 显示帧的当前状态（如被拒绝原因或采样状态）
  line("status: " + decision.reason);
  // 如果覆盖度足够，额外提示 ready
  if (sufficient) {
    line("coverage: ready");
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options opt = ParseArgs(argc, argv);
  if (opt.show_help) {
    PrintUsage();
    return 0;
  }
  if (opt.invalid_args) {
    std::cerr << "Invalid arguments.\n";
    PrintUsage();
    return 1;
  }
  // 默认单机模式配置，若提供 --config 则使用配置文件。
  config::AppConfig app_cfg;
  config::CameraConfig cam_cfg;
  if (!opt.config_path.empty()) {
    if (!config::LoadConfig(opt.config_path, &app_cfg)) {
      std::cerr << "Failed to load config: " << opt.config_path << "\n";
      return 1;
    }
    if (app_cfg.cameras.empty()) {
      std::cerr << "No camera configuration found.\n";
      return 1;
    }
    cam_cfg = app_cfg.cameras.front();
  } else {
    if (opt.board_size.width <= 0 || opt.board_size.height <= 0 ||
        opt.square_size <= 0.0f) {
      std::cerr << "Invalid board size or square size.\n";
      return 1;
    }
    if (opt.model != "pinhole" && opt.model != "fisheye") {
      std::cerr << "Unsupported model: " << opt.model << "\n";
      return 1;
    }
    cam_cfg.device = opt.device;
    cam_cfg.board_size = opt.board_size;
    cam_cfg.square_size = opt.square_size;
    cam_cfg.model = opt.model;
    cam_cfg.output = opt.output;
    cam_cfg.auto_calibrate = opt.auto_calibrate;
  }

  // 确保输出目录存在。
  if (!app_cfg.output_dir.empty()) {
    std::filesystem::create_directories(app_cfg.output_dir);
  }
  if (!app_cfg.report_dir.empty()) {
    std::filesystem::create_directories(app_cfg.report_dir);
  }

  // 打开相机设备并进行基本检查。
  cv::VideoCapture cap(cam_cfg.device);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open camera device." << std::endl;
    return 1;
  }

  // 采样器：根据质量指标决定是否收集该帧。
  calibration::FrameSelector selector(cam_cfg.selector);
  // 标定器：保存全部样本并执行标定。
  calibration::Calibrator calibrator;
  report::SessionStats stats;

  const cv::Size board_size = cam_cfg.board_size;
  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  const auto start = std::chrono::steady_clock::now();

  std::cout << "Press 'q' to quit, 'r' to reset, 's' to save if ready."
            << std::endl;

  bool calibrated = false;
  while (true) {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) continue;
    stats.total_frames++;

    // 超时/帧数限制：用于避免长时间阻塞。
    if (app_cfg.max_frames > 0 && stats.total_frames >= app_cfg.max_frames) {
      break;
    }
    if (app_cfg.max_seconds > 0) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start);
      if (elapsed.count() >= app_cfg.max_seconds) {
        stats.timeout_seconds = static_cast<int>(elapsed.count());
        break;
      }
    }

    // 灰度化以提升角点检测速度与稳定性。
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, board_size, corners, flags);

    // 为叠加显示初始化默认状态。
    calibration::FrameDecision decision;
    decision.metrics.image_size = frame.size();
    decision.reason = "no_chessboard";
    if (found) {
      stats.detected_frames++;
      // 角点亚像素优化，并计算优化前后位移作为质量参考。
      std::vector<cv::Point2f> corners_before = corners;
      cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
                       {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                        0.001});
      double mean_shift = MeanCornerShift(corners_before, corners);
      auto metrics = calibration::ComputeFrameMetrics(gray, corners, mean_shift);
      // 根据质量指标判断是否收集该帧。
      decision = selector.evaluate(metrics);
      if (decision.accept) {
        stats.accepted_frames++;
        calibrator.addSample(corners, board_size, cam_cfg.square_size,
                             frame.size());
      }
      cv::drawChessboardCorners(frame, board_size, corners, found);
    } else {
      stats.no_chessboard_frames++;
    }

    // 当覆盖度达到要求时，可触发自动标定。
    bool sufficient = selector.isSufficient();
    DrawOverlay(frame, decision, calibrator.sampleCount(), sufficient);

    if (cam_cfg.auto_calibrate && sufficient && !calibrated) {
      calibration::CalibrationResult result;
      if (cam_cfg.model == "fisheye") {
        result = calibrator.calibrateFisheye();
      } else {
        result = calibrator.calibratePinhole();
      }
      if (calibrator.saveYaml(cam_cfg.output, result)) {
        std::cout << "Calibration saved: " << cam_cfg.output << std::endl;
        std::cout << "RMS: " << result.rms
                  << " Mean reprojection error: "
                  << result.mean_reprojection_error << std::endl;
        report::WriteMarkdownReport(cam_cfg.report, result, selector, stats,
                                    cam_cfg.name);
      }
      calibrated = true;
    }

    cv::imshow("camera calibration", frame);
    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      break;
    }
    if (key == 'r') {
      // 重置所有采样，重新开始收集。
      selector.reset();
      calibrator.reset();
      stats = report::SessionStats{};
      calibrated = false;
      std::cout << "Reset samples." << std::endl;
    }
    if (key == 's' && sufficient) {
      // 手动触发保存，适用于 manual 模式。
      calibration::CalibrationResult result;
      if (cam_cfg.model == "fisheye") {
        result = calibrator.calibrateFisheye();
      } else {
        result = calibrator.calibratePinhole();
      }
      if (calibrator.saveYaml(cam_cfg.output, result)) {
        std::cout << "Calibration saved: " << cam_cfg.output << std::endl;
        std::cout << "RMS: " << result.rms
                  << " Mean reprojection error: "
                  << result.mean_reprojection_error << std::endl;
        report::WriteMarkdownReport(cam_cfg.report, result, selector, stats,
                                    cam_cfg.name);
      }
      calibrated = true;
    }
  }

  return 0;
}
