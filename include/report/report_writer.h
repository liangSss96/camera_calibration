#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "calibration/calibrator.h"
#include "calibration/frame_selector.h"

namespace report {

// 采集会话统计，用于报告输出。
struct SessionStats {
  int total_frames = 0;
  int detected_frames = 0;
  int accepted_frames = 0;
  int no_chessboard_frames = 0;
  int timeout_seconds = 0;
};

// 生成 Markdown 报告，便于审计与复盘。
bool WriteMarkdownReport(const std::string& path,
                         const calibration::CalibrationResult& result,
                         const calibration::FrameSelector& selector,
                         const SessionStats& stats,
                         const std::string& camera_name);

}  // namespace report
