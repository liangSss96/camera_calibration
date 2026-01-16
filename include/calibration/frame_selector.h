#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace calibration {

// 单帧质量指标。
struct FrameMetrics {
  double area_ratio = 0.0;
  cv::Point2f center = {};
  double tilt = 0.0;
  double sharpness = 0.0;
  double corner_quality = 0.0;
  cv::Size image_size = {};
};

// 采样判定配置，面向“覆盖度 + 质量阈值”。
struct FrameSelectorConfig {
  double min_area_ratio = 0.03;
  double max_area_ratio = 0.60;
  double min_sharpness = 80.0;
  double min_corner_quality = 0.25;

  // 与已有样本距离/差异阈值，用于过滤重复样本。
  double min_center_distance_ratio = 0.06;
  double min_area_diff_ratio = 0.02;
  double min_tilt_diff = 0.05;

  // 分桶覆盖限制与最小样本要求。
  int max_per_bucket = 6;
  int min_total_samples = 20;
  int min_position_buckets = 6;
  int min_scale_buckets = 3;
  int min_tilt_buckets = 3;
};

// 采样结果与原因说明，便于可视化与调参。
struct FrameDecision {
  bool accept = false;
  std::string reason;
  FrameMetrics metrics;
};

// 采样器：根据质量指标与覆盖度判断是否采样该帧。
class FrameSelector {
 public:
  explicit FrameSelector(const FrameSelectorConfig& config = {});

  // 清空历史样本与覆盖度统计。
  void reset();
  // 评估并决定是否接收该帧。
  FrameDecision evaluate(const FrameMetrics& metrics);
  // 覆盖度是否满足标定触发条件。
  bool isSufficient() const;
  // 已接收样本数量。
  int acceptedCount() const;

  // 读取当前配置。
  const FrameSelectorConfig& config() const;
  // 读取覆盖桶统计，用于报告。
  const std::vector<int>& scaleCounts() const;
  const std::vector<int>& positionCounts() const;
  const std::vector<int>& tiltCounts() const;
  // 读取拒绝原因统计，用于报告。
  const std::unordered_map<std::string, int>& reasonCounts() const;

 private:
  // 最小化保存的样本信息，用于相似性判定与分桶统计。
  struct SampleInfo {
    double area_ratio = 0.0;
    cv::Point2f center = {};
    double tilt = 0.0;
    cv::Size image_size = {};
  };

  // 面积占比分桶（小/中/大）。
  int scaleBucket(double area_ratio) const;
  // 位置分桶（3x3 网格）。
  int positionBucket(const cv::Point2f& center, const cv::Size& image_size) const;
  // 倾斜度分桶。
  int tiltBucket(double tilt) const;
  // 与已有样本是否过于相似。
  bool isSimilarToExisting(const SampleInfo& sample) const;
  // 当前样本是否为覆盖度带来增量。
  bool contributes(int scale_idx, int pos_idx, int tilt_idx) const;
  // 统计已覆盖桶数量。
  int coveredBuckets(const std::vector<int>& counts) const;

  FrameSelectorConfig config_;
  std::vector<SampleInfo> samples_;
  std::vector<int> scale_counts_;
  std::vector<int> position_counts_;
  std::vector<int> tilt_counts_;
  std::unordered_map<std::string, int> reason_counts_;
};

// 计算单帧质量指标：面积、中心、倾斜、清晰度与角点质量。
FrameMetrics ComputeFrameMetrics(
    const cv::Mat& gray,
    const std::vector<cv::Point2f>& corners,
    double mean_corner_shift);

}  // namespace calibration
