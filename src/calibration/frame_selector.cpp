#include "calibration/frame_selector.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace calibration {

FrameSelector::FrameSelector(const FrameSelectorConfig& config)
    : config_(config),
      scale_counts_(3, 0),
      position_counts_(9, 0),
      tilt_counts_(3, 0) {}

void FrameSelector::reset() {
  // 重置样本与覆盖度计数。
  samples_.clear();
  std::fill(scale_counts_.begin(), scale_counts_.end(), 0);
  std::fill(position_counts_.begin(), position_counts_.end(), 0);
  std::fill(tilt_counts_.begin(), tilt_counts_.end(), 0);
  reason_counts_.clear();
}

FrameDecision FrameSelector::evaluate(const FrameMetrics& metrics) {
  FrameDecision decision;
  decision.metrics = metrics;
  auto setReason = [&](const std::string& reason) {
    decision.reason = reason;
    reason_counts_[reason]++;
  };

  // 质量阈值过滤：面积、清晰度、角点质量。
  if (metrics.area_ratio < config_.min_area_ratio ||
      metrics.area_ratio > config_.max_area_ratio) {
    setReason("area_ratio_out_of_range");
    return decision;
  }
  if (metrics.sharpness < config_.min_sharpness) {
    setReason("too_blurry");
    return decision;
  }
  if (metrics.corner_quality < config_.min_corner_quality) {
    setReason("corner_quality_low");
    return decision;
  }

  // 分桶索引用于覆盖度统计。
  SampleInfo sample{metrics.area_ratio, metrics.center, metrics.tilt,
                    metrics.image_size};
  int scale_idx = scaleBucket(metrics.area_ratio);
  int pos_idx = positionBucket(metrics.center, metrics.image_size);
  int tilt_idx = tiltBucket(metrics.tilt);

  if (scale_idx < 0 || pos_idx < 0 || tilt_idx < 0) {
    setReason("bucket_invalid");
    return decision;
  }

  // 如果对覆盖度无明显增量且与已有样本过于相似，则丢弃。
  if (!contributes(scale_idx, pos_idx, tilt_idx) &&
      isSimilarToExisting(sample)) {
    setReason("insufficient_contribution");
    return decision;
  }

  // 分桶已满且样本相似时丢弃，避免过度集中。
  if (isSimilarToExisting(sample) &&
      scale_counts_[scale_idx] >= config_.max_per_bucket &&
      position_counts_[pos_idx] >= config_.max_per_bucket &&
      tilt_counts_[tilt_idx] >= config_.max_per_bucket) {
    setReason("bucket_full");
    return decision;
  }

  samples_.push_back(sample);
  scale_counts_[scale_idx]++;
  position_counts_[pos_idx]++;
  tilt_counts_[tilt_idx]++;

  decision.accept = true;
  setReason("accepted");
  return decision;
}

bool FrameSelector::isSufficient() const {
  // 覆盖度条件：总样本数与各维度桶覆盖数。
  if (static_cast<int>(samples_.size()) < config_.min_total_samples) {
    return false;
  }
  if (coveredBuckets(position_counts_) < config_.min_position_buckets) {
    return false;
  }
  if (coveredBuckets(scale_counts_) < config_.min_scale_buckets) {
    return false;
  }
  if (coveredBuckets(tilt_counts_) < config_.min_tilt_buckets) {
    return false;
  }
  return true;
}

int FrameSelector::acceptedCount() const { return static_cast<int>(samples_.size()); }

const FrameSelectorConfig& FrameSelector::config() const { return config_; }

const std::vector<int>& FrameSelector::scaleCounts() const {
  return scale_counts_;
}

const std::vector<int>& FrameSelector::positionCounts() const {
  return position_counts_;
}

const std::vector<int>& FrameSelector::tiltCounts() const { return tilt_counts_; }

const std::unordered_map<std::string, int>& FrameSelector::reasonCounts() const {
  return reason_counts_;
}

int FrameSelector::scaleBucket(double area_ratio) const {
  // 简单三分桶：小/中/大。
  if (area_ratio < 0.12) return 0;
  if (area_ratio < 0.28) return 1;
  return 2;
}

int FrameSelector::positionBucket(const cv::Point2f& center,
                                  const cv::Size& image_size) const {
  // 画面划分 3x3 网格。
  if (image_size.width <= 0 || image_size.height <= 0) return -1;
  int x_bucket = std::clamp(static_cast<int>(center.x / image_size.width * 3), 0, 2);
  int y_bucket =
      std::clamp(static_cast<int>(center.y / image_size.height * 3), 0, 2);
  return y_bucket * 3 + x_bucket;
}

int FrameSelector::tiltBucket(double tilt) const {
  // 倾斜度分桶：小/中/大。
  if (tilt < 0.2) return 0;
  if (tilt < 0.45) return 1;
  return 2;
}

bool FrameSelector::isSimilarToExisting(const SampleInfo& sample) const {
  if (samples_.empty()) return false;
  if (sample.image_size.width <= 0 || sample.image_size.height <= 0) {
    return false;
  }
  // 位置差异按图像尺寸归一化，避免分辨率影响阈值。
  const double min_center_dist =
      config_.min_center_distance_ratio *
      std::min(sample.image_size.width, sample.image_size.height);
  for (const auto& existing : samples_) {
    double center_dist = cv::norm(existing.center - sample.center);
    if (center_dist < min_center_dist &&
        std::abs(existing.area_ratio - sample.area_ratio) <
            config_.min_area_diff_ratio &&
        std::abs(existing.tilt - sample.tilt) < config_.min_tilt_diff) {
      return true;
    }
  }
  return false;
}

bool FrameSelector::contributes(int scale_idx, int pos_idx, int tilt_idx) const {
  // 任一维度未满桶即视为有覆盖增量。
  return scale_counts_[scale_idx] < config_.max_per_bucket ||
         position_counts_[pos_idx] < config_.max_per_bucket ||
         tilt_counts_[tilt_idx] < config_.max_per_bucket;
}

int FrameSelector::coveredBuckets(const std::vector<int>& counts) const {
  // 统计已命中的桶数量。
  return static_cast<int>(
      std::count_if(counts.begin(), counts.end(), [](int v) { return v > 0; }));
}

FrameMetrics ComputeFrameMetrics(const cv::Mat& gray,
                                 const std::vector<cv::Point2f>& corners,
                                 double mean_corner_shift) {
  FrameMetrics metrics;
  metrics.image_size = gray.size();

  if (corners.empty() || gray.empty()) {
    return metrics;
  }

  // 棋盘外接矩形用于估算面积与中心。
  cv::Rect bounds = cv::boundingRect(corners);
  metrics.area_ratio = static_cast<double>(bounds.area()) /
                       static_cast<double>(gray.cols * gray.rows);
  metrics.center = cv::Point2f(bounds.x + bounds.width * 0.5f,
                               bounds.y + bounds.height * 0.5f);

  if (corners.size() >= 4) {
    // 通过与外接矩形四角最近的点估算棋盘四角，计算倾斜度。
    cv::Point2f tl = corners.front();
    cv::Point2f tr = corners.back();
    cv::Point2f bl = corners.front();
    cv::Point2f br = corners.back();
    auto updateCorner = [&](const cv::Point2f& corner) {
      if (cv::norm(corner - cv::Point2f(bounds.x, bounds.y)) <
          cv::norm(tl - cv::Point2f(bounds.x, bounds.y))) {
        tl = corner;
      }
      if (cv::norm(corner - cv::Point2f(bounds.x + bounds.width, bounds.y)) <
          cv::norm(tr - cv::Point2f(bounds.x + bounds.width, bounds.y))) {
        tr = corner;
      }
      if (cv::norm(corner - cv::Point2f(bounds.x, bounds.y + bounds.height)) <
          cv::norm(bl - cv::Point2f(bounds.x, bounds.y + bounds.height))) {
        bl = corner;
      }
      if (cv::norm(corner - cv::Point2f(bounds.x + bounds.width,
                                        bounds.y + bounds.height)) <
          cv::norm(br - cv::Point2f(bounds.x + bounds.width,
                                    bounds.y + bounds.height))) {
        br = corner;
      }
    };
    for (const auto& corner : corners) {
      updateCorner(corner);
    }
    double edge1 = cv::norm(tr - tl);
    double edge2 = cv::norm(br - tr);
    double edge3 = cv::norm(bl - br);
    double edge4 = cv::norm(tl - bl);
    double min_edge = std::min({edge1, edge2, edge3, edge4});
    double max_edge = std::max({edge1, edge2, edge3, edge4});
    if (max_edge > 1e-6) {
      metrics.tilt = 1.0 - (min_edge / max_edge);
    }
  }

  // Laplacian 方差作为清晰度指标。
  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  metrics.sharpness = stddev[0] * stddev[0];

  // 角点亚像素位移越小，质量越高。
  metrics.corner_quality = 1.0 / (1.0 + mean_corner_shift);

  return metrics;
}

}  // namespace calibration
