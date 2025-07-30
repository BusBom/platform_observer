#include <opencv2/opencv.hpp>
#include <vector>
#include "checker.hpp"

void check_bus_platform(const std::vector<cv::Mat>& platform_masks,
                        bool* status, double min_white_ratio, double exit_gate_ratio) {
  for (size_t i = 0; i < platform_masks.size(); ++i) {
    const cv::Mat& mask = platform_masks[i];
    if (mask.empty()) {
      status[i] = false;
      continue;
    }

    cv::Mat binary;
    if (mask.channels() == 3) {
      cv::cvtColor(mask, binary, cv::COLOR_BGR2GRAY);
    } else {
      binary = mask;
    }
    cv::threshold(binary, binary, 200, 255, cv::THRESH_BINARY);

    int white_count = cv::countNonZero(binary);
    int total_pixels = binary.rows * binary.cols;
    double ratio =
        (total_pixels > 0) ? (double)white_count / total_pixels : 0.0;

    // 출구 영역(마지막에서 두 번째)에 대해서는 더 낮은 임계값 사용
    double threshold = (i == platform_masks.size() - 2) ? exit_gate_ratio : min_white_ratio;
    status[i] = (ratio >= threshold);
  }
}

void check_bus_platform_with_ratios(const std::vector<cv::Mat>& platform_masks,
                                   bool* status, PixelRatioInfo& ratio_info,
                                   double min_white_ratio, double exit_gate_ratio) {
  ratio_info.ratios.clear();
  ratio_info.thresholds.clear();
  ratio_info.results.clear();

  for (size_t i = 0; i < platform_masks.size(); ++i) {
    const cv::Mat& mask = platform_masks[i];
    if (mask.empty()) {
      status[i] = false;
      ratio_info.ratios.push_back(0.0);
      ratio_info.thresholds.push_back(0.0);
      ratio_info.results.push_back(false);
      continue;
    }

    cv::Mat binary;
    if (mask.channels() == 3) {
      cv::cvtColor(mask, binary, cv::COLOR_BGR2GRAY);
    } else {
      binary = mask;
    }
    cv::threshold(binary, binary, 200, 255, cv::THRESH_BINARY);

    int white_count = cv::countNonZero(binary);
    int total_pixels = binary.rows * binary.cols;
    double ratio =
        (total_pixels > 0) ? (double)white_count / total_pixels : 0.0;

    // 출구 영역(마지막에서 두 번째)에 대해서는 더 낮은 임계값 사용
    double threshold = (i == platform_masks.size() - 2) ? exit_gate_ratio : min_white_ratio;
    bool result = (ratio >= threshold);
    
    status[i] = result;
    
    // 비율 정보 저장
    ratio_info.ratios.push_back(ratio);
    ratio_info.thresholds.push_back(threshold);
    ratio_info.results.push_back(result);
  }
}