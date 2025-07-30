#ifndef CHECKER_HPP
#define CHECKER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

void check_bus_platform(const std::vector<cv::Mat>& platform_masks, bool* status, 
    double min_white_ratio = 0.3, double exit_gate_ratio = 0.15);

// 픽셀 비율 정보를 저장하는 구조체
struct PixelRatioInfo {
    std::vector<double> ratios;
    std::vector<double> thresholds;
    std::vector<bool> results;
};

void check_bus_platform_with_ratios(const std::vector<cv::Mat>& platform_masks, 
    bool* status, PixelRatioInfo& ratio_info, 
    double min_white_ratio = 0.3, double exit_gate_ratio = 0.15);

#endif