#ifndef CHECKER_HPP
#define CHECKER_HPP

#include <opencv2/opencv.hpp>

void check_bus_platform(const std::vector<cv::Mat>& platform_masks,
                        bool* status, double min_white_ratio = 0.35);

#endif