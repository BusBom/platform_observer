#include <opencv2/opencv.hpp>
#include <vector>

void check_bus_platform(const std::vector<cv::Mat>& platform_masks, 
    bool* status, double min_white_ratio) {
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
        double ratio = (total_pixels > 0) ? (double)white_count / total_pixels : 0.0;

        status[i] = (ratio >= min_white_ratio);
    }
}