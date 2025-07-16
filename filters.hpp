#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <opencv2/opencv.hpp>

void warp_rectified_area(cv::Mat &bgr_image, cv::Mat& dst_image, std::vector<cv::Point>& rect); //LT->RT->RB->LB
bool is_rectified_area_changed(std::vector<cv::Point>& rect);
void remove_achromatic_area(cv::Mat &bgrImage, cv::Mat &dst_mask, float threshold_ach = 0.15f);
bool is_achromatic(const cv::Vec3b& bgr, float threshold);
void revive_white_area(cv::Mat &src_mask, cv::Mat &dst_mask, int threshold_br = 95);

#endif