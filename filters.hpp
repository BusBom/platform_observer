#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <opencv2/opencv.hpp>

void auto_brightness_balance(cv::Mat &bgr_image, cv::Mat& dst_image);

void warp_rectified_areas(cv::Mat &bgr_image, std::vector<cv::Mat> &dst_image,
                         std::vector<std::vector<cv::Point>> &rect); //LT->RT->RB->LB
bool is_rectified_area_changed(std::vector<cv::Point> &rect, int p_index);

void remove_achromatic_areas(std::vector<cv::Mat> &bgrImages, std::vector<cv::Mat> &dst_masks, 
    float threshold_ach = 0.15);
void remove_achromatic_area(cv::Mat &bgrImage, cv::Mat &dst_mask,
                            int index, float threshold_ach = 0.15);
bool is_achromatic(const cv::Vec3b &bgr, float threshold);

void revive_white_areas(std::vector<cv::Mat> &bgrImages, std::vector<cv::Mat> &dst_masks, 
    int threshold_br = 70);
void revive_white_area(cv::Mat &src_mask, cv::Mat &dst_mask, 
    int index, int threshold_br = 70);

#endif