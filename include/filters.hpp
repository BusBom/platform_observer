#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <opencv2/opencv.hpp>

void auto_brightness_balance(cv::Mat &bgr_image, cv::Mat& dst_image);

void warp_rectified_areas(cv::Mat &bgr_image, std::vector<cv::Mat> &dst_image,
                         std::vector<std::vector<cv::Point>> &rect); //LT->RT->RB->LB
bool is_rectified_area_changed(std::vector<cv::Point> &rect, int p_index);

void remove_achromatic_areas(std::vector<cv::Mat> &bgrImages, std::vector<cv::Mat> &dst_masks, 
    float threshold_ach = 0.3);
void remove_achromatic_area(cv::Mat &bgrImage, cv::Mat &dst_mask,
                            int index, float threshold_ach = 0.15);
bool is_achromatic(const cv::Vec3b &bgr, float threshold);

void generate_blue_mask(const cv::Mat& bgrImage, cv::Mat& blue_mask);

void generate_white_mask(const cv::Mat& bgrImage, cv::Mat& white_mask);

void generate_combined_bus_mask(const std::vector<cv::Mat>& bgrImages, std::vector<cv::Mat>& final_masks);

void generate_bus_mask(const std::vector<cv::Mat> &bgrImages,
                                               std::vector<cv::Mat> &final_masks,
                                               float ach_threshold = 0.1,
                                               int brightness_rank = 75);
#endif