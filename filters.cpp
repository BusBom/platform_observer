#include "filters.hpp"

#include <algorithm>
#include <vector>

static std::vector<cv::Mat> lab_channels;
static cv::Mat lab;
static std::vector<std::pair<cv::Point, int>> ach_points;
static std::vector<int> ach_brightness;
static std::vector<cv::Point> prev_rect;
static int target_width;
static int target_height;
static std::vector<cv::Point2f> dst_points;
static cv::Mat perspective_matrix;


void auto_brightness_balance(cv::Mat &bgr_image, cv::Mat& dst_image){

    cv::cvtColor(bgr_image, lab, cv::COLOR_BGR2Lab);

    lab_channels.clear();
    cv::split(lab, lab_channels);

     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0); // 대비 제한
    clahe->setTilesGridSize(cv::Size(8, 8));
    clahe->apply(lab_channels[0], lab_channels[0]); // L* 채널 //97% 85% 여전히 다름

    // 다시 합치고 변환
    cv::merge(lab_channels, lab);

    cv::cvtColor(lab, dst_image, cv::COLOR_Lab2BGR);
}

void warp_rectified_area(cv::Mat &bgr_image, cv::Mat &dst_image, std::vector<cv::Point> &rect)
{
    // 기본 유효성 검사
    if (rect.size() != 4) {
        return;
    }
    if (bgr_image.empty()) {
        return;
    }

    // rect 변경 여부 확인
    if (prev_rect.empty() || prev_rect.size() != 4 || is_rectified_area_changed(rect)) {
        prev_rect = rect;  // 이전 rect 갱신

        // Point2f로 변환
        std::vector<cv::Point2f> src_points(rect.begin(), rect.end());

        // 거리 기반 width/height 계산
        float widthA = cv::norm(src_points[1] - src_points[0]); // top
        float widthB = cv::norm(src_points[2] - src_points[3]); // bottom
        float heightA = cv::norm(src_points[3] - src_points[0]); // left
        float heightB = cv::norm(src_points[2] - src_points[1]); // right

        target_width = (int)(std::max(widthA, widthB));
        target_height = (int)(std::max(heightA, heightB));

        dst_points = {
            {0.f, 0.f},
            {(float)target_width - 1.f, 0.f},
            {(float)target_width - 1.f, (float)target_height - 1.f},
            {0.f, (float)target_height - 1.f},
        };

        perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);
    }

    cv::warpPerspective(bgr_image, dst_image, perspective_matrix,
                            cv::Size(target_width, target_height));
}

bool is_rectified_area_changed(std::vector<cv::Point> &rect)
{
    for (int i = 0; i < 4; i++)
    {
        if (rect[i] != prev_rect[i])
            return true;
    }

    return false;
}

void remove_achromatic_area(cv::Mat &bgrImage, cv::Mat &dst_mask, float threshold_ach)
{
    dst_mask = cv::Mat::zeros(bgrImage.size(), CV_8UC1);

    ach_points.clear();
    ach_brightness.clear();

    const int rows = bgrImage.rows;
    const int cols = bgrImage.cols;

    for (int y = 0; y < rows; ++y)
    {
        const cv::Vec3b* bgr_row = bgrImage.ptr<cv::Vec3b>(y);
        uchar* mask_row = dst_mask.ptr<uchar>(y);

        for (int x = 0; x < cols; ++x)
        {
            const cv::Vec3b& bgr = bgr_row[x];
            if (!is_achromatic(bgr, threshold_ach))
            {
                mask_row[x] = 255;
            }
            else
            {
                int brightness = bgr[0] + bgr[1] + bgr[2];
                ach_points.push_back({cv::Point(x, y), brightness});
                ach_brightness.push_back(brightness);
            }
        }
    }
}

bool is_achromatic(const cv::Vec3b &bgr, float threshold)
{
    float sum, b_ratio, g_ratio, r_ratio, max_val, min_val;

    sum = bgr[0] + bgr[1] + bgr[2];
    if (sum == 0)
        return true;

    b_ratio = (float)bgr[0] / sum;
    g_ratio = (float)bgr[1] / sum;
    r_ratio = (float)bgr[2] / sum;

    max_val = std::max({b_ratio, g_ratio, r_ratio});
    min_val = std::min({b_ratio, g_ratio, r_ratio});

    return (max_val - min_val) < threshold;
}

void revive_white_area(cv::Mat &src_mask, cv::Mat &dst_mask,
                       int threshold_br)
{
    if (ach_points.empty())
        return;

    dst_mask = src_mask.clone();

    std::sort(ach_brightness.begin(), ach_brightness.end());
    ach_brightness.erase(std::unique(ach_brightness.begin(), ach_brightness.end()), ach_brightness.end());
    int idx = std::clamp(
        (int)(threshold_br / 100.f * ach_brightness.size()),
        0, (int)(ach_brightness.size() - 1));

    int dynamic_br_threshold = ach_brightness[idx];

    for (auto &[pt, brightness] : ach_points)
    {
        if (brightness > dynamic_br_threshold)
        {
            dst_mask.at<uchar>(pt) = 255;
        }
    }
}