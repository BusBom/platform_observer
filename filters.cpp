#include "filters.hpp"

#include <algorithm>
#include <vector>

#define MAX_WARP_NUM 20
#define SCALE_FACTOR 1.2

static std::vector<cv::Mat> lab_channels;
static cv::Mat lab;

static std::vector<std::vector<cv::Point>> prev_rect_vec(MAX_WARP_NUM);
static std::vector<std::pair<int, int>> width_and_height_vec(
    MAX_WARP_NUM);  // width_and_height
static std::vector<cv::Mat> perspective_matrix_vec(MAX_WARP_NUM);
static std::vector<std::vector<std::pair<cv::Point, int>>> ach_points_vec(
    MAX_WARP_NUM);
static std::vector<std::vector<int>> ach_brightness_vec(MAX_WARP_NUM);

void auto_brightness_balance(cv::Mat &bgr_image, cv::Mat &dst_image) {
  cv::cvtColor(bgr_image, lab, cv::COLOR_BGR2Lab);

  lab_channels.clear();
  cv::split(lab, lab_channels);

  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(2.0);  // 대비 제한
  clahe->setTilesGridSize(cv::Size(32, 32));
  clahe->apply(lab_channels[0],
               lab_channels[0]);  // L* 채널 //97% 85% 여전히 다름

  // 다시 합치고 변환
  cv::merge(lab_channels, lab);

  cv::cvtColor(lab, dst_image, cv::COLOR_Lab2BGR);
}

void warp_rectified_areas(cv::Mat &bgr_image, std::vector<cv::Mat> &dst_image,
                          std::vector<std::vector<cv::Point>> &rect) {
  if (rect.empty()) return;

  // dst_image 크기를 rect 크기에 맞게 설정
  dst_image.resize(rect.size());
  // 초기화하지 않음 - 실제 warped 이미지만 저장

  cv::Mat result;

  for (int r_index = 0; r_index < rect.size(); r_index++) {
    if (rect[r_index].size() != 4) {
      std::cout << "Warning: Platform " << r_index
                << " has invalid point count: " << rect[r_index].size()
                << std::endl;
      continue;
    }

    if (prev_rect_vec[r_index].empty() || prev_rect_vec[r_index].size() != 4 ||
        is_rectified_area_changed(rect[r_index], r_index)) {
      prev_rect_vec[r_index] = rect[r_index];

      // Point2f로 변환
      std::vector<cv::Point2f> src_points(rect[r_index].begin(),
                                          rect[r_index].end());

      // 거리 기반 width/height 계산
      float widthA = cv::norm(src_points[1] - src_points[0]);   // top
      float widthB = cv::norm(src_points[2] - src_points[3]);   // bottom
      float heightA = cv::norm(src_points[3] - src_points[0]);  // left
      float heightB = cv::norm(src_points[2] - src_points[1]);  // right

      width_and_height_vec[r_index].first = (int)(std::max(widthA, widthB));
      width_and_height_vec[r_index].second =
          (int)(std::max(heightA, heightB) * SCALE_FACTOR);

      std::vector<cv::Point2f> dst_points = {
          {0.f, 0.f},
          {(float)width_and_height_vec[r_index].first - 1.f, 0.f},
          {(float)width_and_height_vec[r_index].first - 1.f,
           (float)width_and_height_vec[r_index].second - 1.f},
          {0.f, (float)width_and_height_vec[r_index].second - 1.f},
      };

      perspective_matrix_vec[r_index] =
          cv::getPerspectiveTransform(src_points, dst_points);
    }

    cv::warpPerspective(bgr_image, result, perspective_matrix_vec[r_index],
                        cv::Size(width_and_height_vec[r_index].first,
                                 width_and_height_vec[r_index].second));

    dst_image[r_index] = result;
    // std::cout << "Platform " << r_index << " warped: " << result.cols << "x"
    // << result.rows << std::endl;
  }
}

bool is_rectified_area_changed(std::vector<cv::Point> &rect, int p_index) {
  for (int i = 0; i < 4; i++) {
    if (rect[i] != prev_rect_vec[p_index][i]) return true;
  }

  return false;
}

void remove_achromatic_areas(std::vector<cv::Mat> &bgrImages,
                             std::vector<cv::Mat> &dst_masks,
                             float threshold_ach) {
  if (bgrImages.empty() || dst_masks.empty() ||
      bgrImages.size() != dst_masks.size()) {
    std::cout << "problem" << std::endl;
    return;
  }

  for (int i = 0; i < MAX_WARP_NUM; i++) {
    ach_points_vec[i].clear();
    ach_brightness_vec[i].clear();
  }

  for (int i_index = 0; i_index < bgrImages.size(); i_index++) {
    remove_achromatic_area(bgrImages[i_index], dst_masks[i_index], i_index,
                           threshold_ach);
  }
}

void remove_achromatic_area(cv::Mat &bgrImage, cv::Mat &dst_mask, int index,
                            float threshold_ach) {
  dst_mask = cv::Mat::zeros(bgrImage.size(), CV_8UC1);

  const int rows = bgrImage.rows;
  const int cols = bgrImage.cols;

  for (int y = 0; y < rows; ++y) {
    const cv::Vec3b *bgr_row = bgrImage.ptr<cv::Vec3b>(y);
    uchar *mask_row = dst_mask.ptr<uchar>(y);

    for (int x = 0; x < cols; ++x) {
      const cv::Vec3b &bgr = bgr_row[x];
      if (!is_achromatic(bgr, threshold_ach)) {
        mask_row[x] = 255;
      } else {
        int brightness =
            static_cast<int>(0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2]);
        ach_points_vec[index].push_back({cv::Point(x, y), brightness});
        ach_brightness_vec[index].push_back(brightness);
      }
    }
  }
}

bool is_achromatic(const cv::Vec3b &bgr, float threshold) {
  float sum = bgr[0] + bgr[1] + bgr[2];
  if (sum == 0) return true;

  float b_ratio = static_cast<float>(bgr[0]) / sum;
  float g_ratio = static_cast<float>(bgr[1]) / sum;
  float r_ratio = static_cast<float>(bgr[2]) / sum;

  float max_val = std::max({b_ratio, g_ratio, r_ratio});
  float min_val = std::min({b_ratio, g_ratio, r_ratio});

  // 파란색 성분이 가장 높다면 threshold를 관대하게
  float adaptive_threshold = (b_ratio > r_ratio && b_ratio > g_ratio)
                                 ? threshold * 0.8f  // 또는 +0.05f 등으로 조정
                                 : threshold;

  return (max_val - min_val) < adaptive_threshold;
}

/*
bool is_achromatic(const cv::Vec3b &bgr, float threshold) {
  float sum, b_ratio, g_ratio, r_ratio, max_val, min_val;

  sum = bgr[0] + bgr[1] + bgr[2];
  if (sum == 0) return true;

  b_ratio = (float)bgr[0] / sum;
  g_ratio = (float)bgr[1] / sum;
  r_ratio = (float)bgr[2] / sum;

  max_val = std::max({b_ratio, g_ratio, r_ratio});
  min_val = std::min({b_ratio, g_ratio, r_ratio});

  return (max_val - min_val) < threshold;
}
*/
void revive_white_areas(std::vector<cv::Mat> &bgrImages,
                        std::vector<cv::Mat> &dst_masks, int threshold_br) {
  if (bgrImages.empty() || dst_masks.empty() ||
      bgrImages.size() != dst_masks.size()) {
    return;
  }

  for (int i_index = 0; i_index < bgrImages.size(); i_index++) {
    revive_white_area(bgrImages[i_index], dst_masks[i_index], i_index,
                      threshold_br);
  }
}

void revive_white_area(cv::Mat &src_mask, cv::Mat &dst_mask, int index,
                       int threshold_br) {
  if (ach_points_vec[index].empty()) return;

  dst_mask = src_mask;

  std::sort(ach_brightness_vec[index].begin(), ach_brightness_vec[index].end());
  ach_brightness_vec[index].erase(std::unique(ach_brightness_vec[index].begin(),
                                              ach_brightness_vec[index].end()),
                                  ach_brightness_vec[index].end());
  int idx =
      std::clamp((int)(threshold_br / 100.f * ach_brightness_vec[index].size()),
                 0, (int)(ach_brightness_vec[index].size() - 1));

  int dynamic_br_threshold = ach_brightness_vec[index][idx];

  for (auto &[pt, brightness] : ach_points_vec[index]) {
    if (brightness > dynamic_br_threshold) {
      dst_mask.at<uchar>(pt) = 255;
    }
  }
}

void notchFilterByColorsHSV(const cv::Mat &inputImage, cv::Mat &outputImage,
                            const std::vector<cv::Scalar> &targetColorsHSV,
                            int hueTolerance, int satTolerance,
                            int valTolerance) {
  if (inputImage.empty()) {
    std::cerr << "입력 이미지가 비어 있습니다." << std::endl;
    return;
  }

  // [1] 출력 이미지 초기화
  outputImage = inputImage.clone();
  cv::Mat mask = cv::Mat::zeros(inputImage.size(), CV_8UC1);

  // [2] 입력 이미지 BGR → HSV 변환
  cv::Mat hsvImage;
  cv::cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

  // [3] 각 타겟 HSV 색상에 대해 마스킹 처리
  for (const auto &hsvColor : targetColorsHSV) {
    int h = static_cast<int>(hsvColor[0]);
    int s = static_cast<int>(hsvColor[1]);
    int v = static_cast<int>(hsvColor[2]);

    // Hue 범위는 0~179, Saturation과 Value는 0~255 범위
    cv::Scalar lower(std::max(0, h - hueTolerance),
                     std::max(0, s - satTolerance),
                     std::max(0, v - valTolerance));
    cv::Scalar upper(std::min(179, h + hueTolerance),
                     std::min(255, s + satTolerance),
                     std::min(255, v + valTolerance));

    // 마스크 생성 및 누적
    cv::Mat tempMask;
    cv::inRange(hsvImage, lower, upper, tempMask);
    cv::bitwise_or(mask, tempMask, mask);
  }

  // [4] 마스크 영역 제거
  outputImage.setTo(cv::Scalar(0, 0, 0), mask);
}

void batchApplyNotchFilterHSVtoAll(
    const std::vector<cv::Mat> &inputImages, std::vector<cv::Mat> &outputImages,
    const std::vector<cv::Scalar> &targetColorsBGR, int hueTolerance,
    int satTolerance, int valTolerance) {
  outputImages.clear();
  outputImages.reserve(inputImages.size());

  for (size_t i = 0; i < inputImages.size(); ++i) {
    const cv::Mat &img = inputImages[i];
    cv::Mat filtered;

    notchFilterByColorsHSV(img, filtered, targetColorsBGR, hueTolerance,
                           satTolerance, valTolerance);
    outputImages.push_back(filtered);
  }
}