#include "filters.hpp"

#include <algorithm>
#include <vector>

#define MAX_WARP_NUM 20
#define SCALE_FACTOR 1.2

static std::vector<cv::Mat> lab_channels;
static cv::Mat lab;

static std::vector<std::vector<cv::Point>> prev_rect_vec(MAX_WARP_NUM);
static std::vector<std::pair<int, int>> width_and_height_vec(MAX_WARP_NUM);  // width_and_height
static std::vector<cv::Mat> perspective_matrix_vec(MAX_WARP_NUM);
static std::vector<std::vector<std::pair<cv::Point, int>>> ach_points_vec(MAX_WARP_NUM);
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
		std::cout << "Warning: Platform " << r_index << " has invalid point count: " << rect[r_index].size() << std::endl;
		continue;
		}

		if (prev_rect_vec[r_index].empty() || prev_rect_vec[r_index].size() != 4 ||
			is_rectified_area_changed(rect[r_index], r_index)) {
		prev_rect_vec[r_index] = rect[r_index];

		// Point2f로 변환
		std::vector<cv::Point2f> src_points(rect[r_index].begin(), rect[r_index].end());

		// 거리 기반 width/height 계산
		float widthA = cv::norm(src_points[1] - src_points[0]);   // top
		float widthB = cv::norm(src_points[2] - src_points[3]);   // bottom
		float heightA = cv::norm(src_points[3] - src_points[0]);  // left
		float heightB = cv::norm(src_points[2] - src_points[1]);  // right

		width_and_height_vec[r_index].first = (int)(std::max(widthA, widthB));
		width_and_height_vec[r_index].second = (int)(std::max(heightA, heightB) * SCALE_FACTOR);

		std::vector<cv::Point2f> dst_points = {
				{0.f, 0.f},
				{(float)width_and_height_vec[r_index].first - 1.f, 0.f},
				{(float)width_and_height_vec[r_index].first - 1.f, (float)width_and_height_vec[r_index].second - 1.f},
				{0.f, (float)width_and_height_vec[r_index].second - 1.f},
			};

			perspective_matrix_vec[r_index] = cv::getPerspectiveTransform(src_points, dst_points);
		}
		// 미리 dst_image[r_index] 크기 및 타입 맞춰 생성
		cv::Size roi_size(width_and_height_vec[r_index].first, width_and_height_vec[r_index].second);

		if (dst_image[r_index].empty() || dst_image[r_index].size() != roi_size) {
			dst_image[r_index].create(roi_size, bgr_image.type());
		}

		// clone 없이 직접 출력
		cv::warpPerspective(bgr_image, dst_image[r_index], perspective_matrix_vec[r_index], roi_size);

		//std::cout << "Platform " << r_index << " warped: " << result.cols << "x" << result.rows << std::endl;
	}
}

bool is_rectified_area_changed(std::vector<cv::Point> &rect, int p_index) {
    for (int i = 0; i < 4; i++) {
        if (rect[i] != prev_rect_vec[p_index][i]) return true;
    }

    return false;
}

void remove_achromatic_areas(std::vector<cv::Mat> &bgrImages, std::vector<cv::Mat> &dst_masks, 
    float threshold_ach){

    if(bgrImages.empty()||dst_masks.empty()||
        bgrImages.size()!=dst_masks.size()){
            std::cout << "problem" << std::endl;
            return;
        } 

    for(int i = 0; i < MAX_WARP_NUM; i++){
        ach_points_vec[i].clear();
        ach_brightness_vec[i].clear();
    }

    for(int i_index = 0; i_index < bgrImages.size(); i_index++){
        remove_achromatic_area(bgrImages[i_index], dst_masks[i_index], i_index, threshold_ach);
    }
}

void remove_achromatic_area(cv::Mat &bgrImage, cv::Mat &dst_mask,
                            int index, float threshold_ach) {
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
                int brightness = bgr[0] + bgr[1] + bgr[2];
                ach_points_vec[index].push_back({cv::Point(x, y), brightness});
                ach_brightness_vec[index].push_back(brightness);
            }
        }
    }
}

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

void generate_blue_mask(const cv::Mat& bgrImage, cv::Mat& blue_mask) {
    cv::Mat hsv;
    cv::cvtColor(bgrImage, hsv, cv::COLOR_BGR2HSV);
    // 파란색 범위: Hue 100~130
    cv::inRange(hsv, cv::Scalar(95, 100, 30), cv::Scalar(125, 255, 230), blue_mask);
}

// 2. 밝은 흰색 마스크 생성 함수 (새로 추가)
void generate_white_mask(const cv::Mat& bgrImage, cv::Mat& white_mask) {
    cv::Mat gray;
    cv::cvtColor(bgrImage, gray, cv::COLOR_BGR2GRAY);
    // 매우 밝은 영역만 흰색으로 추출 (임계값 220, 조정 가능)
    cv::threshold(gray, white_mask, 210, 255, cv::THRESH_BINARY);
}

// 3. 두 마스크를 결합하는 메인 필터 함수
void generate_combined_bus_mask(const std::vector<cv::Mat>& bgrImages, std::vector<cv::Mat>& final_masks) {
    final_masks.resize(bgrImages.size());

    for (size_t i = 0; i < bgrImages.size(); ++i) {
        cv::Mat blue_mask, white_mask;

        // 각 트랙을 실행하여 마스크 생성
        generate_blue_mask(bgrImages[i], blue_mask);
        generate_white_mask(bgrImages[i], white_mask);

        // 비트 OR 연산으로 두 마스크를 합침
        cv::bitwise_or(blue_mask, white_mask, final_masks[i]);
    }
}

void generate_bus_mask(const std::vector<cv::Mat> &bgrImages, std::vector<cv::Mat> &final_masks,
                                               float ach_threshold, int brightness_rank) {
    final_masks.resize(bgrImages.size());

    for (size_t idx = 0; idx < bgrImages.size(); ++idx) {
        const cv::Mat &img = bgrImages[idx];
        cv::Mat hsv, blue_mask, white_mask, gray;

        // [1] 블루 마스크
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(95, 120, 80), cv::Scalar(125, 255, 255), blue_mask);

        // [2] 밝은 무채색 마스크
        std::vector<int> brightness_values;
        std::vector<cv::Point> bright_points;

        white_mask = cv::Mat::zeros(img.size(), CV_8UC1);

        for (int y = 0; y < img.rows; ++y) {
            const cv::Vec3b *row = img.ptr<cv::Vec3b>(y);
            for (int x = 0; x < img.cols; ++x) {
                const cv::Vec3b &bgr = row[x];
                float sum = bgr[0] + bgr[1] + bgr[2];
                if (sum == 0) continue;

                float b_ratio = (float)bgr[0] / sum;
                float g_ratio = (float)bgr[1] / sum;
                float r_ratio = (float)bgr[2] / sum;
                float max_v = std::max({b_ratio, g_ratio, r_ratio});
                float min_v = std::min({b_ratio, g_ratio, r_ratio});

                if ((max_v - min_v) < ach_threshold) {
                    int brightness = bgr[0] + bgr[1] + bgr[2];
                    brightness_values.push_back(brightness);
                    bright_points.push_back(cv::Point(x, y));
                }
            }
        }

        // 밝기 기준 상위 N%만 선택
        if (!brightness_values.empty()) {
            std::sort(brightness_values.begin(), brightness_values.end());
            brightness_values.erase(std::unique(brightness_values.begin(), brightness_values.end()), brightness_values.end());

            int rank_idx = std::clamp(static_cast<int>((brightness_rank / 100.0f) * brightness_values.size()), 0, (int)brightness_values.size() - 1);
            int bright_thresh = brightness_values[rank_idx];

            for (size_t i = 0; i < bright_points.size(); ++i) {
                if ((bgrImages[idx].at<cv::Vec3b>(bright_points[i])[0] +
                     bgrImages[idx].at<cv::Vec3b>(bright_points[i])[1] +
                     bgrImages[idx].at<cv::Vec3b>(bright_points[i])[2]) > bright_thresh) {
                    white_mask.at<uchar>(bright_points[i]) = 255;
                }
            }
        }

        // [3] 최종 OR
        cv::bitwise_or(blue_mask, white_mask, final_masks[idx]);
    }
}
