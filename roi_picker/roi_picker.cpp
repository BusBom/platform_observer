// roi_picker.cpp
#include <opencv2/opencv.hpp>
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using json = nlohmann::json;

// --- 전역 변수 ---
std::vector<std::vector<cv::Point>> all_platforms; // 완성된 모든 플랫폼 ROI
std::vector<cv::Point> current_platform;           // 현재 그리고 있는 플랫폼 ROI
cv::Mat original_frame;                            // 원본 영상 프레임
cv::VideoCapture cap;                              // 비디오 캡처 객체
bool is_paused = true;                             // 일시정지 상태 플래그

/**
 * @brief 현재까지 수집된 모든 ROI 데이터를 JSON 파일로 저장합니다.
 * @param filename 저장할 파일 이름.
 */
void save_to_json(const std::string& filename) {
    json j;
    j["stop_rois"] = json::array(); // 명시적으로 배열로 초기화

    for (const auto& roi : all_platforms) {
        json roi_coords = json::array();
        for (const auto& pt : roi) {
            roi_coords.push_back({pt.x, pt.y});
        }
        j["stop_rois"].push_back(roi_coords);
    }

    std::ofstream ofs(filename);
    ofs << j.dump();
    std::cout << "💾 JSON 저장 완료: " << filename << std::endl;
}

/**
 * @brief 마우스 이벤트를 처리하는 콜백 함수.
 */
void onMouse(int event, int x, int y, int, void*) {
    if (is_paused && event == cv::EVENT_LBUTTONDOWN) {
        if (current_platform.size() < 4) {
            current_platform.emplace_back(x, y);
            std::cout << "📌 좌표 추가: (" << x << ", " << y << ")" << std::endl;

            if (current_platform.size() == 4) {
                all_platforms.push_back(current_platform);
                current_platform.clear();
                std::cout << "✅ 플랫폼 ROI 완성! 자동으로 저장됩니다." << std::endl;
                save_to_json("platform_rois.json"); // 4개의 점이 완성되면 바로 저장
            }
        } else {
             std::cout << "⚠️ 한 플랫폼은 4개의 점만 가질 수 있습니다. 'r'로 초기화하거나 새 플랫폼을 시작하세요." << std::endl;
        }
    }
}

/**
 * @brief 화면에 현재 상태와 안내 문구를 표시합니다.
 * @param img 텍스트를 그릴 이미지.
 */
void draw_hud(cv::Mat& img) {
    std::string status_text = is_paused ? "PAUSED" : "PLAYING";
    long long current_frame_idx = static_cast<long long>(cap.get(cv::CAP_PROP_POS_FRAMES));
    long long total_frames = static_cast<long long>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    
    std::string frame_text = "Frame: " + std::to_string(current_frame_idx) + " / " + std::to_string(total_frames);

    cv::putText(img, status_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
    cv::putText(img, frame_text, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

    cv::putText(img, "SPACE: Play/Pause | N/B: Frame | S: Save", cv::Point(20, img.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(img, "Z: Undo Point | R: Reset Platform | Q: Quit", cv::Point(20, img.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

int main() {
    std::string video_path = "video/2nd_color_graded_15fps.mp4";
    cap.open(video_path);
    if (!cap.isOpened()) {
        std::cerr << "❌ 영상 열기 실패: " << video_path << std::endl;
        return -1;
    }

    cap.read(original_frame);
    if (original_frame.empty()) {
        std::cerr << "❌ 첫 프레임 로딩 실패" << std::endl;
        return -1;
    }

    cv::namedWindow("ROI Picker");
    cv::setMouseCallback("ROI Picker", onMouse);
    std::cout << "--- ROI Picker 시작 ---" << std::endl;
    std::cout << "화면을 클릭하여 플랫폼 별 ROI 꼭짓점 4개를 선택하세요." << std::endl;
    std::cout << "자세한 단축키는 화면 좌측 하단을 참고하세요." << std::endl;

    while (true) {
        if (!is_paused) {
            cap.read(original_frame);
            if (original_frame.empty()) {
                std::cout << "영상의 끝에 도달했습니다. 마지막 프레임에서 정지합니다." << std::endl;
                cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_FRAME_COUNT) - 1);
                cap.read(original_frame);
                is_paused = true;
            }
        }

        cv::Mat display_frame = original_frame.clone();

        // 완성된 플랫폼들 그리기 (초록색)
        for (const auto& roi : all_platforms) {
            cv::polylines(display_frame, roi, true, cv::Scalar(0, 255, 0), 2);
        }

        // 현재 그리고 있는 플랫폼 그리기 (노란색)
        for (size_t i = 0; i < current_platform.size(); ++i) {
            cv::circle(display_frame, current_platform[i], 5, cv::Scalar(0, 255, 255), -1);
            if (i > 0) {
                cv::line(display_frame, current_platform[i-1], current_platform[i], cv::Scalar(0, 255, 255), 2);
            }
        }

        draw_hud(display_frame);
        cv::imshow("ROI Picker", display_frame);

        char key = cv::waitKey(30);

        if (key == 'q' || key == 27) { // 'q' 또는 ESC
            break;
        } else if (key == ' ') { // 스페이스바
            is_paused = !is_paused;
        } else if (key == 's') { // 저장 (수동 저장 기능)
            save_to_json("platform_rois.json");
        }
        
        if (is_paused) {
            if (key == 'n') { // 다음 프레임
                cap.read(original_frame);
                if (original_frame.empty()) { // 영상 끝이면 이전 프레임으로 복귀
                   cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_FRAME_COUNT) - 2);
                   cap.read(original_frame);
                }
            } else if (key == 'b') { // 이전 프레임
                long long current_pos = static_cast<long long>(cap.get(cv::CAP_PROP_POS_FRAMES));
                if (current_pos > 1) {
                    cap.set(cv::CAP_PROP_POS_FRAMES, current_pos - 2);
                    cap.read(original_frame);
                }
            } else if (key == 'z') { // 마지막 점 취소
                if (!current_platform.empty()) {
                    current_platform.pop_back();
                    std::cout << "🔙 마지막 점을 취소했습니다." << std::endl;
                }
            } else if (key == 'r') { // 현재 플랫폼 리셋
                current_platform.clear();
                std::cout << "🔄 현재 플랫폼 그리기를 초기화했습니다." << std::endl;
            }
        }
    }
    return 0;
}
