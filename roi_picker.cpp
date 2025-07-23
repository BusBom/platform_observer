// roi_picker_json.cpp
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using json = nlohmann::json;

std::vector<std::vector<cv::Point>> all_platforms;
std::vector<cv::Point> current_platform;
cv::Mat image;

void save_to_json(const std::string& filename) {
    json j;
    for (const auto& roi : all_platforms) {
        std::vector<std::vector<int>> roi_coords;
        for (const auto& pt : roi) {
            roi_coords.push_back({pt.x, pt.y});
        }
        j["stop_rois"].push_back(roi_coords);
    }

    std::ofstream ofs(filename);
    ofs << j.dump(); // pretty print
    std::cout << "💾 JSON 저장 완료: " << filename << std::endl;
}
void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        current_platform.emplace_back(x, y);
        std::cout << "📌 좌표 추가: (" << x << ", " << y << ")" << std::endl;

        cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
        cv::imshow("ROI Picker", image);

        if (current_platform.size() == 4) {
            std::cout << "✅ 플랫폼 ROI 4개 완성됨!\n";
            all_platforms.push_back(current_platform);
            current_platform.clear();
            save_to_json("platform_rois.json");  // 플랫폼이 추가될 때마다 저장
        }
    }
}
int main() {
    std::string video_path = "file://home/Qwd/platform_observer/video/IMG_4403.mp4";
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "❌ 영상 열기 실패: " << video_path << std::endl;
        return -1;
    }

    cap.read(image);
    if (image.empty()) {
        std::cerr << "❌ 첫 프레임 로딩 실패" << std::endl;
        return -1;
    }

    cv::namedWindow("ROI Picker");
    cv::setMouseCallback("ROI Picker", onMouse);
    std::cout << "🖱️ 플랫폼 별 ROI 꼭짓점 4개씩 클릭! (여러 플랫폼 가능)\n";

    while (true) {
        cv::imshow("ROI Picker", image);
        if(cv::waitKey(10) == 'q') break;
    }
    return 0;
}
