#include <chrono>  // warp 되는 것까지 확인함, 화면 디버깅 지우고, 다음 단계로 넘어갈 것
#include <iostream>
#include <queue>
#include <thread>

#include "checker.hpp"
#include "filters.hpp"
#include "safeQueue.hpp"

#if 1
#include <X11/Xlib.h>
#endif

#define MAX_QUEUE_SIZE 1000

/** 현재 조정 상황
유채색 vs 무채색 : 0.15
영역 내 흰색 판단 : 70 (상위 30%)
버스 또는 물체 유무 판단 : 0.4
*/

unsigned int PLATFORM_SIZE = 4;
const char cap_name[] = "final.mp4";
// const char cap_name[] =
//   "libcamerasrc ! video/x-raw,format=BGR,width=640,height=480,framerate=30/1
//   ! " "videoconvert ! appsink";
bool is_capture_thread_running = true;

static std::vector<cv::Scalar> notch_target = {
    cv::Scalar(0, 15, 110),    // 아스팔트
    cv::Scalar(0, 20, 150),    //정류장 기둥
    cv::Scalar(100, 25, 170),  // 정류장 유리
    cv::Scalar(0, 10, 200)     // 인도 블록
};
static std::vector<std::vector<cv::Point>> clicked_points(PLATFORM_SIZE);
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> frame_queue;
static ThreadSafeQueue<std::shared_ptr<std::vector<cv::Mat>>> warped_queue;
static ThreadSafeQueue<std::shared_ptr<std::vector<cv::Mat>>> masked_queue;

void capture_thread();
void warp_thread();
void mask_thread();

/*start debugging */
unsigned int total_frame = 0;
unsigned int frame_wasted = 0;
unsigned int warped_wasted = 0;
unsigned int masked_wasted = 0;

static std::vector<cv::Point> temp_points;
static int current_platform_index = 0;
static bool ready_to_start = false;

// for debugging
void onMouseClick(int event, int x, int y, int flags, void* userdata) {
  if (event != cv::EVENT_LBUTTONDOWN || current_platform_index >= PLATFORM_SIZE)
    return;

  temp_points.emplace_back(x, y);
  std::cout << "Point " << temp_points.size() << " for Platform "
            << current_platform_index << ": (" << x << ", " << y << ")\n";

  if (temp_points.size() == 4) {
    bool is_valid_rectangle = true;

    // 1. 중복 점 확인
    for (int i = 0; i < 4; ++i) {
      for (int j = i + 1; j < 4; ++j) {
        if (temp_points[i] == temp_points[j]) {
          std::cout << "Error: Duplicate points detected!" << std::endl;
          is_valid_rectangle = false;
          break;
        }
      }
    }

    // 2. 최소 크기 확인
    if (is_valid_rectangle) {
      int min_x = std::min({temp_points[0].x, temp_points[1].x,
                            temp_points[2].x, temp_points[3].x});
      int max_x = std::max({temp_points[0].x, temp_points[1].x,
                            temp_points[2].x, temp_points[3].x});
      int min_y = std::min({temp_points[0].y, temp_points[1].y,
                            temp_points[2].y, temp_points[3].y});
      int max_y = std::max({temp_points[0].y, temp_points[1].y,
                            temp_points[2].y, temp_points[3].y});

      int width = max_x - min_x;
      int height = max_y - min_y;

      if (width < 10 || height < 10) {
        std::cout << "Error: Rectangle too small! (width: " << width
                  << ", height: " << height << ")" << std::endl;
        is_valid_rectangle = false;
      }
    }

    if (is_valid_rectangle) {
      clicked_points[current_platform_index].clear();
      clicked_points[current_platform_index].reserve(4);
      clicked_points[current_platform_index] = temp_points;

      if (clicked_points[current_platform_index].size() != 4) {
        std::cout << "Error: Failed to assign points to platform "
                  << current_platform_index << std::endl;
        temp_points.clear();
        return;
      }

      std::cout << "Platform " << current_platform_index
                << " selected successfully.\n";
      std::cout << "  Points: (" << temp_points[0].x << "," << temp_points[0].y
                << ") "
                << "(" << temp_points[1].x << "," << temp_points[1].y << ") "
                << "(" << temp_points[2].x << "," << temp_points[2].y << ") "
                << "(" << temp_points[3].x << "," << temp_points[3].y << ")\n";

      current_platform_index++;
      temp_points.clear();
    } else {
      std::cout
          << "Invalid rectangle! Please select 4 points again for Platform "
          << current_platform_index << ".\n";
      temp_points.clear();
    }
  }

  if (current_platform_index == PLATFORM_SIZE) {
    std::cout << "All platforms selected.\n";
    ready_to_start = true;
  }
}

void wait_for_user_clicks(cv::VideoCapture& cap) {
  cv::Mat frame, aframe;
  cap >> frame;
  auto_brightness_balance(frame, aframe);
  cap.release();  // ✅ 첫 프레임 캡처 후 비디오 객체 해제
  if (frame.empty()) {
    std::cerr << "❌ 첫 프레임을 불러오지 못했습니다.\n";
    return;
  }

  cv::namedWindow("Select Platforms", cv::WINDOW_NORMAL);
  cv::resizeWindow("Select Platforms", 1280, 960);
  cv::setMouseCallback("Select Platforms", onMouseClick, nullptr);

  std::cout << "💡 Tip: Select 4 points in clockwise order (top-left, "
               "top-right, bottom-right, bottom-left)\n";

  cv::Mat final_display;

  while (!ready_to_start) {
    cv::Mat display = aframe.clone();

    std::string status_text = "Platform " +
                              std::to_string(current_platform_index) + " (" +
                              std::to_string(temp_points.size()) + "/4 points)";
    cv::putText(display, status_text, cv::Point(30, 60),
                cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 128), 6);

    // 기존 선택된 플랫폼 그리기
    for (int i = 0; i < current_platform_index; ++i) {
      const auto& pts = clicked_points[i];
      for (int j = 0; j < 4; ++j)
        cv::line(display, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 255, 128), 5);
      for (int j = 0; j < 4; ++j)
        cv::circle(display, pts[j], 8, cv::Scalar(0, 255, 128), -1);
    }

    // 현재 선택 중인 점
    for (const auto& pt : temp_points)
      cv::circle(display, pt, 8, cv::Scalar(0, 255, 128), -1);

    // 4개 점 모두 선택되었을 경우 임시 사각형 표시
    if (temp_points.size() == 4) {
      for (int j = 0; j < 4; ++j)
        cv::line(display, temp_points[j], temp_points[(j + 1) % 4],
                 cv::Scalar(0, 255, 128), 16);
    }

    final_display = display.clone();
    cv::imshow("Select Platforms", display);

    if (cv::waitKey(30) == 27) break;  // ESC 키 누르면 강제 종료
  }

  // 최종 결과 이미지 작성 및 저장
  if (!frame.empty()) {
    final_display = frame.clone();

    std::string final_text =
        "Final Platform Selections: " + std::to_string(current_platform_index);
    cv::putText(final_display, final_text, cv::Point(30, 60),
                cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 128), 6);

    for (const auto& pts : clicked_points) {
      for (int j = 0; j < 4; ++j)
        cv::line(final_display, pts[j], pts[(j + 1) % 4],
                 cv::Scalar(0, 255, 128), 5);
      for (int j = 0; j < 4; ++j)
        cv::circle(final_display, pts[j], 8, cv::Scalar(0, 255, 128), -1);
    }

    std::string filename = "img/selected_platforms_final.jpg";
    cv::imwrite(filename, final_display);
    std::cout << "✅ 최종 플랫폼 선택 이미지를 저장했습니다: " << filename
              << std::endl;
  }

  cv::destroyWindow("Select Platforms");
}

/*end debugging */

/*bus*/
bool BUS_PLATFORM_STATUS[20] = {false};  // MAX_PLATFORM_SIZE = 20

int main() {
  if (!XInitThreads()) {
    std::cerr << "Failed to initialize X11 threading" << std::endl;
    return -1;
  }

  cv::VideoCapture cap(cap_name);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open video source.\n";
    return -1;
  }

  std::cout << "Initial clicked_points size: " << clicked_points.size()
            << std::endl;
  for (int i = 0; i < clicked_points.size(); ++i) {
    std::cout << "Platform " << i
              << " initial size: " << clicked_points[i].size() << std::endl;
  }

  std::cout << "📌 사용자 클릭으로 플랫폼 영역을 지정하세요 (총 "
            << PLATFORM_SIZE << "개).\n";
  wait_for_user_clicks(cap);

  cap.release();

  std::thread cap_thread(capture_thread);
  std::thread warp_thread_(warp_thread);
  std::thread mask_thread_(mask_thread);

  std::shared_ptr<std::vector<cv::Mat>> masked;
  std::vector<cv::Mat> last_masked;

  cv::namedWindow("Original Frame", cv::WINDOW_NORMAL);
  cv::resizeWindow("Original Frame", 800, 600);

  for (int i = 0; i < PLATFORM_SIZE; ++i) {
    std::string win_name = "Masked Platform " + std::to_string(i);
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(win_name, 100, 200);
  }

  auto frame_start = std::chrono::high_resolution_clock::now();

  while (is_capture_thread_running || !masked_queue.empty()) {
    // 기존 masked_queue 처리
    if (masked_queue.try_pop(masked)) {
      last_masked = *masked;
      check_bus_platform(last_masked, BUS_PLATFORM_STATUS);
      std::cout << "-------------------------------------------\n";
      for (int i = 0; i < last_masked.size(); i++) {
        std::cout << "Platform " << i << " status: " << BUS_PLATFORM_STATUS[i]
                  << std::endl;
      }
      std::cout << "-------------------------------------------\n";
      auto frame_end = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         frame_end - frame_start)
                         .count();
      // std::cout << "[main] Frame output time: " << elapsed << " ms" <<
      // std::endl;
      frame_start = std::chrono::high_resolution_clock::now();
    }

    // warped, masked 출력 처리
    for (int i = 0; i < last_masked.size(); ++i) {
      if (!last_masked[i].empty()) {
        std::string win_name = "Masked Platform " + std::to_string(i);
        cv::Mat resized_masked;
        cv::resize(last_masked[i], resized_masked, cv::Size(800, 600));
        cv::imshow(win_name, resized_masked);
      }
    }

    // 여기서 frame_queue에서 이미지 하나 꺼내서 보여주기
    std::shared_ptr<cv::Mat> original_frame;
    if (frame_queue.try_pop(original_frame)) {
      if (original_frame && !original_frame->empty()) {
        cv::Mat resized_orig;
        cv::resize(*original_frame, resized_orig, cv::Size(800, 600));
        cv::imshow("Original Frame", resized_orig);
      }
    }

    int key = cv::waitKey(30);
    if (key == 27) {
      is_capture_thread_running = false;
      break;
    }
  }

  cap_thread.join();
  warp_thread_.join();
  mask_thread_.join();

  cv::destroyAllWindows();

  std::cout << "Program done : " << total_frame << ":" << frame_wasted << ":"
            << warped_wasted << ":" << masked_wasted << std::endl;

  return 0;
}

void capture_thread() {
  cv::VideoCapture cap(cap_name);
  if (!cap.isOpened()) {
    std::cerr << "ERROR: Could not open camera\n";
    is_capture_thread_running = false;
    return;
  }

  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);
  cv::Mat frame, balanced;
  std::shared_ptr<cv::Mat> garbage;

  while (is_capture_thread_running) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!cap.read(frame) || frame.empty()) {
      std::cerr << "WARNING: Empty frame or failed to read\n";
      break;
    }

    if (frame_queue.size() > MAX_QUEUE_SIZE) {
      frame_queue.try_pop(garbage);
      frame_wasted++;
    }

    auto_brightness_balance(frame,
                            balanced);  // 이 함수가 정의되어 있어야 합니다.
    total_frame++;
    frame_queue.push(std::make_shared<cv::Mat>(balanced));

    // 캡처 스레드에서는 GUI 관련 코드는 제거 권장

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
  }

  cap.release();
  is_capture_thread_running = false;
}

bool is_every_rects_ready(std::vector<std::vector<cv::Point>>& rects) {
  for (int i = 0; i < rects.size(); i++) {
    if (rects[i].empty() || rects[i].size() != 4) return false;
  }

  return true;
}

void warp_thread() {
  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);

  std::shared_ptr<cv::Mat> frame;
  std::shared_ptr<std::vector<cv::Mat>> garbage;
  std::vector<cv::Mat> warped;

  while (is_capture_thread_running || !frame_queue.empty()) {
    auto start_time = std::chrono::high_resolution_clock::now();
    warped.clear();

    if (clicked_points.size() != PLATFORM_SIZE ||
        !is_every_rects_ready(clicked_points)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;  // 아직 포인트가 준비 안됨
    }

    if (warped_queue.size() > MAX_QUEUE_SIZE) {
      warped_queue.try_pop(garbage);
      warped_wasted++;
    }

    if (frame_queue.try_pop(frame)) {
      warp_rectified_areas((*frame), warped, clicked_points);
      warped_queue.push(
          std::make_shared<std::vector<cv::Mat>>(std::move(warped)));
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
  }
}

void mask_thread() {
  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);

  std::shared_ptr<std::vector<cv::Mat>> frame;
  std::shared_ptr<std::vector<cv::Mat>> garbage;
  std::vector<cv::Mat> notched, masked_1, masked_2;

  while (is_capture_thread_running || !warped_queue.empty()) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (masked_queue.size() > MAX_QUEUE_SIZE) {
      masked_queue.try_pop(garbage);
      masked_wasted++;
    }

    if (warped_queue.try_pop(frame)) {
      notched.resize(PLATFORM_SIZE);
      masked_1.resize(PLATFORM_SIZE);
      masked_2.resize(PLATFORM_SIZE);

      batchApplyNotchFilterHSVtoAll((*frame), notched, notch_target);
      remove_achromatic_areas(notched, masked_1);  // default : 0.15
      revive_white_areas(masked_1, masked_2);      // default : 95%

      masked_queue.push(
          std::make_shared<std::vector<cv::Mat>>(std::move(masked_2)));
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
  }
}
