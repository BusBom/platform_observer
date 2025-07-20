// clone 하는 부분 최대한 출일 것, shared_ptr => 관련 글쓰기
#include <chrono> //warp 되는 것까지 확인함, 화면 디버깅 지우고, 다음 단계로 넘어갈 것것
#include <iostream>
#include <queue>
#include <thread>

#include "checker.hpp"
#include "filters.hpp"
#include "safeQueue.hpp"

#define MAX_QUEUE_SIZE 1000


unsigned int PLATFORM_SIZE = 10;
const char cap_name[] = "test.MP4";
// const char cap_name[] =
//   "libcamerasrc ! video/x-raw,format=BGR,width=640,height=480,framerate=30/1
//   ! " "videoconvert ! appsink";
bool is_capture_thread_running = true;

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

void onMouseClick(int event, int x, int y, int flags, void* userdata) {
  if (event != cv::EVENT_LBUTTONDOWN || current_platform_index >= PLATFORM_SIZE)
    return;

  temp_points.emplace_back(x, y);
  std::cout << "Point " << temp_points.size() << " for Platform "
            << current_platform_index << ": (" << x << ", " << y << ")\n";

  if (temp_points.size() == 4) {
    // 사각형 유효성 검증
    bool is_valid_rectangle = true;
    
    // 1. 모든 점이 서로 다른지 확인
    for (int i = 0; i < 4; ++i) {
      for (int j = i + 1; j < 4; ++j) {
        if (temp_points[i] == temp_points[j]) {
          std::cout << "Error: Duplicate points detected!" << std::endl;
          is_valid_rectangle = false;
          break;
        }
      }
    }
    
    // 2. 최소 크기 확인 (너무 작은 사각형 방지)
    if (is_valid_rectangle) {
      int min_x = std::min({temp_points[0].x, temp_points[1].x, temp_points[2].x, temp_points[3].x});
      int max_x = std::max({temp_points[0].x, temp_points[1].x, temp_points[2].x, temp_points[3].x});
      int min_y = std::min({temp_points[0].y, temp_points[1].y, temp_points[2].y, temp_points[3].y});
      int max_y = std::max({temp_points[0].y, temp_points[1].y, temp_points[2].y, temp_points[3].y});
      
      int width = max_x - min_x;
      int height = max_y - min_y;
      
      if (width < 10 || height < 10) {
        std::cout << "Error: Rectangle too small! (width: " << width << ", height: " << height << ")" << std::endl;
        is_valid_rectangle = false;
      }
    }
    
    if (is_valid_rectangle) {
      // clicked_points에 안전하게 할당
      clicked_points[current_platform_index].clear(); // 기존 내용 제거
      clicked_points[current_platform_index].reserve(4); // 메모리 예약
      clicked_points[current_platform_index] = temp_points; // 복사
      
      // 할당 후 검증
      if (clicked_points[current_platform_index].size() != 4) {
        std::cout << "Error: Failed to assign points to platform " << current_platform_index << std::endl;
        temp_points.clear();
        return;
      }
      
      std::cout << "Platform " << current_platform_index << " selected successfully.\n";
      std::cout << "  Points: (" << temp_points[0].x << "," << temp_points[0].y << ") "
                << "(" << temp_points[1].x << "," << temp_points[1].y << ") "
                << "(" << temp_points[2].x << "," << temp_points[2].y << ") "
                << "(" << temp_points[3].x << "," << temp_points[3].y << ")\n";
      std::cout << "  Stored points count: " << clicked_points[current_platform_index].size() << std::endl;
      
      current_platform_index++;
      temp_points.clear();
    } else {
      std::cout << "Invalid rectangle! Please select 4 points again for Platform " << current_platform_index << ".\n";
      temp_points.clear();
    }
  }

  if (current_platform_index == PLATFORM_SIZE) {
    std::cout << "All platforms selected.\n";
    ready_to_start = true;
  }
}

void wait_for_user_clicks(cv::VideoCapture& cap) {
  cv::Mat frame;

  cv::namedWindow("Select Platforms", cv::WINDOW_NORMAL);
  cv::resizeWindow("Select Platforms", 800, 600);
  cv::setMouseCallback("Select Platforms", onMouseClick, nullptr);

  std::cout << "💡 Tip: Select 4 points in clockwise order (top-left, top-right, bottom-right, bottom-left)\n";

  while (!ready_to_start) {
    cap >> frame;
    if (frame.empty()) break;

    // 현재 선택 중인 플랫폼 정보 표시
    std::string status_text = "Platform " + std::to_string(current_platform_index) + 
                             " (" + std::to_string(temp_points.size()) + "/4 points)";
    cv::putText(frame, status_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    // 이미 선택한 플랫폼 그리기
    for (int i = 0; i < current_platform_index; ++i) {
      auto& pts = clicked_points[i];
      for (int j = 0; j < 4; ++j)
        cv::line(frame, pts[j], pts[(j + 1) % 4], cv::Scalar(255, 10*i, 0), 2);
    }

    // 현재 선택 중인 점 표시
    for (const auto& pt : temp_points)
      cv::circle(frame, pt, 5, cv::Scalar(0, 255, 0), -1);

    cv::imshow("Select Platforms", frame);
    if (cv::waitKey(30) == 27) break;  // ESC 누르면 종료
  }

  cv::destroyWindow("Select Platforms");
}

/*end debuging */

/*bus*/
bool BUS_PLATFORM_STATUS[20] = {
    false,
}; //MAX_PLATFORM_SIZE = 20

int main() {
  cv::VideoCapture cap(cap_name);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open video source.\n";
    return -1;
  }

  // clicked_points 벡터 초기화 확인
  std::cout << "Initial clicked_points size: " << clicked_points.size() << std::endl;
  for (int i = 0; i < clicked_points.size(); ++i) {
    std::cout << "Platform " << i << " initial size: " << clicked_points[i].size() << std::endl;
  }

  std::cout << "📌 사용자 클릭으로 플랫폼 영역을 지정하세요 (총 "
            << PLATFORM_SIZE << "개).\n";
  wait_for_user_clicks(cap);
  
  // 선택 완료 후 clicked_points 상태 확인
  std::cout << "\n=== Final clicked_points status ===" << std::endl;
  for (int i = 0; i < clicked_points.size(); ++i) {
    std::cout << "Platform " << i << " has " << clicked_points[i].size() << " points: ";
    if (clicked_points[i].size() == 4) {
      for (int j = 0; j < 4; ++j) {
        std::cout << "(" << clicked_points[i][j].x << "," << clicked_points[i][j].y << ") ";
      }
    } else {
      std::cout << "INVALID - missing points!";
    }
    std::cout << std::endl;
  }
  std::cout << "===================================\n" << std::endl;

  cap.release();

  std::thread cap_thread(capture_thread);
  std::thread warp_thread_(warp_thread);
  std::thread mask_thread_(mask_thread);

// GUI 이벤트 처리를 위한 메인 루프 및 masked_queue imshow
std::shared_ptr<std::vector<cv::Mat>> masked;
std::vector<cv::Mat> last_masked; // 마지막으로 표시한 프레임 저장

// 창을 한 번만 생성
for (int i = 0; i < PLATFORM_SIZE; ++i) {
  std::string win_name = "Masked Platform " + std::to_string(i);
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(win_name, 800, 600);
}

auto frame_start = std::chrono::high_resolution_clock::now();
while (is_capture_thread_running) {
  if (masked_queue.try_pop(masked)) {
    last_masked = *masked;
    auto frame_end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start).count();
    std::cout << "[main] Frame output time: " << elapsed << " ms" << std::endl;
    frame_start = std::chrono::high_resolution_clock::now();
  }

  // 항상 마지막 프레임을 반복적으로 그려줌
  for (int i = 0; i < last_masked.size(); ++i) {
    if (!last_masked[i].empty()) {
      std::string win_name = "Masked Platform " + std::to_string(i);
      cv::Mat resized_masked;
      cv::resize(last_masked[i], resized_masked, cv::Size(800, 600));
      cv::imshow(win_name, resized_masked);
    }
  }

  int key = cv::waitKey(30); // 루프당 한 번만 호출
  if (key == 27) {
    is_capture_thread_running = false;
    break;
  }
}

  // 스레드 종료 대기
  cap_thread.join();
  warp_thread_.join();
  mask_thread_.join();

  // 모든 창 닫기
  cv::destroyAllWindows();
  
  std::cout << "Program done : " << total_frame << ":" 
    << frame_wasted << ":" << warped_wasted << ":" << masked_wasted << std::endl;

  return 0;
}

void capture_thread() {
  cv::VideoCapture cap(cap_name);

  if (!cap.isOpened()) {
    std::cout << "ERROR: Could not open camera\n";
    is_capture_thread_running = false;
    return;
  } else {
    is_capture_thread_running = true;
  }

  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);
  cv::Mat frame, balanced;
  std::shared_ptr<cv::Mat> garbage;
  while (true) {
    auto start_time = std::chrono::high_resolution_clock::now();
    if (!cap.read(frame) || frame.empty()) {
      break;
    }

    if (frame_queue.size() > MAX_QUEUE_SIZE) {
      frame_queue.try_pop(garbage);
      frame_wasted++;
    }

    auto_brightness_balance(frame, balanced);

    total_frame++;

    frame_queue.push(std::make_shared<cv::Mat>(std::move(balanced)));

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);  // CPU 낭비 방지
    }
  }

  cap.release();
  is_capture_thread_running = false;
}

bool is_every_rects_ready(std::vector<std::vector<cv::Point>>& rects){
  for(int i = 0; i < rects.size(); i++){
      if(rects[i].empty() || rects[i].size() != 4)
        return false;
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
    warped.clear(); // 초기화하지 않음

    // 조건문 수정: || 로 변경하고 로직 개선
    if (clicked_points.size() != PLATFORM_SIZE || !is_every_rects_ready(clicked_points)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;  // 아직 포인트가 준비 안됨
    }

    if (warped_queue.size() > MAX_QUEUE_SIZE) {
      warped_queue.try_pop(garbage);
      warped_wasted++;
    }

    if (frame_queue.try_pop(frame)) {
      warp_rectified_areas((*frame), warped, clicked_points);
      warped_queue.push(std::make_shared<std::vector<cv::Mat>>(std::move(warped)));
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        frame_duration - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);  // CPU 낭비 방지
    }
  }
}

void mask_thread() {

  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);

  std::shared_ptr<std::vector<cv::Mat>> frame;
  std::shared_ptr<std::vector<cv::Mat>> garbage;
  std::vector<cv::Mat> masked_1, masked_2;
  
  while (is_capture_thread_running || !warped_queue.empty()) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (masked_queue.size() > MAX_QUEUE_SIZE) {
      masked_queue.try_pop(garbage);
      masked_wasted++;
    }

    if (warped_queue.try_pop(frame)) {
      masked_1.resize(PLATFORM_SIZE);
      masked_2.resize(PLATFORM_SIZE);
      
      remove_achromatic_areas((*frame), masked_1);  // default : 0.15
      revive_white_areas(masked_1, masked_2);       // default : 95%

      masked_queue.push(std::make_shared<std::vector<cv::Mat>>(std::move(masked_2)));
    }
      
    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);  // CPU 낭비 방지
    }
  }

}

