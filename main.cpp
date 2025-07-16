// clone 하는 부분 최대한 출일 것, shared_ptr => 관련 글쓰기
#include <chrono>
#include <iostream>
#include <queue>
#include <thread>

//#include <libcamera/libcamera.h>
//#include <libcamera/libcamera_app.h>

#include "filters.hpp"
#include "safeQueue.hpp"

#define MAX_QUEUE_SIZE 1000

const char cap_name[] = "test.MP4";
//const char cap_name[] = 
//  "libcamerasrc ! video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! "
//  "videoconvert ! appsink";
bool is_capture_thread_running = true;

static ThreadSafeQueue<std::shared_ptr<cv::Mat>> frame_queue;
static std::vector<cv::Point> clicked_points;
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> warped_queue;
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> masked_queue;

void onMouseClick(int event, int x, int y, int flags, void *userdata);
void capture_thread();
void warp_thread();
void mask_thread();

/*variable for debugging */
unsigned int frame_wasted = 0;
unsigned int warped_wasted = 0;
unsigned int masked_wasted = 0;

int main() {
  cv::VideoCapture cap(cap_name);
  cv::Mat firstImage;
  cap >> firstImage;
  cap.release();

  cv::namedWindow("original", cv::WINDOW_NORMAL);
  cv::resizeWindow("original", 1000, 800);
  cv::setMouseCallback("original", onMouseClick, &clicked_points);
  cv::imshow("original", firstImage);

  while (clicked_points.size() != 4) {
    cv::waitKey(30);
  }

  std::thread tcapture(capture_thread);
  std::thread twarp(warp_thread);
  std::thread tmask(mask_thread);

   // 디버깅 윈도우 설정
  cv::namedWindow("mask", cv::WINDOW_NORMAL);
  cv::resizeWindow("mask", 1000, 800);

  // main에서 직접 imshow 실행
  std::shared_ptr<cv::Mat> result;
  while (is_capture_thread_running || !masked_queue.empty()) {
    if (masked_queue.try_pop(result)) {
      if (result && !result->empty()) {
        cv::imshow("mask", *result);
        std::cout << "left masked : " << masked_queue.size() << "\n";
      } else {
        std::cout << "Invalid or empty frame received.\n";
      }
    }

    char c = (char)cv::waitKey(30);  // 반드시 GUI 갱신을 위해 필요
    if (c == 'q') break;

    // 너무 자주 루프가 돌지 않게 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  tcapture.join();
  twarp.join();
  tmask.join();

  std::cout << masked_wasted << ' ' << warped_wasted << ' ' << masked_wasted << '\n';

  cv::destroyAllWindows();
  return 0;
}

void onMouseClick(int event, int x, int y, int flags, void *userdata) {
  if (event != cv::EVENT_LBUTTONDOWN) return;

  std::vector<cv::Point> *points_vec =
      static_cast<std::vector<cv::Point> *>(userdata);

  if (points_vec->size() >= 4) {
    std::cout << "🔄 Point list is full. Clearing list." << std::endl;
    points_vec->clear();
  }

  points_vec->push_back(cv::Point(x, y));
  std::cout << "📌 Point added: (" << x << ", " << y
            << "). Total points: " << points_vec->size() << std::endl;
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
  cv::Mat frame;
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
    frame_queue.push(std::make_shared<cv::Mat>(std::move(frame)));

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    std::cout << "org: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                     .count()
              << " : " << frame_queue.size() << '\n';
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);  // CPU 낭비 방지
    }
  }

  std::cout << "done!!" << '\n';
  cap.release();
  is_capture_thread_running = false;
}

void warp_thread() {
  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);

  cv::Mat warped;
  std::shared_ptr<cv::Mat> frame, garbage;
  while (is_capture_thread_running) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (clicked_points.size() != 4) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;  // 아직 포인트가 준비 안됨
    }

    if (warped_queue.size() > MAX_QUEUE_SIZE) {
      warped_queue.try_pop(garbage);
      warped_wasted++;
    }

    if (frame_queue.try_pop(frame)) {
      warp_rectified_area((*frame), warped, clicked_points);
      warped_queue.push(std::make_shared<cv::Mat>(std::move(warped)));
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    std::cout << "warp: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                     .count()
              << " : " << warped_queue.size() << '\n';
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);  // CPU 낭비 방지
    }
  }
}

void mask_thread() {
  const int target_fps = 30;
  const std::chrono::milliseconds frame_duration(1000 / target_fps);

  cv::Mat masked_1, masked_2;
  std::shared_ptr<cv::Mat> frame, garbage;
  while (is_capture_thread_running) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (masked_queue.size() > MAX_QUEUE_SIZE) {
      masked_queue.try_pop(garbage);
      masked_wasted++;
    }

    if (warped_queue.try_pop(frame)) {
      remove_achromatic_area((*frame), masked_1);  // default : 0.15
      revive_white_area(masked_1, masked_2);       // default : 95%
      masked_queue.push(std::make_shared<cv::Mat>(std::move(masked_2)));
    }

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    std::cout << "masked: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                     .count()
              << " : " << masked_queue.size() << '\n';
    auto sleep_time =
        frame_duration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);  // CPU 낭비 방지
    }
  }
}

// DEBUG
/*
std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            warp_rectified_area(frame, warped, clicked_points);
            remove_achromatic_area(warped, mask, 0.15);
            revive_white_area(mask, remask);
            std::chrono::system_clock::time_point end =
std::chrono::system_clock::now();; std::chrono::duration<double, std::milli>
msec = end - start;
*/
