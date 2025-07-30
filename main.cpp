/**
 * @file main.cpp
 * @brief 유닉스 도메인 소켓으로 ROI 설정과 영상 스트림을 받아 버스를 감지하고,
 * 정제된 감지 결과를 공유 메모리에 기록하는 메인 프로그램
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

// 시스템 헤더
#include <fcntl.h>       // 공유 메모리
#include <sys/mman.h>    // 공유 메모리
#include <sys/select.h>  // select
#include <sys/socket.h>  //소켓
#include <sys/stat.h>    // chmod
#include <sys/un.h>      // 유닉스 도메인 소켓
#include <unistd.h>

#include "checker.hpp"  // check_bus_platform, PixelRatioInfo
#include "filters.hpp"  // 차량 마스킹
#include "safeQueue.hpp"
#include "stop_status.hpp"  // StopStatus 구조체

#if 1
#include <X11/Xlib.h>  // XInitThreads
#endif

// 상수 정의
#define MAX_QUEUE_SIZE 2       // 큐사이즈가 클수록 딜레이 증가
#define MAX_PLATFORM_COUNT 20  // 최대 플랫폼 수

// 통신 경로
#define SHM_NAME_FRAME "/busbom_frame"    // live stream
#define SHM_NAME_STATUS "/busbom_status"  // bus stop status
#define SOCKET_PATH "/tmp/roi_socket"     // roi info

// 프레임 설정
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define FRAME_CHANNELS 3

// --- 전역 설정 ---
const int TARGET_FPS = 15;
const std::chrono::milliseconds FRAME_DURATION(1000 / TARGET_FPS);
const char* STATION_ID = "101000004";  // 정류장 ID

// --- 상태 안정화(Debouncing)용 변수 ---
static int stable_status[MAX_PLATFORM_COUNT] = {0};
static int prev_stable_status[MAX_PLATFORM_COUNT] = {0};

// --- 시간 기반 정차 판단용 변수 ---
static std::chrono::steady_clock::time_point
    detection_start_time[MAX_PLATFORM_COUNT];
static int detection_loss_counter[MAX_PLATFORM_COUNT] = {
    0};                                      // 감지 손실 카운터
const double STABLE_TIME_THRESHOLD_S = 3.0;  // 정차로 판단하기 위한 시간 (초)
const int LOSS_TOLERANCE_CYCLES = 5;  // 감지 손실 허용 횟수 (사이클)

// --- 게이트 및 버스 카운팅용 변수 ---
static int entered_bus_count = 0;
static int exited_bus_count = 0;
static bool prev_entry_gate_status = false;
static bool prev_exit_gate_status = false;
static int entry_gate_loss_counter = 0;
static int exit_gate_loss_counter = 0;
const int GATE_LOSS_TOLERANCE_CYCLES = 3;  // 게이트 감지 손실 허용 횟수

// --- 입구/출구 영역 상태 추적용 변수 ---
static bool entry_area_filled = false;
static bool exit_area_filled = false;
static bool prev_entry_area_filled = false;
static bool prev_exit_area_filled = false;

// --- 출구 영역 안정화를 위한 변수 ---
static int exit_detection_counter = 0;
static const int EXIT_DETECTION_THRESHOLD = 3;  // 연속 3회 감지되면 출구로 인식

// --- 입구 영역 안정화를 위한 변수 ---
static int entry_detection_counter = 0;
static const int ENTRY_DETECTION_THRESHOLD = 5;  // 연속 5회 감지되면 입구로 인식 (더 엄격)

// --- 전역 변수 ---
unsigned int PLATFORM_SIZE = 0;
std::vector<std::vector<cv::Point>> platform_rois;
bool BUS_PLATFORM_STATUS[MAX_PLATFORM_COUNT] = {false};
std::mutex rois_mutex;

std::atomic<bool> running(true);
std::atomic<bool> is_config_ready(false);  // ROI 설정 완료 여부 플래그
std::atomic<bool> initial_bus_count_set(false);  // 초기 버스 수 설정 완료 여부 플래그
std::atomic<bool> bus_count_corrected(false);  // 버스 수 보정 완료 여부 플래그

// --- 공유 메모리 포인터 (상태 출력용) ---
int status_shm_fd = -1;
StopStatus* status_shm_ptr = nullptr;

// --- 데이터 큐 ---
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> frame_queue;
static ThreadSafeQueue<std::shared_ptr<std::vector<cv::Mat>>> warped_queue;
static ThreadSafeQueue<std::shared_ptr<std::vector<cv::Mat>>> masked_queue;
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> debug_frame_queue;  // 디버깅용

// --- 디버깅용 카운터 ---
unsigned int total_frame = 0;
unsigned int frame_wasted = 0;
unsigned int warped_wasted = 0;
unsigned int masked_wasted = 0;

// --- 입출구 관련 변수 ---
// [추가] 입출구 감지 상태 및 시간 보정 변수
bool prev_entry_roi_filled = false;
bool prev_exit_roi_filled = false;
std::chrono::steady_clock::time_point last_entry_detected;
std::chrono::steady_clock::time_point last_exit_detected;
const double ENTRY_DELAY_SEC = 2.5;
const double EXIT_DELAY_SEC = 2.5;

// [추가] 연속 감지 방지를 위한 변수
std::chrono::steady_clock::time_point last_entry_counted;
std::chrono::steady_clock::time_point last_exit_counted;
const double MIN_ENTRY_INTERVAL_SEC = 10.0;  // 최소 진입 간격 (초) - 늘림
const double MIN_EXIT_INTERVAL_SEC = 3.0;   // 최소 진출 간격 (초)

// [추가] 정류장 내 추정 버스 수
int estimated_bus_in_station = 0;

void update_bus_count_by_entry_exit(unsigned int platform_count, const bool* raw_status);
void initialize_platform_status(unsigned int platform_count);
void set_initial_bus_count(unsigned int platform_count, const bool* raw_status);
void correct_bus_count_if_needed(unsigned int platform_count, const bool* raw_status);
void reset_bus_count_to_platform_status(unsigned int platform_count, const bool* raw_status);
void process_bus_status(unsigned int stop_platform_count,
                        const bool* raw_status);
void process_gate_status(unsigned int total_platform_count,
                         const bool* raw_status);
void process_entry_exit_status(unsigned int total_platform_count,
                              const bool* raw_status);
void update_shared_status(unsigned int platform_count);
void video_read_thread(const std::string& video_filename);

void signal_handler(int signum) {
  std::cout << "\nTermination signal received. Shutting down..." << std::endl;
  running.store(false);
}

/**
 * @brief CGI 스크립트로부터 유닉스 도메인 소켓을 통해 바이너리 ROI 설정을
 * 수신하는 스레드.
 */
void receive_roi_config_thread() {
  int server_fd;
  struct sockaddr_un address;

  if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == 0) {
    perror("socket failed");
    return;
  }

  memset(&address, 0, sizeof(struct sockaddr_un));
  address.sun_family = AF_UNIX;
  strncpy(address.sun_path, SOCKET_PATH, sizeof(address.sun_path) - 1);
  unlink(SOCKET_PATH);

  if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    perror("bind failed");
    close(server_fd);
    return;
  }

  if (chmod(SOCKET_PATH, 0777) < 0) {
    perror("chmod failed");
    close(server_fd);
    unlink(SOCKET_PATH);
    return;
  }

  if (listen(server_fd, 5) < 0) {
    perror("listen failed");
    close(server_fd);
    unlink(SOCKET_PATH);
    return;
  }

  while (running.load()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(server_fd, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 1;  // 1초 타임아웃
    timeout.tv_usec = 0;

    int activity = select(server_fd + 1, &read_fds, NULL, NULL, &timeout);

    if ((activity < 0) && (errno != EINTR)) {
      perror("select error");
      break;
    }

    if (FD_ISSET(server_fd, &read_fds)) {
      int client_fd = accept(server_fd, NULL, NULL);
      if (client_fd < 0) {
        if (running.load()) perror("accept failed");
        break;
      }

      std::cout << "CGI client connected. Receiving ROI data..." << std::endl;

      std::vector<char> buffer;
      char temp_buf[4096];
      ssize_t bytes_read;
      while ((bytes_read = read(client_fd, temp_buf, sizeof(temp_buf))) > 0) {
        buffer.insert(buffer.end(), temp_buf, temp_buf + bytes_read);
      }

      try {
        std::vector<std::vector<cv::Point>> temp_rois;
        size_t offset = 0;

        auto read_from_buffer = [&](void* dest, size_t len) {
          if (offset + len > buffer.size())
            throw std::runtime_error("Buffer underflow");
          memcpy(dest, buffer.data() + offset, len);
          offset += len;
        };

        uint32_t stop_count;
        read_from_buffer(&stop_count, sizeof(stop_count));

        if (stop_count < 2) {
          throw std::runtime_error("ROI count must be at least 2 for gates.");
        }

        for (uint32_t i = 0; i < stop_count; ++i) {
          uint32_t point_count;
          read_from_buffer(&point_count, sizeof(point_count));
          std::vector<cv::Point> roi;
          for (uint32_t j = 0; j < point_count; ++j) {
            int32_t x, y;
            read_from_buffer(&x, sizeof(x));
            read_from_buffer(&y, sizeof(y));
            roi.emplace_back(x, y);
          }
          temp_rois.push_back(roi);
        }

        std::lock_guard<std::mutex> lock(rois_mutex);
        platform_rois = temp_rois;
        PLATFORM_SIZE = platform_rois.size();

        // 플랫폼 상태 및 공유 메모리 초기화
        initialize_platform_status(PLATFORM_SIZE);

        std::cout << "✅ ROI configuration updated. Total platforms: "
                  << PLATFORM_SIZE << std::endl;
        std::cout << "   - Stop Platforms: "
                  << (PLATFORM_SIZE > 1 ? PLATFORM_SIZE - 2 : 0) << std::endl;
        std::cout << "   - Exit Gate: Platform " << PLATFORM_SIZE - 2
                  << std::endl;
        std::cout << "   - Entry Gate: Platform " << PLATFORM_SIZE - 1
                  << std::endl;
        is_config_ready = true;

      } catch (const std::exception& e) {
        std::cerr << "Error during deserialization: " << e.what() << std::endl;
      }
      close(client_fd);
    }
  }
  close(server_fd);
  unlink(SOCKET_PATH);
}

/**
 * @brief 공유 메모리에서 지속적으로 영상 프레임을 읽어와 큐에 넣는 스레드 함수
 */
void shm_read_thread() {
  const size_t frame_size = FRAME_WIDTH * FRAME_HEIGHT * FRAME_CHANNELS;
  int shm_fd = -1;

  std::cout << "Trying to connect to shared memory " << SHM_NAME_FRAME << "..."
            << std::endl;
  while (shm_fd == -1 && running.load()) {
    shm_fd = shm_open(SHM_NAME_FRAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  if (!running.load()) return;
  std::cout << "✅ Connected to shared memory." << std::endl;

  void* ptr = mmap(0, frame_size, PROT_READ, MAP_SHARED, shm_fd, 0);
  if (ptr == MAP_FAILED) {
    perror("shm_read_thread mmap");
    close(shm_fd);
    running = false;
    return;
  }

  cv::Mat shared_frame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, ptr);
  cv::Mat balanced;
  std::shared_ptr<cv::Mat> garbage;

  while (running.load()) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (shared_frame.empty() || shared_frame.data == (uchar*)-1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    cv::Mat local_frame;
    shared_frame.copyTo(local_frame);

    // ----- Debug -----
    cv::Mat debug_frame = local_frame.clone();
    {
      std::lock_guard<std::mutex> lock(rois_mutex);
      if (!platform_rois.empty()) {
        // 일반 플랫폼은 초록색, 게이트는 다른 색으로 표시
        for (size_t i = 0; i < platform_rois.size(); ++i) {
          cv::Scalar color;
          if (i == PLATFORM_SIZE - 2)
            color = cv::Scalar(0, 0, 255);  // Exit=Red
          else if (i == PLATFORM_SIZE - 1)
            color = cv::Scalar(255, 0, 0);  // Entry=Blue
          else
            color = cv::Scalar(0, 255, 0);  // Platform=Green
          cv::polylines(debug_frame,
                        std::vector<std::vector<cv::Point>>{platform_rois[i]},
                        true, color, 2);
        }
      }
    }
    if (debug_frame_queue.size() < 10) {
      debug_frame_queue.push(std::make_shared<cv::Mat>(std::move(debug_frame)));
    }
    // ----------------

    if (frame_queue.size() > MAX_QUEUE_SIZE) {
      frame_queue.try_pop(garbage);
      frame_wasted++;
    }

    auto_brightness_balance(local_frame, balanced);
    total_frame++;
    frame_queue.push(std::make_shared<cv::Mat>(std::move(balanced)));

    auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
    auto sleep_time =
        FRAME_DURATION -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
  }

  munmap(ptr, frame_size);
  close(shm_fd);
}

/**
 * @brief 상태 정보를 기록할 공유 메모리를 설정합니다.
 * @return 성공 시 true, 실패 시 false
 */
bool setup_status_shm() {
  umask(0);
  status_shm_fd = shm_open(SHM_NAME_STATUS, O_CREAT | O_RDWR, 0666);
  if (status_shm_fd == -1) {
    perror("shm_open for status failed");
    return false;
  }

  if (ftruncate(status_shm_fd, sizeof(StopStatus)) == -1) {
    perror("ftruncate for status failed");
    close(status_shm_fd);
    shm_unlink(SHM_NAME_STATUS);
    return false;
  }

  status_shm_ptr = (StopStatus*)mmap(0, sizeof(StopStatus), PROT_WRITE,
                                     MAP_SHARED, status_shm_fd, 0);
  if (status_shm_ptr == MAP_FAILED) {
    perror("mmap for status failed");
    close(status_shm_fd);
    shm_unlink(SHM_NAME_STATUS);
    status_shm_ptr = nullptr;
    return false;
  }

  std::cout << "✅ Status shared memory '" << SHM_NAME_STATUS << "' is ready."
            << std::endl;
  return true;
}

/**
 * @brief 프로그램 종료 시 상태 공유 메모리를 해제합니다.
 */
void cleanup_status_shm() {
  if (status_shm_ptr != nullptr && status_shm_ptr != MAP_FAILED) {
    munmap(status_shm_ptr, sizeof(StopStatus));
  }
  if (status_shm_fd != -1) {
    close(status_shm_fd);
  }
  shm_unlink(SHM_NAME_STATUS);
  std::cout << "Status shared memory cleaned up." << std::endl;
}

/**
 * @brief ROI 설정에 따라 플랫폼 상태 및 공유 메모리를 초기화합니다.
 */
void initialize_platform_status(unsigned int platform_count) {
  // 카운팅 관련 변수 초기화
  entered_bus_count = 0;
  exited_bus_count = 0;
  prev_entry_gate_status = false;
  prev_exit_gate_status = false;
  entry_gate_loss_counter = 0;
  exit_gate_loss_counter = 0;

  // 입구/출구 영역 상태 초기화
  entry_area_filled = false;
  exit_area_filled = false;
  prev_entry_area_filled = false;
  prev_exit_area_filled = false;
  exit_detection_counter = 0;
  entry_detection_counter = 0;
  
  // 연속 감지 방지 변수 초기화
  last_entry_counted = std::chrono::steady_clock::time_point();
  last_exit_counted = std::chrono::steady_clock::time_point();

  // 플랫폼 상태 초기화
  for (unsigned int i = 0; i < MAX_PLATFORM_COUNT; ++i) {
    stable_status[i] = 0;
    prev_stable_status[i] = 0;
    detection_start_time[i] = std::chrono::steady_clock::time_point();
    detection_loss_counter[i] = 0;
  }

  // 초기 버스 수는 별도 함수에서 설정
  estimated_bus_in_station = 0;

  if (status_shm_ptr == nullptr) return;

  // 공유 메모리 초기화
  for (unsigned int i = 0; i < MAX_PLATFORM_COUNT; ++i) {
    if (i < platform_count) {
      status_shm_ptr->platform_status[i] =
          0;  // 사용 플랫폼은 0(empty)으로 초기화
    } else {
      status_shm_ptr->platform_status[i] = -1;  // 미사용 플랫폼은 -1로 초기화
    }
  }
  strncpy(status_shm_ptr->station_id, STATION_ID,
          sizeof(status_shm_ptr->station_id) - 1);
  status_shm_ptr->entered_bus_count = 0;
  status_shm_ptr->exited_bus_count = 0;
  status_shm_ptr->current_bus_count = 0;
  status_shm_ptr->updated_at = time(nullptr);
}

/**
 * @brief 원본 감지 결과를 안정화된 상태로 변환합니다. (정차 플랫폼 전용)
 */
void process_bus_status(unsigned int stop_platform_count,
                        const bool* raw_status) {
  for (unsigned int i = 0; i < stop_platform_count; ++i) {
    if (raw_status[i]) {              // 버스가 감지된 경우
      detection_loss_counter[i] = 0;  // 감지되었으므로 손실 카운터 리셋

      // 이전에 정차 상태가 아니었을 때 (새로 감지 시작)
      if (stable_status[i] == 0) {
        // 감지 시작 시간을 기록한 적이 없다면, 현재 시간을 기록
        if (detection_start_time[i].time_since_epoch().count() == 0) {
          detection_start_time[i] = std::chrono::steady_clock::now();
        }

        // 경과 시간 계산
        auto elapsed_time =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - detection_start_time[i]);

        // 경과 시간이 임계값을 넘으면 정차로 확정
        if (elapsed_time.count() >= STABLE_TIME_THRESHOLD_S) {
          stable_status[i] = 1;
        }
      }
    } else {  // 버스가 감지되지 않은 경우
      detection_loss_counter[i]++;

      // 허용된 손실 횟수를 초과하면, 모든 상태를 완전히 리셋
      if (detection_loss_counter[i] > LOSS_TOLERANCE_CYCLES) {
        stable_status[i] = 0;
        detection_start_time[i] = std::chrono::steady_clock::time_point();
      }
      // 허용치 이내라면, 아무것도 하지 않고 타이머를 유지 (다음 감지를 기다림)
    }
  }
}

/**
 * @brief 게이트의 상태를 처리하고 버스 진입/진출을 카운트합니다.
 */
void process_gate_status(unsigned int total_platform_count,
                         const bool* raw_status) {
  if (total_platform_count < 2) return;

  unsigned int exit_gate_idx = total_platform_count - 2;
  unsigned int entry_gate_idx = total_platform_count - 1;

  // --- 시간적 보정을 포함한 현재 게이트 상태 결정 ---
  bool current_entry_gate_status;
  if (raw_status[entry_gate_idx]) {
    current_entry_gate_status = true;
    entry_gate_loss_counter = 0;
  } else {
    entry_gate_loss_counter++;
    if (entry_gate_loss_counter > GATE_LOSS_TOLERANCE_CYCLES) {
      current_entry_gate_status = false;
    } else {
      current_entry_gate_status = prev_entry_gate_status;  // 이전 상태 유지
    }
  }

  bool current_exit_gate_status;
  if (raw_status[exit_gate_idx]) {
    current_exit_gate_status = true;
    exit_gate_loss_counter = 0;
  } else {
    exit_gate_loss_counter++;
    if (exit_gate_loss_counter > GATE_LOSS_TOLERANCE_CYCLES) {
      current_exit_gate_status = false;
    } else {
      current_exit_gate_status = prev_exit_gate_status;  // 이전 상태 유지
    }
  }

  auto now = std::chrono::steady_clock::now();
  
  // --- Rising Edge(False -> True) 감지로 카운트 ---
  if (!prev_entry_gate_status && current_entry_gate_status) {
    // 최소 간격 확인
    auto time_since_last_entry = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - last_entry_counted).count();
    
    if (time_since_last_entry >= MIN_ENTRY_INTERVAL_SEC) {
      entered_bus_count++;  // 디버깅용 카운트
      estimated_bus_in_station++;  // 현재 버스 수 직접 증가
      last_entry_counted = now;
      std::cout << "🚌 Bus entered through entry gate (gate detection) - Current: " << estimated_bus_in_station << std::endl;
    } else {
      std::cout << "⚠️  Gate entry detection ignored (too soon: " 
                << std::fixed << std::setprecision(1) << time_since_last_entry 
                << "s < " << MIN_ENTRY_INTERVAL_SEC << "s)" << std::endl;
    }
  }
  
  if (!prev_exit_gate_status && current_exit_gate_status) {
    // 최소 간격 확인
    auto time_since_last_exit = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - last_exit_counted).count();
    
    if (time_since_last_exit >= MIN_EXIT_INTERVAL_SEC) {
      exited_bus_count++;  // 디버깅용 카운트
      estimated_bus_in_station--;  // 현재 버스 수 직접 감소
      if (estimated_bus_in_station < 0) estimated_bus_in_station = 0;  // 음수 방지
      last_exit_counted = now;
      std::cout << "🚌 Bus exited through exit gate (gate detection) - Current: " << estimated_bus_in_station << std::endl;
    } else {
      std::cout << "⚠️  Gate exit detection ignored (too soon: " 
                << std::fixed << std::setprecision(1) << time_since_last_exit 
                << "s < " << MIN_EXIT_INTERVAL_SEC << "s)" << std::endl;
    }
  }

  // 다음 사이클을 위해 현재 상태를 이전 상태로 저장
  prev_entry_gate_status = current_entry_gate_status;
  prev_exit_gate_status = current_exit_gate_status;
}

/**
 * @brief ROI 설정 후 초기 버스 수를 플랫폼 중 filled된 개수로 설정합니다.
 */
void set_initial_bus_count(unsigned int platform_count, const bool* raw_status) {
  if (platform_count < 2) return;

  unsigned int stop_platform_count = platform_count - 2;
  estimated_bus_in_station = 0;
  
  for (unsigned int i = 0; i < stop_platform_count; ++i) {
    if (raw_status[i]) {
      estimated_bus_in_station++;
      stable_status[i] = 1;  // 이미 정차 상태로 설정
      std::cout << "  Platform " << i << ": BUS DETECTED (initial)" << std::endl;
    }
  }
  
  std::cout << "🚌 Initial bus count set to: " << estimated_bus_in_station 
            << " (based on " << stop_platform_count << " platforms)" << std::endl;
}

/**
 * @brief 현재 버스 수와 실제 감지된 버스 수를 비교하여 필요시 보정합니다.
 */
void correct_bus_count_if_needed(unsigned int platform_count, const bool* raw_status) {
  if (platform_count < 2) return;

  unsigned int stop_platform_count = platform_count - 2;
  int detected_bus_count = 0;
  
  // 현재 플랫폼에서 감지된 버스 수 계산
  for (unsigned int i = 0; i < stop_platform_count; ++i) {
    if (raw_status[i]) {
      detected_bus_count++;
    }
  }
  
  // 현재 추정 버스 수와 감지된 버스 수 비교
  if (detected_bus_count != estimated_bus_in_station) {
    std::cout << "🔧 Bus count correction needed:" << std::endl;
    std::cout << "  Current estimated: " << estimated_bus_in_station << std::endl;
    std::cout << "  Actually detected: " << detected_bus_count << std::endl;
    
    // 감지된 버스 수로 보정
    int correction = detected_bus_count - estimated_bus_in_station;
    estimated_bus_in_station = detected_bus_count;
    
    // 진입/진출 카운트도 보정
    if (correction > 0) {
      // 더 많은 버스가 감지되었으면 진입 카운트 증가
      entered_bus_count += correction;
      std::cout << "  📈 Entry count adjusted by +" << correction << std::endl;
    } else if (correction < 0) {
      // 더 적은 버스가 감지되었으면 진출 카운트 증가
      exited_bus_count += (-correction);
      std::cout << "  📉 Exit count adjusted by +" << (-correction) << std::endl;
    }
    
    std::cout << "  ✅ Bus count corrected to: " << estimated_bus_in_station << std::endl;
  }
}

/**
 * @brief R키를 눌렀을 때 호출되는 보정 함수. 현재 버스 수를 채워진 플랫폼 수로 맞추고 진입/진출 카운트를 0으로 리셋합니다.
 */
void reset_bus_count_to_platform_status(unsigned int platform_count, const bool* raw_status) {
  if (platform_count < 2) return;

  unsigned int stop_platform_count = platform_count - 2;
  int detected_bus_count = 0;
  
  // 현재 플랫폼에서 감지된 버스 수 계산
  for (unsigned int i = 0; i < stop_platform_count; ++i) {
    if (raw_status[i]) {
      detected_bus_count++;
    }
  }
  
  std::cout << "\n🔄 Manual bus count reset (R key pressed):" << std::endl;
  std::cout << "  Previous estimated: " << estimated_bus_in_station << std::endl;
  std::cout << "  Previous entered: " << entered_bus_count << std::endl;
  std::cout << "  Previous exited: " << exited_bus_count << std::endl;
  std::cout << "  Currently detected: " << detected_bus_count << std::endl;
  
  // 현재 버스 수를 감지된 플랫폼 수로 설정
  estimated_bus_in_station = detected_bus_count;
  
  // 진입/진출 카운트를 0으로 리셋
  entered_bus_count = 0;
  exited_bus_count = 0;
  
  std::cout << "  ✅ Reset completed:" << std::endl;
  std::cout << "    Current bus count: " << estimated_bus_in_station << std::endl;
  std::cout << "    Entry count: " << entered_bus_count << std::endl;
  std::cout << "    Exit count: " << exited_bus_count << std::endl;
}



/**
 * @brief 입구/출구 영역의 상태 변화를 감지하여 버스 진입/진출을 추적합니다.
 */
void process_entry_exit_status(unsigned int total_platform_count,
                              const bool* raw_status) {
  if (total_platform_count < 2) return;

  unsigned int exit_gate_idx = total_platform_count - 2;
  unsigned int entry_gate_idx = total_platform_count - 1;

  // 현재 입구/출구 영역 상태
  bool current_entry_filled = raw_status[entry_gate_idx];
  bool current_exit_filled = raw_status[exit_gate_idx];

  // 이전 상태와 비교하여 변화 감지
  bool entry_state_changed = (prev_entry_area_filled != current_entry_filled);
  bool exit_state_changed = (prev_exit_area_filled != current_exit_filled);

  // 출구 영역 안정화 로직
  if (current_exit_filled) {
    exit_detection_counter++;
  } else {
    exit_detection_counter = 0;
  }

  // 입구 영역 안정화 로직
  if (current_entry_filled) {
    entry_detection_counter++;
  } else {
    entry_detection_counter = 0;
  }

  auto now = std::chrono::steady_clock::now();
  
  // 출구 영역이 빈 상태였다가 안정적으로 찬 상태가 되면 버스가 나가는 것으로 인식
  if (!prev_exit_area_filled && exit_detection_counter >= EXIT_DETECTION_THRESHOLD) {
    // 최소 간격 확인
    auto time_since_last_exit = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - last_exit_counted).count();
    
    if (time_since_last_exit >= MIN_EXIT_INTERVAL_SEC) {
      exited_bus_count++;  // 디버깅용 카운트
      estimated_bus_in_station--;  // 현재 버스 수 직접 감소
      if (estimated_bus_in_station < 0) estimated_bus_in_station = 0;  // 음수 방지
      last_exit_counted = now;
      std::cout << "🚌 Bus exited through exit gate (stable detection) - Current: " << estimated_bus_in_station << std::endl;
    } else {
      std::cout << "⚠️  Exit detection ignored (too soon: " 
                << std::fixed << std::setprecision(1) << time_since_last_exit 
                << "s < " << MIN_EXIT_INTERVAL_SEC << "s)" << std::endl;
    }
  }

  // 입구 영역이 찬 상태였다가 안정적으로 빈 상태가 되면 버스가 맨 뒤 플랫폼에 진입한 것으로 판단
  if (prev_entry_area_filled && entry_detection_counter == 0) {
    // 최소 간격 확인
    auto time_since_last_entry = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - last_entry_counted).count();
    
    if (time_since_last_entry >= MIN_ENTRY_INTERVAL_SEC) {
      entered_bus_count++;  // 디버깅용 카운트
      estimated_bus_in_station++;  // 현재 버스 수 직접 증가
      last_entry_counted = now;
      std::cout << "🚌 Bus entered through entry gate (stable detection) - Current: " << estimated_bus_in_station << std::endl;
    } else {
      std::cout << "⚠️  Entry detection ignored (too soon: " 
                << std::fixed << std::setprecision(1) << time_since_last_entry 
                << "s < " << MIN_ENTRY_INTERVAL_SEC << "s)" << std::endl;
    }
  }

  // 현재 정류장 내 버스 수는 직접 증감 방식으로 관리 (진입/진출 카운트는 디버깅용)
  if (estimated_bus_in_station < 0) {
    estimated_bus_in_station = 0;  // 음수가 되지 않도록 보정
  }

  // 다음 사이클을 위해 현재 상태를 이전 상태로 저장
  // 입구 영역은 안정화된 상태로만 업데이트
  if (entry_detection_counter >= ENTRY_DETECTION_THRESHOLD) {
    prev_entry_area_filled = true;
  } else if (entry_detection_counter == 0) {
    prev_entry_area_filled = false;
  }
  
  // 출구 영역은 안정화된 상태로만 업데이트
  if (exit_detection_counter >= EXIT_DETECTION_THRESHOLD) {
    prev_exit_area_filled = true;
  } else if (exit_detection_counter == 0) {
    prev_exit_area_filled = false;
  }

  // 디버그 정보 출력 (상태 변화 시)
  if (exit_state_changed) {
    std::cout << "🔍 Exit gate state changed: " 
              << (prev_exit_area_filled ? "FILLED" : "EMPTY") 
              << " -> " 
              << (current_exit_filled ? "FILLED" : "EMPTY") 
              << " (counter: " << exit_detection_counter << "/" << EXIT_DETECTION_THRESHOLD << ")" << std::endl;
  }
  
  if (entry_state_changed) {
    std::cout << "🔍 Entry gate state changed: " 
              << (prev_entry_area_filled ? "FILLED" : "EMPTY") 
              << " -> " 
              << (current_entry_filled ? "FILLED" : "EMPTY") 
              << " (counter: " << entry_detection_counter << "/" << ENTRY_DETECTION_THRESHOLD << ")" << std::endl;
  }
}

/**
 * @brief 안정화된 상태를 공유 메모리에 업데이트합니다.
 */
void update_shared_status(unsigned int platform_count) {
  if (status_shm_ptr == nullptr) return;

  // 정차 플랫폼 상태 업데이트
  unsigned int stop_platform_count =
      (platform_count >= 2) ? platform_count - 2 : 0;
  for (unsigned int i = 0; i < stop_platform_count; ++i) {
    status_shm_ptr->platform_status[i] = stable_status[i];
  }
  // 게이트 상태는 정차 여부가 아니므로 별도로 기록 (필요시 -1 등으로)
  if (platform_count >= 2) {
    status_shm_ptr->platform_status[platform_count - 2] = -1;  // Exit Gate
    status_shm_ptr->platform_status[platform_count - 1] = -1;  // Entry Gate
  }

  // 버스 카운트 정보 업데이트
  status_shm_ptr->entered_bus_count = entered_bus_count;
  status_shm_ptr->exited_bus_count = exited_bus_count;
  status_shm_ptr->current_bus_count = estimated_bus_in_station;

  status_shm_ptr->updated_at = time(nullptr);
}

/**
 * @brief 프레임을 큐에서 가져와 투시 변환(warp)을 수행하는 스레드 함수.
 */
void warp_thread() {
  std::shared_ptr<cv::Mat> frame;
  std::shared_ptr<cv::Mat> garbage;

  while (running.load() || !frame_queue.empty()) {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<cv::Point>> local_rois;
    {
      std::lock_guard<std::mutex> lock(rois_mutex);
      local_rois = platform_rois;
    }

    if (local_rois.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    while (frame_queue.size() > MAX_QUEUE_SIZE && running.load()) {
      frame_queue.try_pop(garbage);
      frame_wasted++;
    }

    if (frame_queue.try_pop(frame)) {
      std::vector<cv::Mat> warped;
      warp_rectified_areas((*frame), warped, local_rois);
      warped_queue.push(
          std::make_shared<std::vector<cv::Mat>>(std::move(warped)));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

/**
 * @brief 변환된 이미지를 가져와 색상 필터링(마스킹)을 수행하는 스레드 함수.
 */
void mask_thread() {
  std::shared_ptr<std::vector<cv::Mat>> frame;
  std::shared_ptr<std::vector<cv::Mat>> garbage;

  while (running.load() || !warped_queue.empty()) {
    auto start_time = std::chrono::high_resolution_clock::now();

    unsigned int local_platform_size;
    {
      std::lock_guard<std::mutex> lock(rois_mutex);
      local_platform_size = platform_rois.size();
    }

    if (local_platform_size == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    while (warped_queue.size() > MAX_QUEUE_SIZE && running.load()) {
      warped_queue.try_pop(garbage);
      warped_wasted++;
    }

    if (warped_queue.try_pop(frame)) {
      std::vector<cv::Mat> final_masks;
      generate_bus_mask(*frame, final_masks);
      masked_queue.push(
          std::make_shared<std::vector<cv::Mat>>(std::move(final_masks)));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

/**
 * @brief 영상 파일에서 프레임을 읽어와 큐에 넣는 스레드 함수
 */
void video_read_thread(const std::string& video_path) {
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open viedo file: " << video_path << std::endl;
    running.store(false);
    return;
  }

  std::cout << "success open the file: " << video_path << std::endl;

  cv::Mat balanced;
  cv::Mat frame, resized;
  std::shared_ptr<cv::Mat> garbage;

  while (running.load()) {
    auto start = std::chrono::high_resolution_clock::now();

    if (!cap.read(frame) || frame.empty()) break;

    cv::resize(frame, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

    // ---- 디버그용 ROI 오버레이 ----
    {
      std::lock_guard<std::mutex> lock(rois_mutex);
      if (!platform_rois.empty()) {
        // 일반 플랫폼은 초록색, 게이트는 다른 색으로 표시
        for (size_t i = 0; i < platform_rois.size(); ++i) {
          cv::Scalar color;
          if (i == PLATFORM_SIZE - 2)
            color = cv::Scalar(0, 0, 255);  // Exit=Red
          else if (i == PLATFORM_SIZE - 1)
            color = cv::Scalar(255, 0, 0);  // Entry=Blue
          else
            color = cv::Scalar(0, 255, 0);  // Platform=Green
          cv::polylines(resized,
                        std::vector<std::vector<cv::Point>>{platform_rois[i]},
                        true, color, 2);
        }
      }
    }

    if (debug_frame_queue.size() < 10) {
      debug_frame_queue.push(std::make_shared<cv::Mat>(resized.clone()));
    }
    // ----------------------------

    if (frame_queue.size() > MAX_QUEUE_SIZE) {
      frame_queue.try_pop(garbage);
      frame_wasted++;
    }

    auto_brightness_balance(resized, balanced);
    total_frame++;
    frame_queue.push(std::make_shared<cv::Mat>(std::move(balanced)));

    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto sleep_time =
        FRAME_DURATION -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
  }

  cap.release();
  running.store(false);
}

int main(int argc, char* argv[]) {
  XInitThreads();
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  if (!setup_status_shm()) {
    return 1;
  }

  std::thread roi_config_thread(receive_roi_config_thread);
  std::thread reader_thread;

  if (argc > 1) {
    std::cout << "Starting in video file mode with: " << argv[1] << std::endl;
    reader_thread = std::thread(
        video_read_thread,
        "file://home/Qwd/platform_observer/video/" + std::string(argv[1]));
  } else {
    std::cout << "Starting in shared memory mode." << std::endl;
    reader_thread = std::thread(shm_read_thread);
  }

  std::thread warp_thread_;
  std::thread mask_thread_;
  bool processing_threads_started = false;

  std::cout << "✅ Main process running. Waiting for initial CGI configuration "
               "via socket..."
            << std::endl;

  std::vector<cv::Mat> last_masked_frames;
  cv::namedWindow("Debug View");
  cv::moveWindow("Debug View", 600, 280);

  while (running.load()) {
    if (is_config_ready.load() && !processing_threads_started) {
      std::cout
          << "✅ Initial configuration received. Starting processing threads..."
          << std::endl;
      warp_thread_ = std::thread(warp_thread);
      mask_thread_ = std::thread(mask_thread);
      processing_threads_started = true;
      initial_bus_count_set = false;  // 초기 버스 수 설정 플래그 리셋
      bus_count_corrected = false;    // 버스 수 보정 플래그 리셋
    }

    std::shared_ptr<cv::Mat> debug_frame;
    if (debug_frame_queue.try_pop(debug_frame)) {
      // 디버그 뷰에 카운트 정보 표시
      std::string text = "Current: " + std::to_string(estimated_bus_in_station) + " buses";
      cv::putText(*debug_frame, text, cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2,
                  cv::LINE_AA);
      
      // 조작키 안내 표시
      std::string controls = "Controls: Q=Quit, R=Reset Bus Count";
      cv::putText(*debug_frame, controls, cv::Point(10, 60),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2,
                  cv::LINE_AA);
      cv::imshow("Debug View", *debug_frame);
    }

    if (is_config_ready.load() && PLATFORM_SIZE >= 2) {
      std::shared_ptr<std::vector<cv::Mat>> masked;
      if (masked_queue.try_pop(masked)) {
        // 픽셀 비율 정보를 저장할 구조체
        static PixelRatioInfo ratio_info;
        check_bus_platform_with_ratios(*masked, BUS_PLATFORM_STATUS, ratio_info, 0.3, 0.15);

        // 첫 번째 프레임에서 초기 버스 수 설정
        if (!initial_bus_count_set.load()) {
          set_initial_bus_count(PLATFORM_SIZE, BUS_PLATFORM_STATUS);
          initial_bus_count_set = true;
          std::cout << "🎯 Initial bus count has been set based on current platform status" << std::endl;
        }
        
        // 초기 버스 수 설정 후 한 번만 보정 수행
        if (initial_bus_count_set.load() && !bus_count_corrected.load()) {
          correct_bus_count_if_needed(PLATFORM_SIZE, BUS_PLATFORM_STATUS);
          bus_count_corrected = true;
          std::cout << "🎯 Bus count correction completed" << std::endl;
        }

        unsigned int stop_platform_count = PLATFORM_SIZE - 2;
        process_bus_status(stop_platform_count, BUS_PLATFORM_STATUS);
        process_gate_status(PLATFORM_SIZE, BUS_PLATFORM_STATUS);
        process_entry_exit_status(PLATFORM_SIZE, BUS_PLATFORM_STATUS);

        update_shared_status(PLATFORM_SIZE);

        // 콘솔 디버그 출력
        std::cout << "\n--- Bus Count ---" << std::endl;
        std::cout << "  Current: " << estimated_bus_in_station << " buses" << std::endl;
        std::cout << "  [Debug] Entered: " << entered_bus_count
                  << " | Exited: " << exited_bus_count
                  << std::endl;
        std::cout << "--- Platform Status ---" << std::endl;
        for (unsigned int i = 0; i < stop_platform_count; i++) {
          std::cout << "  Platform " << i << ": "
                    << (stable_status[i] ? "BUS STOPPED" : "Empty")
                    << std::endl;
        }
        std::cout << "--- Gate Status ---" << std::endl;
        std::cout << "  Entry Gate: " << (BUS_PLATFORM_STATUS[PLATFORM_SIZE - 1] ? "FILLED" : "EMPTY") << std::endl;
        std::cout << "  Exit Gate: " << (BUS_PLATFORM_STATUS[PLATFORM_SIZE - 2] ? "FILLED" : "EMPTY") << std::endl;
        
        // 픽셀 비율 정보 출력
        std::cout << "--- Pixel Ratios ---" << std::endl;
        for (size_t i = 0; i < ratio_info.ratios.size(); ++i) {
            std::string area_name;
            if (i < stop_platform_count) {
                area_name = "Platform " + std::to_string(i);
            } else if (i == PLATFORM_SIZE - 2) {
                area_name = "Exit Gate";
            } else if (i == PLATFORM_SIZE - 1) {
                area_name = "Entry Gate";
            } else {
                area_name = "Unknown " + std::to_string(i);
            }
            
            double ratio_percent = ratio_info.ratios[i] * 100.0;
            double threshold_percent = ratio_info.thresholds[i] * 100.0;
            std::string result = ratio_info.results[i] ? "DETECTED" : "EMPTY";
            
            std::cout << "  " << area_name << ": " 
                      << std::fixed << std::setprecision(2) << ratio_percent << "%"
                      << " (threshold: " << threshold_percent << "%) -> " << result << std::endl;
        }
        last_masked_frames = *masked;
      }
    }

    if (!last_masked_frames.empty()) {
      int base_x = 520;
      int current_y = 280;
      int padding = 40;

      for (size_t i = 0; i < last_masked_frames.size(); ++i) {
        if (!last_masked_frames[i].empty()) {
          std::string win_name = "Platform Mask " + std::to_string(i);
          cv::imshow(win_name, last_masked_frames[i]);
          cv::moveWindow(win_name, base_x, current_y);
          current_y += last_masked_frames[i].rows + padding;
        }
      }
    }

    int key = cv::waitKey(1);
    if (key == 'q') {
      running = false;
    } else if (key == 'r' || key == 'R') {
      // R키를 누르면 버스 수 보정
      if (is_config_ready.load() && PLATFORM_SIZE >= 2) {
        reset_bus_count_to_platform_status(PLATFORM_SIZE, BUS_PLATFORM_STATUS);
      } else {
        std::cout << "⚠️  Cannot reset bus count: ROI not configured yet" << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "Shutting down all threads..." << std::endl;
  if (reader_thread.joinable()) reader_thread.join();
  if (warp_thread_.joinable()) warp_thread_.join();
  if (mask_thread_.joinable()) mask_thread_.join();

  std::cout << "Shutting down config thread..." << std::endl;
  if (roi_config_thread.joinable()) roi_config_thread.join();

  cleanup_status_shm();
  cv::destroyAllWindows();

  std::cout << "Program done. Total frames: " << total_frame
            << ", Wasted: " << frame_wasted << "/" << warped_wasted << "/"
            << masked_wasted << std::endl;
  return 0;
}

void update_bus_count_by_entry_exit(unsigned int platform_count, const bool* raw_status) {
  if (platform_count < 2) return;

  unsigned int entry_roi_idx = platform_count;     // N번 ROI
  unsigned int exit_roi_idx = platform_count + 1;  // N+1번 ROI

  bool current_entry_filled = raw_status[entry_roi_idx];
  bool current_exit_filled = raw_status[exit_roi_idx];

  auto now = std::chrono::steady_clock::now();

  if (prev_entry_roi_filled && !current_entry_filled) {
    last_entry_detected = now;
  }

  if (!prev_exit_roi_filled && current_exit_filled) {
    last_exit_detected = now;
  }

  if ((now - last_entry_detected) > std::chrono::duration<double>(ENTRY_DELAY_SEC)) {
    estimated_bus_in_station++;
    last_entry_detected = std::chrono::steady_clock::time_point();
  }

  if ((now - last_exit_detected) > std::chrono::duration<double>(EXIT_DELAY_SEC)) {
    if (estimated_bus_in_station > 0) estimated_bus_in_station--;
    last_exit_detected = std::chrono::steady_clock::time_point();
  }

  prev_entry_roi_filled = current_entry_filled;
  prev_exit_roi_filled = current_exit_filled;
}