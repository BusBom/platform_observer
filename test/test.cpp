#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <thread>
#include <atomic>
#include <csignal>

// 시스템 헤더
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// 공유 메모리 구조체 정의
struct StopStatus {
  int platform_status[20];  // 0: empty, 1: stopped
  char station_id[20];      // 정류장 ID
  time_t updated_at;        // 마지막 업데이트 시간
  int current_bus_count;    // 현재 정류장 내 버스 수
  int entered_bus_count;    // 누적 진입 버스 수
  int exited_bus_count;     // 누적 진출 버스 수
};



// 전역 변수
std::atomic<bool> running(true);
int status_shm_fd = -1;
StopStatus* status_shm_ptr = nullptr;

void signal_handler(int signum) {
  std::cout << "\n종료 신호를 받았습니다. 프로그램을 종료합니다..." << std::endl;
  running.store(false);
}

bool setup_status_shm() {
  status_shm_fd = shm_open("/busbom_status", O_RDONLY, 0666);
  if (status_shm_fd == -1) {
    std::cerr << "Cannot connect to shared memory /busbom_status." << std::endl;
    std::cerr << "Please make sure main.cpp is running." << std::endl;
    return false;
  }

  status_shm_ptr = (StopStatus*)mmap(0, sizeof(StopStatus), PROT_READ,
                                     MAP_SHARED, status_shm_fd, 0);
  if (status_shm_ptr == MAP_FAILED) {
    std::cerr << "Failed to map shared memory." << std::endl;
    close(status_shm_fd);
    return false;
  }

  std::cout << "✅ Connected to shared memory /busbom_status successfully." << std::endl;
  return true;
}

void cleanup_status_shm() {
  if (status_shm_ptr != nullptr && status_shm_ptr != MAP_FAILED) {
    munmap(status_shm_ptr, sizeof(StopStatus));
  }
  if (status_shm_fd != -1) {
    close(status_shm_fd);
  }
  std::cout << "Shared memory connection cleaned up." << std::endl;
}

void print_status() {
  if (status_shm_ptr == nullptr) return;

  // 현재 시간
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto tm_now = *std::localtime(&time_t_now);

  // 화면 클리어 (ANSI 이스케이프 시퀀스)
  std::cout << "\033[2J\033[H";  // 화면 클리어 및 커서를 맨 위로

  std::cout << "================================================================" << std::endl;
  std::cout << "                    Bus Station Status Monitor                   " << std::endl;
  std::cout << "================================================================" << std::endl;
  
  std::cout << "Current Time: " << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S") << std::endl;
  std::cout << "Station ID: " << status_shm_ptr->station_id << std::endl;
  
  // 마지막 업데이트 시간
  auto tm_updated = *std::localtime(&status_shm_ptr->updated_at);
  std::cout << "Last Updated: " << std::put_time(&tm_updated, "%H:%M:%S") << std::endl;
  
  std::cout << "\nBus Information:" << std::endl;
  std::cout << "   Current buses in station: " << status_shm_ptr->current_bus_count << std::endl;
  
  // 누적 진입/진출 카운트 표시
  std::cout << "   Total entered buses: " << status_shm_ptr->entered_bus_count << " buses" << std::endl;
  std::cout << "   Total exited buses: " << status_shm_ptr->exited_bus_count << " buses" << std::endl;
  
  // 순이동 계산
  int net_movement = status_shm_ptr->entered_bus_count - status_shm_ptr->exited_bus_count;
  std::cout << "   Net movement: " << net_movement << " buses" << std::endl;
  
  std::cout << "\nPlatform Status:" << std::endl;
  int platform_count = 0;
  for (int i = 0; i < 20; ++i) {
    if (status_shm_ptr->platform_status[i] == -1) {
      // 게이트 플랫폼
      if (i == 18) {  // Exit Gate
        std::cout << "   Exit Gate: ";
      } else if (i == 19) {  // Entry Gate
        std::cout << "   Entry Gate: ";
      } else {
        std::cout << "   Gate " << i << ": ";
      }
      std::cout << "GATE" << std::endl;
    } else if (status_shm_ptr->platform_status[i] >= 0) {
      // 일반 플랫폼
      std::string status = (status_shm_ptr->platform_status[i] == 1) ? "BUS STOPPED" : "EMPTY";
      std::cout << "   Platform " << i << ": " << status << std::endl;
      platform_count++;
    }
  }
  
  std::cout << "\nSummary:" << std::endl;
  std::cout << "   Total Platforms: " << platform_count << std::endl;
  std::cout << "   Currently Stopped: " << status_shm_ptr->current_bus_count << " buses" << std::endl;
  
  // 추가 통계 정보
  std::cout << "\nStatistics:" << std::endl;
  std::cout << "   Total Bus Movements: " << (status_shm_ptr->entered_bus_count + status_shm_ptr->exited_bus_count) << " buses" << std::endl;
  if (status_shm_ptr->entered_bus_count > 0 || status_shm_ptr->exited_bus_count > 0) {
    double entry_ratio = (double)status_shm_ptr->entered_bus_count / (status_shm_ptr->entered_bus_count + status_shm_ptr->exited_bus_count) * 100.0;
    double exit_ratio = (double)status_shm_ptr->exited_bus_count / (status_shm_ptr->entered_bus_count + status_shm_ptr->exited_bus_count) * 100.0;
    std::cout << "   Entry Ratio: " << std::fixed << std::setprecision(1) << entry_ratio << "%" << std::endl;
    std::cout << "   Exit Ratio: " << std::fixed << std::setprecision(1) << exit_ratio << "%" << std::endl;
  }
  
  std::cout << "\n💡 Controls: Ctrl+C to exit" << std::endl;
  std::cout << "================================================================" << std::endl;
}

int main() {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  std::cout << "🚌 Bus station status monitor starting..." << std::endl;
  std::cout << "Connecting to shared memory /busbom_status..." << std::endl;

  if (!setup_status_shm()) {
    std::cerr << "ERROR: Failed to connect to shared memory." << std::endl;
    std::cerr << "Please make sure main.cpp is running." << std::endl;
    return 1;
  }

  std::cout << "✅ Monitoring started. Press Ctrl+C to exit." << std::endl;

  // 메인 루프
  while (running.load()) {
    print_status();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5초마다 업데이트
  }

  cleanup_status_shm();
  std::cout << "Program terminated." << std::endl;
  return 0;
}
