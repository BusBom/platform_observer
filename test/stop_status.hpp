// stop_status.hpp
#pragma once
#include <time.h>

#define SHM_NAME_STATUS "/busbom_status"
#define MAX_PLATFORM_COUNT 20

struct StopStatus {
  int platform_status[MAX_PLATFORM_COUNT];  // 0: empty, 1: stopped
  char station_id[20];
  time_t updated_at;

  // --- 추가 필드 ---
  int current_bus_count;  // 현재 정류장 내 버스 수
  int entered_bus_count;  // 누적 진입 버스 수
  int exited_bus_count;   // 누적 진출 버스 수
};