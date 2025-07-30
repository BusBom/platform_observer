#pragma once
#include <ctime>

#define SHM_NAME_STATUS "/busbom_status"
#define MAX_PLATFORM_COUNT 20  // 최대 플랫폼 수

/**
 * StopStatus 구조체는 정류장 서버가 Main 서버에 전달하는
 * 플랫폼별 정차 상태를 공유 메모리로 전송하기 위한 데이터 구조입니다.
 *
 * - platform_status[i]의 값 의미:
 *   -1: 해당 인덱스는 현재 플랫폼으로 사용되지 않음 (Unused)
 *    0: 플랫폼에 버스 없음 (empty)
 *    1: 플랫폼에 버스 정차 중 (BUS DETECTED)
 */
struct StopStatus {
    int platform_status[MAX_PLATFORM_COUNT];  // 플랫폼 상태: -1(미사용), 0(비어있음), 1(정차)
    char station_id[16];                      // 정류장 고유 ID
    time_t updated_at;                        // 마지막 상태 갱신 시각 (time(nullptr))
};