/**
 * @file stop-status.cgi/main.cpp
 * @brief 공유 메모리(/busbom_status)의 데이터를 읽어 JSON 형식으로 출력하는 CGI 스크립트.
 * @details 이 스크립트는 Nginx에 의해 실행되며, 메인 서버의 GET 요청에 응답합니다.
 */
#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <cstring>

// 시스템 헤더
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "stop_status.hpp"

#define MAX_PLATFORM_COUNT 20
#define SHM_NAME_STATUS "/busbom_status"

/**
 * @brief time_t 형식을 "YYYY-MM-DD HH:MM:SS" 형식의 문자열로 변환합니다.
 * @param time 변환할 time_t 값.
 * @return 변환된 문자열.
 */
 std::string time_t_to_string(time_t time_val) {
    char buffer[80];
    struct tm *timeinfo;
    timeinfo = localtime(&time_val);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    return std::string(buffer);
}

int main() {
    std::cout << "Content-Type: application/json\r\n\r\n";

    int shm_fd = shm_open(SHM_NAME_STATUS, O_RDONLY, 0666);
    if (shm_fd == -1) {
        std::cout << "{\"error\": \"Shared memory not available. Is the main program running?\"}" << std::endl;
        return 1;
    }

    void* ptr = mmap(0, sizeof(StopStatus), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        std::cout << "{\"error\": \"Failed to map shared memory\"}" << std::endl;
        close(shm_fd);
        return 1;
    }

    StopStatus* status_ptr = static_cast<StopStatus*>(ptr);

    // JSON 응답 생성
    std::cout << "{" << std::endl;
    std::cout << "  \"station_id\": \"" << status_ptr->station_id << "\"," << std::endl;
    
    std::cout << "  \"platform_status\": [";
    bool first = true;
    for (int i = 0; i < MAX_PLATFORM_COUNT; ++i) {
        // platform_status가 -1이면 미사용 플랫폼이므로, 그 이전까지만 출력
        if (status_ptr->platform_status[i] == -1) {
            break; 
        }
        if (!first) {
            std::cout << ", ";
        }
        std::cout << status_ptr->platform_status[i];
        first = false;
    }
    std::cout << "]," << std::endl;

    std::cout << "  \"updated_at\": \"" << time_t_to_string(status_ptr->updated_at) << "\"" << std::endl;
    std::cout << "}" << std::endl;

    munmap(ptr, sizeof(StopStatus));
    close(shm_fd);

    return 0;
}