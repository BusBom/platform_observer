#include "json.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <fcgi_stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>  // for ntohl
#include <cstring>      // for strncpy, memset

using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

struct ROIData {
    std::vector<std::vector<cv::Point>> stop_rois;
};

// Convert JSON array of [[x, y], [x, y], ...] to cv::Point vector
std::vector<cv::Point> convertPointsFromJson(const json& points_json) {
    if (!points_json.is_array()) {
        throw std::runtime_error("Points must be an array");
    }
    
    std::vector<cv::Point> points;
    
    for (const auto& point : points_json) {
        if (!point.is_array() || point.size() != 2) {
            throw std::runtime_error("Each point must be an array with 2 values [x, y]");
        }
        
        // 더 안전한 타입 체크와 변환
        if (!point[0].is_number() || !point[1].is_number()) {
            throw std::runtime_error("Point coordinates must be numbers");
        }
        
        int x = point[0].get<int>();
        int y = point[1].get<int>();
        points.push_back(cv::Point(x, y));
    }
    
    return points;
}

// Serialize ROIData to binary format for socket transmission
std::string serializeROIData(const ROIData& roi_data) {
    std::string serialized;
    
    // Serialize stop_rois count
    uint32_t stop_count = roi_data.stop_rois.size();
    serialized.append(reinterpret_cast<const char*>(&stop_count), sizeof(stop_count));
    
    // Serialize each stop_roi
    for (const auto& roi : roi_data.stop_rois) {
        uint32_t point_count = roi.size();
        serialized.append(reinterpret_cast<const char*>(&point_count), sizeof(point_count));
        
        for (const auto& point : roi) {
            int32_t x = point.x;
            int32_t y = point.y;
            serialized.append(reinterpret_cast<const char*>(&x), sizeof(x));
            serialized.append(reinterpret_cast<const char*>(&y), sizeof(y));
        }
    }
    // wait_rois 관련 코드 삭제
    return serialized;
}

const char* SOCKET_PATH = "/tmp/roi_socket";

bool request_to_setup_roi(const std::string& payload) {
    int sock_fd = 0;
    struct sockaddr_un serv_addr;
    
    fprintf(stderr, "[CGI DEBUG] Attempting to connect to socket: %s\n", SOCKET_PATH);
    fprintf(stderr, "[CGI DEBUG] Payload size: %zu bytes\n", payload.size());
    
    // 1. 소켓 생성
    if ((sock_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        fprintf(stderr, "[CGI ERROR] socket() failed: %s\n", strerror(errno));
        return false; // 실패 시 false 반환
    }
    fprintf(stderr, "[CGI DEBUG] Socket created successfully\n");

    std::memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sun_family = AF_UNIX;
    strncpy(serv_addr.sun_path, SOCKET_PATH, sizeof(serv_addr.sun_path) - 1);

    // 2. 서버에 연결
    if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        fprintf(stderr, "[CGI ERROR] connect() to %s failed: %s\n", SOCKET_PATH, strerror(errno));
        close(sock_fd);
        return false;
    }
    fprintf(stderr, "[CGI DEBUG] Connected to socket successfully\n");

    // 3. 데이터 전송
    ssize_t bytes_sent = send(sock_fd, payload.c_str(), payload.length(), 0);
    if (bytes_sent != (ssize_t)payload.length()) {
        fprintf(stderr, "[CGI ERROR] send() failed: %s\n", strerror(errno));
        close(sock_fd);
        return false;
    }
    fprintf(stderr, "[CGI DEBUG] Sent %zd bytes successfully\n", bytes_sent);
    
    close(sock_fd);
    fprintf(stderr, "[CGI DEBUG] Socket closed, operation completed successfully\n");
    return true;
}

int main() {
    while (FCGI_Accept() >= 0) {
        char* content_length_str = getenv("CONTENT_LENGTH");
        int content_length = content_length_str ? atoi(content_length_str) : 0;

        fprintf(stderr, "[CGI DEBUG] New request received, content length: %d\n", content_length);
        printf("Content-Type: application/json\r\n\r\n");

        if (content_length > 0) {
            std::string post_data(content_length, '\0');
            FCGI_fread(&post_data[0], 1, content_length, FCGI_stdin);
            
            fprintf(stderr, "[CGI DEBUG] POST data received: %s\n", post_data.c_str());

            try {
                json j = json::parse(post_data);
                fprintf(stderr, "[CGI DEBUG] JSON parsed successfully\n");
                
                // 필수 필드 확인
                if (!j.contains("stop_rois")) {
                    fprintf(stderr, "[CGI ERROR] Missing required fields\n");
                    json err = {
                        {"error", "Missing required fields: stop_rois"},
                        {"status", "400 Bad Request"}
                    };
                    printf("%s\n", err.dump().c_str());
                    continue;
                }
                
                ROIData roi_data;
                
                // stop_rois 파싱
                if (j["stop_rois"].is_array()) {
                    fprintf(stderr, "[CGI DEBUG] stop_rois array size: %zu\n", j["stop_rois"].size());
                    for (size_t i = 0; i < j["stop_rois"].size(); ++i) {
                        const auto& roi = j["stop_rois"][i];
                        fprintf(stderr, "[CGI DEBUG] Processing stop_roi %zu\n", i);
                        if (roi.is_array()) {
                            fprintf(stderr, "[CGI DEBUG] stop_roi %zu has %zu points\n", i, roi.size());
                            try {
                                roi_data.stop_rois.push_back(convertPointsFromJson(roi));
                                fprintf(stderr, "[CGI DEBUG] stop_roi %zu converted successfully\n", i);
                            } catch (const std::exception& e) {
                                fprintf(stderr, "[CGI ERROR] Error converting stop_roi %zu: %s\n", i, e.what());
                                fprintf(stderr, "[CGI DEBUG] stop_roi %zu content: %s\n", i, roi.dump().c_str());
                                throw;
                            }
                        } else {
                            fprintf(stderr, "[CGI ERROR] stop_roi %zu is not an array\n", i);
                        }
                    }
                }
                
                fprintf(stderr, "[CGI DEBUG] ROI data parsed: %zu stop_rois\n", roi_data.stop_rois.size());
                
                // ROI 데이터를 바이너리로 직렬화
                std::string serialized_data = serializeROIData(roi_data);
                fprintf(stderr, "[CGI DEBUG] Serialized data size: %zu bytes\n", serialized_data.size());
                
                // 소켓을 통해 전송
                bool success = request_to_setup_roi(serialized_data);
                
                if (success) {
                    fprintf(stderr, "[CGI DEBUG] ROI data sent successfully\n");
                    json response = {
                        {"status", "success"},
                        {"message", "ROI data sent successfully"},
                        {"stop_rois_count", roi_data.stop_rois.size()}
                    };
                    printf("%s\n", response.dump().c_str());
                } else {
                    fprintf(stderr, "[CGI ERROR] Failed to send ROI data\n");
                    json err = {
                        {"error", "Failed to send ROI data to socket"},
                        {"status", "500 Internal Server Error"}
                    };
                    printf("%s\n", err.dump().c_str());
                }
                
            } catch (std::exception& e) {
                fprintf(stderr, "[CGI ERROR] Exception: %s\n", e.what());
                json err = {
                    {"error", "Invalid JSON data or processing error"},
                    {"details", e.what()},
                    {"status", "400 Bad Request"}
                };
                printf("%s\n", err.dump().c_str());
            }
        } else {
            fprintf(stderr, "[CGI ERROR] No POST data\n");
            json err = {
                {"error", "No POST data received"},
                {"status", "400 Bad Request"}
            };
            printf("%s\n", err.dump().c_str());
        }
    }
    
    return 0;
}