#include "json.hpp"     // nlohmann/json

#include <fcgi_stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

// ----- System Programming Header -----
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>  // for ntohl
#include <cstring>      // for strncpy, memset

using json = nlohmann::json;

// ----- Socket Setting -----
const char* SOCKET_PATH = "/tmp/camera_socket";

/**
 * @brief 서버에 요청을 보내고 '변경 전' 이미지 데이터를 받아오는 함수
 * @param payload 서버로 보낼 JSON 문자열
 * @return 서버로부터 받은 JPEG 데이터 벡터. 실패 시 비어있는 벡터 반환.
 */
std::vector<char> request_to_server(const std::string& payload) {
    int sock_fd = 0;
    struct sockaddr_un serv_addr;
    
    // 1. 소켓 생성
    if ((sock_fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        fprintf(stderr, "[CGI ERROR] socket() failed: %s\n", strerror(errno));
        return {}; // 실패 시 빈 벡터 반환
    }

    std::memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sun_family = AF_UNIX;
    strncpy(serv_addr.sun_path, SOCKET_PATH, sizeof(serv_addr.sun_path) - 1);

    // 2. 서버에 연결
    if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        fprintf(stderr, "[CGI ERROR] connect() to %s failed: %s\n", SOCKET_PATH, strerror(errno));
        close(sock_fd);
        return {};
    }

    // 3. JSON 데이터 전송
    ssize_t bytes_sent = send(sock_fd, payload.c_str(), payload.length(), 0);
    if (bytes_sent != (ssize_t)payload.length()) {
        fprintf(stderr, "[CGI ERROR] send() failed: %s\n", strerror(errno));
        close(sock_fd);
        return {};
    }

    // 4. 서버로부터 응답 수신
    // 헤더: 이미지 타입(1바이트) + 이미지 크기(4바이트)
    char header[5] = {0};
    ssize_t bytes_read = read(sock_fd, header, 5);
    if (bytes_read != 5) {
        fprintf(stderr, "[CGI ERROR] read() header failed. Expected 5, got %zd\n", bytes_read);
        close(sock_fd);
        return {};
    }

    // uint8_t img_type = header[0]; // 타입은 1(JPEG)로 고정
    uint32_t img_size;
    memcpy(&img_size, &header[1], 4);
    img_size = ntohl(img_size); // 네트워크 바이트 순서 -> 호스트 바이트 순서

    if (img_size == 0 || img_size > 10 * 1024 * 1024) { // 10MB 이상은 비정상으로 간주
        fprintf(stderr, "[CGI ERROR] Invalid image size in header: %u\n", img_size);
        close(sock_fd);
        return {};
    }

    // 5. 실제 이미지 데이터 수신
    std::vector<char> img_buffer(img_size);
    ssize_t total_received = 0;
    while (total_received < img_size) {
        ssize_t result = read(sock_fd, img_buffer.data() + total_received, img_size - total_received);
        if (result <= 0) {
            fprintf(stderr, "[CGI ERROR] read() body failed: %s\n", strerror(errno));
            close(sock_fd);
            return {}; // 읽기 실패 또는 연결 종료
        }
        total_received += result;
    }
    
    close(sock_fd);
    return img_buffer;
}

int main() {
    while (FCGI_Accept() >= 0) {
        // POST 데이터 길이 확인
        char* content_length_str = getenv("CONTENT_LENGTH");
        int content_length = content_length_str ? atoi(content_length_str) : 0;
        
        std::vector<char> image_jpeg;

        if (content_length > 0) {
            // POST 데이터 읽기
            std::string post_data(content_length, '\0');
            FCGI_fread(&post_data[0], 1, content_length, FCGI_stdin);

            try {
                // JSON 유효성 검사. j는 컴파일 오류 회피용으로 사용 안 함.
                json j = json::parse(post_data);

                // 서버로부터 before img 받기
                image_jpeg = request_to_server(post_data);
                
            } catch (std::exception& e) {
                fprintf(stderr, "[CGI ERROR] JSON parse failed: %s\n", e.what());
            }
        }
        if (!image_jpeg.empty()) {
            // 성공 시: JPEG 이미지 출력
            printf("Content-Type: image/jpeg\r\n");
            printf("Content-Length: %zu\r\n\r\n", image_jpeg.size());
            fflush(stdout);                                                     // 헤더 flush
            FCGI_fwrite(image_jpeg.data(), 1, image_jpeg.size(), FCGI_stdout);  //이미지 데이터 전송
            fflush(FCGI_stdout);                                                // 이미지 flush
        } else {
            // 실패 시: 오류 메시지 JSON 출력
            printf("Content-Type: application/json\r\n\r\n");
            json err = {
                {"result", "error"},
                {"msg", "Failed to get image from the camera server."}
            };
            printf("%s\n", err.dump().c_str());
        }
    }
    return 0;
}
