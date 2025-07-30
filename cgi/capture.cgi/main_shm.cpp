// THIS CODE IS FOR USING SHARED MEMORY(RPI_CAMERA) TO CAPTURE IMAGES
// FROM A VIDEO SOURCE AND SERVE THEM AS JPEG IMAGES OVER FCGI.

// avoid OpenCV and FCGI conflicts
#define _FILE_OFFSET_BITS 64
#include <opencv2/opencv.hpp>

// ----- System Programming Header -----
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>  // for ntohl
#include <cstring>      // for strncpy, memset

// ----- Shared Memory Header -----
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

// ----- FCGI Header -----
#include <fcgi_stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>

// ----- Shared Memory Setting -----
const char* SHM_NAME = "/busbom_frame";
#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720
#define FRAME_CHANNELS 3
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT * FRAME_CHANNELS)

/**
 * @brief 공유메모리에서 RAW 이미지를 읽어서 JPEG로 인코딩하는 함수
 * @return 인코딩된 JPEG 이미지 데이터. 실패 시 빈 벡터 반환.
 */
std::vector<char> capture_image_from_shm() {
    int shm_fd = -1;
    void* shm_ptr = nullptr;
    std::vector<char> jpeg_data;

    try {
        // open shared memory
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0);
        if (shm_fd == -1) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] shm_open() failed: %s\n", strerror(errno));
            return {};
        }

        // check shared memory size
        struct stat shm_stat;
        if (fstat(shm_fd, &shm_stat) == -1) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] fstat() failed: %s\n", strerror(errno));
            close(shm_fd);
            return {};
        }

        size_t shm_size = shm_stat.st_size;
        if (shm_size != FRAME_SIZE) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] Shared memory size mismatch. Expected: %d, Actual: %zu\n", FRAME_SIZE, shm_size);
            close(shm_fd);
            return {};
        }

        // mmap shared memory
        shm_ptr = mmap(nullptr, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
        if (shm_ptr == MAP_FAILED) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] mmap() failed: %s\n", strerror(errno));
            close(shm_fd);
            return {};
        }

        // raw bgr to OpenCV Mat
        unsigned char* raw_data = static_cast<unsigned char*>(shm_ptr);
        cv::Mat frame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, raw_data);
        
        // JPEG enconding
        std::vector<uchar> jpeg_buffer;
        std::vector<int> encode_params = { cv::IMWRITE_JPEG_QUALITY, 85 };
        
        if (!cv::imencode(".jpg", frame, jpeg_buffer, encode_params)) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] Failed to encode image to JPEG\n");
            munmap(shm_ptr, shm_size);
            close(shm_fd);
            return {};
        }
        
        // JPEG data to char vector
        jpeg_data.resize(jpeg_buffer.size());
        memcpy(jpeg_data.data(), jpeg_buffer.data(), jpeg_buffer.size());

        // cleanup
        munmap(shm_ptr, shm_size);
        close(shm_fd);

    } catch (std::exception& e) {
        FCGI_fprintf(FCGI_stderr, "[CGI ERROR] Exception in capture_image_from_shm: %s\n", e.what());
        if (shm_ptr && shm_ptr != MAP_FAILED) {
            munmap(shm_ptr, FRAME_SIZE);
        }
        if (shm_fd != -1) {
            close(shm_fd);
        }
        return {};
    }

    return jpeg_data;
}

int main() {
    while (FCGI_Accept() >= 0) {
        // GET
        char* request_method = getenv("REQUEST_METHOD");
        if (!request_method || strcmp(request_method, "GET") != 0) {
            printf("Content-Type: application/json\r\n\r\n");
            printf("{\"result\":\"error\",\"msg\":\"Only GET method is supported.\"}\n");
            continue;
        }

        // capture image from shared memory
        std::vector<char> image_jpeg = capture_image_from_shm();
        
        if (!image_jpeg.empty()) {
            // success: JPEG image output
            printf("Content-Type: image/jpeg\r\n");
            printf("Content-Length: %zu\r\n", image_jpeg.size());
            printf("Cache-Control: no-cache, no-store, must-revalidate\r\n");
            printf("Pragma: no-cache\r\n");
            printf("Expires: 0\r\n\r\n");
            FCGI_fflush(FCGI_stdout);
            FCGI_fwrite(image_jpeg.data(), 1, image_jpeg.size(), FCGI_stdout);
            FCGI_fflush(FCGI_stdout);
        } else {
            // failure: when JPEG encoding fails
            printf("Content-Type: application/json\r\n\r\n");
            printf("{\"result\":\"error\",\"msg\":\"Failed to capture image from shared memory.\"}\n");
        }
    }
    return 0;
}