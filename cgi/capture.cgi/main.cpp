// OpenCV와 FCGI 충돌 방지
#include <opencv2/opencv.hpp>


// ----- FCGI Header -----
#include <fcgi_stdio.h>
#include <iostream>
#include <string>
#include <vector>

#define VIDEO_SRC "chunlee.mov" // video source file

int main() {
    while(FCGI_Accept() >= 0) {

        cv::VideoCapture cap(VIDEO_SRC);
        if (!cap.isOpened()) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] Failed to open video source: %s\n", VIDEO_SRC);
            continue;   
        }
        cv::Mat frame;
        if (!cap.read(frame)) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] Failed to read frame from video source: %s\n", VIDEO_SRC);
            continue;   
        }

        std::vector<char> jpeg_data;
        std::vector<uchar> jpeg_buffer;
        std::vector<int> encode_params = { cv::IMWRITE_JPEG_QUALITY, 85 };
        
        if (!cv::imencode(".jpg", frame, jpeg_buffer, encode_params)) {
            FCGI_fprintf(FCGI_stderr, "[CGI ERROR] Failed to encode image to JPEG\n");
            continue;
        }

        jpeg_data.resize(jpeg_buffer.size());
        memcpy(jpeg_data.data(), jpeg_buffer.data(), jpeg_buffer.size());

        if (!jpeg_data.empty()) {
            // success: JPEG image output
            printf("Content-Type: image/jpeg\r\n");
            printf("Content-Length: %zu\r\n", jpeg_data.size());
            printf("Cache-Control: no-cache, no-store, must-revalidate\r\n");
            printf("Pragma: no-cache\r\n");
            printf("Expires: 0\r\n\r\n");
            FCGI_fflush(FCGI_stdout);
            FCGI_fwrite(jpeg_data.data(), 1, jpeg_data.size(), FCGI_stdout);
            FCGI_fflush(FCGI_stdout);
        } else {
            // failure: when JPEG encoding fails
            printf("Content-Type: application/json\r\n\r\n");
            printf("{\"result\":\"error\",\"msg\":\"Failed to capture image video.\"}\n");
        }
    }

    
    return 0;
}