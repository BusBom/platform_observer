/**
 * @file main.cpp
 * @brief 유닉스 도메인 소켓으로 ROI 설정과 영상 스트림을 받아 버스를 감지하는 메인 프로그램.
 */
#include <chrono> //warp 되는 것까지 확인함, 화면 디버깅 지우고, 다음 단계로 넘어갈 것것
#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>

// 시스템 헤더
#include <unistd.h>
#include <sys/socket.h>     //소켓
#include <sys/un.h>         // 유닉스 도메인 소켓
#include <sys/mman.h>       // 공유 메모리
#include <fcntl.h>          // 공유 메모리
#include <sys/stat.h>       // chmod
#include <sys/select.h>     // select

#include "checker.hpp"
#include "filters.hpp"
#include "safeQueue.hpp"
#include <X11/Xlib.h>       // XInitThreads
// 상수 정의
#define MAX_QUEUE_SIZE 2
#define MAX_PLATFORM_COUNT 20       //플랫폼 개수 설정

// 통신 경로
#define SHM_NAME "/busbom_frame"
#define SOCKET_PATH "/tmp/roi_socket"

// 프레임 설정
#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720
#define FRAME_CHANNELS 3

// --- 전역 설정 ---
const int TARGET_FPS = 15;
const std::chrono::milliseconds FRAME_DURATION(1000 / TARGET_FPS);

// --- 전역 변수 ---
unsigned int PLATFORM_SIZE = 0;
std::vector<std::vector<cv::Point>> platform_rois;
bool BUS_PLATFORM_STATUS[MAX_PLATFORM_COUNT] = {false};
std::mutex rois_mutex;

std::atomic<bool> running(true);
std::atomic<bool> is_config_ready(false);           // ROI 설정 완료 여부 플래그

// --- 데이터 큐 ---
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> frame_queue;
static ThreadSafeQueue<std::shared_ptr<std::vector<cv::Mat>>> warped_queue;
static ThreadSafeQueue<std::shared_ptr<std::vector<cv::Mat>>> masked_queue;
static ThreadSafeQueue<std::shared_ptr<cv::Mat>> debug_frame_queue; // 디버깅용

// --- 디버깅용 카운터 ---
unsigned int total_frame = 0;
unsigned int frame_wasted = 0;
unsigned int warped_wasted = 0;
unsigned int masked_wasted = 0;


/** 현재 조정 상황
유채색 vs 무채색 : 0.15
영역 내 흰색 판단 : 70 (상위 30%)
버스 또는 물체 유무 판단 : 0.4
*/

void signal_handler(int signum) {
    std::cout << "\nTermination signal received. Shutting down..." << std::endl;
    running.store(false);
}


/**
 * @brief CGI 스크립트로부터 유닉스 도메인 소켓을 통해 바이너리 ROI 설정을 수신하는 스레드.
 */
void receive_roi_config_thread() {
    int server_fd;
    struct sockaddr_un address;
    
    if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == 0) {
        perror("socket failed"); return;
    }

    memset(&address, 0, sizeof(struct sockaddr_un));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, SOCKET_PATH, sizeof(address.sun_path) - 1);
    unlink(SOCKET_PATH);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed"); close(server_fd); return;
    }
    // 소켓 파일의 권한을 777로 변경하여 모든 사용자가 접근 가능하도록 함
    if (chmod(SOCKET_PATH, 0777) < 0) {
        perror("chmod failed");
        close(server_fd);
        unlink(SOCKET_PATH);
        return;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen failed"); close(server_fd); unlink(SOCKET_PATH); return;
    }

    while (running.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(server_fd, &read_fds);

        struct timeval timeout;
        timeout.tv_sec = 1; // 1초 타임아웃
        timeout.tv_usec = 0;

        int activity = select(server_fd + 1, &read_fds, NULL, NULL, &timeout);

        if ((activity < 0) && (errno != EINTR)) {
            perror("select error"); break;
        }

        if (FD_ISSET(server_fd, &read_fds)) {
            int client_fd = accept(server_fd, NULL, NULL);
            if (client_fd < 0) {
                if (running.load()) perror("accept failed");
                break;
            }
            
            std::cout << "🤝 CGI client connected. Receiving ROI data..." << std::endl;

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
                    if (offset + len > buffer.size()) throw std::runtime_error("Buffer underflow");
                    memcpy(dest, buffer.data() + offset, len);
                    offset += len;
                };

                uint32_t stop_count;
                read_from_buffer(&stop_count, sizeof(stop_count));
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
                std::fill_n(BUS_PLATFORM_STATUS, MAX_PLATFORM_COUNT, false);

                std::cout << "✅ ROI configuration updated. Total platforms: " << PLATFORM_SIZE << std::endl;
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
    
    std::cout << "Trying to connect to shared memory " << SHM_NAME << "..." << std::endl;
    while(shm_fd == -1 && running.load()) {
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
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

        cv::Mat debug_frame = local_frame.clone();
        {
            std::lock_guard<std::mutex> lock(rois_mutex);
            if (!platform_rois.empty()) {
                cv::polylines(debug_frame, platform_rois, true, cv::Scalar(0, 255, 0), 2);
            }
        }
        if (debug_frame_queue.size() < 10) {
             debug_frame_queue.push(std::make_shared<cv::Mat>(std::move(debug_frame)));
        }

        if (frame_queue.size() > MAX_QUEUE_SIZE) {
            frame_queue.try_pop(garbage);
            frame_wasted++;
        }

        auto_brightness_balance(local_frame, balanced);
        total_frame++;
        frame_queue.push(std::make_shared<cv::Mat>(std::move(balanced)));

        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        auto sleep_time = FRAME_DURATION - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        if (sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    munmap(ptr, frame_size);
    close(shm_fd);
}

/**
 * @brief 프레임을 큐에서 가져와 투시 변환(warp)을 수행하는 스레드 함수.
 */
void warp_thread() {
std::shared_ptr<cv::Mat> frame;
    std::shared_ptr<std::vector<cv::Mat>> garbage;

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
            frame_queue.try_pop(frame);
            frame_wasted++;
        }

        if (frame_queue.try_pop(frame)) {
            std::vector<cv::Mat> warped;
            warp_rectified_areas((*frame), warped, local_rois);
            warped_queue.push(std::make_shared<std::vector<cv::Mat>>(std::move(warped)));
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

        // 큐에 프레임이 너무 많이 쌓이면 오래된 프레임을 버려서 지연시간을 줄임
        while (warped_queue.size() > MAX_QUEUE_SIZE && running.load()) {
            warped_queue.try_pop(garbage);
            warped_wasted++;
        }

        if (warped_queue.try_pop(frame)) {
            std::vector<cv::Mat> masked_1(local_platform_size), masked_2(local_platform_size);
            remove_achromatic_areas((*frame), masked_1);
            revive_white_areas(masked_1, masked_2);
            masked_queue.push(std::make_shared<std::vector<cv::Mat>>(std::move(masked_2)));
        } 


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * @brief 로컬 영상 파일에서 프레임을 읽어와 큐에 넣는 스레드 함수
 */
void video_read_thread(const std::string& video_path) {
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "failed to open viedo: " << video_path << std::endl;
        running.store(false);
        return;
    }

    std::cout << "success open the file: " << video_path << std::endl;

    // 🎯 영상 FPS에 맞춰 sleep 시간 계산
    double video_fps = cap.get(cv::CAP_PROP_FPS);
    int delay_ms = (video_fps > 1.0) ? static_cast<int>(1000.0 / video_fps) : 33;
    std::chrono::milliseconds frame_delay(delay_ms);

    cv::Mat frame, resized;
    std::shared_ptr<cv::Mat> garbage;

    while (running.load()) {
        auto start = std::chrono::high_resolution_clock::now();

        if (!cap.read(frame) || frame.empty()) break;

        // 👉 프레임 리사이즈
        cv::resize(frame, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

        // 👉 디버그용 ROI 오버레이
        {
            std::lock_guard<std::mutex> lock(rois_mutex);
            if (!platform_rois.empty()) {
                cv::polylines(resized, platform_rois, true, cv::Scalar(0, 255, 0), 2);
            }
        }

        if (debug_frame_queue.size() < 10) {
            debug_frame_queue.push(std::make_shared<cv::Mat>(resized.clone()));
        }

        if (frame_queue.size() > MAX_QUEUE_SIZE) {
            frame_queue.try_pop(garbage);
            frame_wasted++;
        }

        frame_queue.push(std::make_shared<cv::Mat>(std::move(resized)));
        total_frame++;

        // 💡 영상 fps 기준 sleep
        // auto elapsed = std::chrono::high_resolution_clock::now() - start;
        // auto sleep_time = frame_delay - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        // if (sleep_time > std::chrono::milliseconds(0)) {
        //     std::this_thread::sleep_for(sleep_time);
        // }
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        auto sleep_time = FRAME_DURATION - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        if (sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    cap.release();
    running.store(false);
}


int main() {
    XInitThreads(); // GUI 멀티스레딩 안정성 확보
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::thread roi_config_thread(receive_roi_config_thread);

    std::thread reader_thread(video_read_thread, "file://home/Qwd/platform_observer/video/IMG_4403.mp4");
    //std::thread reader_thread(shm_read_thread);

    // 처리 스레드들을 담을 변수
    std::thread warp_thread_;
    std::thread mask_thread_;
    bool processing_threads_started = false;

    cv::Mat last_raw_frame_with_rois;
    std::vector<cv::Mat> last_masked_frames;

    std::cout << "✅ Main process running. Waiting for initial CGI configuration via socket..." << std::endl;

    while (running.load()) {
        // ROI 설정이 완료되었고, 처리 스레드가 아직 시작되지 않았다면 시작
        if (is_config_ready.load() && !processing_threads_started) {
            std::cout << "✅ Initial configuration received. Starting processing threads..." << std::endl;
            warp_thread_ = std::thread(warp_thread);
            mask_thread_ = std::thread(mask_thread);
            processing_threads_started = true;
        }

        // 원본 프레임 가져와서 표시
        std::shared_ptr<cv::Mat> debug_frame;
        if (debug_frame_queue.try_pop(debug_frame)) {
            last_raw_frame_with_rois = *debug_frame;
        }
        if (!last_raw_frame_with_rois.empty()) {
            cv::imshow("Raw Frame with ROIs", last_raw_frame_with_rois);
        }

        // 처리된 마스크 프레임 가져와서 표시
        if (is_config_ready.load()) {
            std::shared_ptr<std::vector<cv::Mat>> masked;
            if (masked_queue.try_pop(masked)) {
                check_bus_platform(*masked, BUS_PLATFORM_STATUS);
                for (unsigned int i = 0; i < masked->size(); i++) {
                    std::cout << "Platform " << i << " status: " << (BUS_PLATFORM_STATUS[i] ? "BUS DETECTED" : "empty") << std::endl;
                }
                last_masked_frames = *masked;
            }
            for (size_t i = 0; i < last_masked_frames.size(); ++i) {
                if (!last_masked_frames[i].empty()) {
                    std::string win_name = "Debug View - Platform " + std::to_string(i);
                    cv::imshow(win_name, last_masked_frames[i]);
                }
            }
        }

        // ✔ GUI 창이 있을 경우 imshow 이후에도 최소한의 이벤트 처리를 위해 다음 줄 추가
        cv::waitKey(1);  // 블록되지 않음. GUI 창이 안 뜰 경우 이 줄도 삭제 가능

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // CPU 과점유 방지용

    }

    std::cout << "Shutting down all threads..." << std::endl;
    if(reader_thread.joinable()) reader_thread.join();
    if(warp_thread_.joinable()) warp_thread_.join();
    if(mask_thread_.joinable()) mask_thread_.join();

    std::cout << "Shutting down config thread..." << std::endl;
    if(roi_config_thread.joinable()) roi_config_thread.join();

    cv::destroyAllWindows();
    std::cout << "Program done. Total frames: " << total_frame << ", Wasted: " 
              << frame_wasted << "/" << warped_wasted << "/" << masked_wasted << std::endl;
    return 0;
}
