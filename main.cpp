/**
 * @file main.cpp
 * @brief 유닉스 도메인 소켓으로 ROI 설정과 영상 스트림을 받아 버스를 감지하고,
 *        정제된 감지 결과를 공유 메모리에 기록하는 메인 프로그램
 */
#include <chrono>
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

#include "checker.hpp"      // check_bus_platform
#include "filters.hpp"      // 차량 마스킹
#include "safeQueue.hpp"
#include "stop_status.hpp"  // StopStatus 구조체
#include <X11/Xlib.h>       // XInitThreads

// 상수 정의
#define MAX_QUEUE_SIZE 2            // 큐사이즈가 클수록 딜레이 증가
#define MAX_PLATFORM_COUNT 20       // 최대 플랫폼 수

// 통신 경로
#define SHM_NAME_FRAME "/busbom_frame"      // live stream 
#define SHM_NAME_STATUS "/busbom_status"    // bus stop status
#define SOCKET_PATH "/tmp/roi_socket"       // roi info

// 프레임 설정
#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720
#define FRAME_CHANNELS 3

// --- 전역 설정 ---
const int TARGET_FPS = 15;
const std::chrono::milliseconds FRAME_DURATION(1000 / TARGET_FPS);
const char* STATION_ID = "101000004";    // 정류장 ID

// --- 상태 안정화(Debouncing)용 변수 ---
static int stable_status[MAX_PLATFORM_COUNT] = {0};
static int prev_stable_status[MAX_PLATFORM_COUNT] = {0};

// --- 시간 기반 정차 판단용 변수 ---
static std::chrono::steady_clock::time_point detection_start_time[MAX_PLATFORM_COUNT];
static int detection_loss_counter[MAX_PLATFORM_COUNT] = {0}; // 감지 손실 카운터
const double STABLE_TIME_THRESHOLD_S = 3.5;     // 정차로 판단하기 위한 시간 (초)
const int LOSS_TOLERANCE_CYCLES = 5;            // 감지 손실 허용 횟수 (사이클)

// --- 전역 변수 ---
unsigned int PLATFORM_SIZE = 0;
std::vector<std::vector<cv::Point>> platform_rois;
bool BUS_PLATFORM_STATUS[MAX_PLATFORM_COUNT] = {false};
std::mutex rois_mutex;

std::atomic<bool> running(true);
std::atomic<bool> is_config_ready(false);           // ROI 설정 완료 여부 플래그

// --- 공유 메모리 포인터 (상태 출력용) ---
int status_shm_fd = -1;
StopStatus* status_shm_ptr = nullptr;

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

void initialize_platform_status(unsigned int platform_count);
void process_bus_status(unsigned int platform_count, const bool* raw_status);
void update_shared_status(unsigned int platform_count);
void video_read_thread(const std::string& video_filename);

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
                
                // 플랫폼 상태 및 공유 메모리 초기화
                initialize_platform_status(PLATFORM_SIZE);

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
    
    std::cout << "Trying to connect to shared memory " << SHM_NAME_FRAME << "..." << std::endl;
    while(shm_fd == -1 && running.load()) {
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
                cv::polylines(debug_frame, platform_rois, true, cv::Scalar(0, 255, 0), 2);
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
        auto sleep_time = FRAME_DURATION - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
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

    status_shm_ptr = (StopStatus*)mmap(0, sizeof(StopStatus), PROT_WRITE, MAP_SHARED, status_shm_fd, 0);
    if (status_shm_ptr == MAP_FAILED) {
        perror("mmap for status failed");
        close(status_shm_fd);
        shm_unlink(SHM_NAME_STATUS);
        status_shm_ptr = nullptr;
        return false;
    }

    std::cout << "✅ Status shared memory '" << SHM_NAME_STATUS << "' is ready." << std::endl;
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
    if (status_shm_ptr == nullptr) return;

    for (unsigned int i = 0; i < MAX_PLATFORM_COUNT; ++i) {
        stable_status[i] = 0;
        prev_stable_status[i] = 0;
        detection_start_time[i] = std::chrono::steady_clock::time_point(); // 시간 기록 초기화
        detection_loss_counter[i] = 0;

        if (i < platform_count) {
            status_shm_ptr->platform_status[i] = 0; // 사용 플랫폼은 0(empty)으로 초기화
        } else {
            status_shm_ptr->platform_status[i] = -1; // 미사용 플랫폼은 -1로 초기화
        }
    }
    strncpy(status_shm_ptr->station_id, STATION_ID, sizeof(status_shm_ptr->station_id) - 1);
    status_shm_ptr->updated_at = time(nullptr);
}

/**
 * @brief 원본 감지 결과를 안정화된 상태로 변환합니다.
 */
void process_bus_status(unsigned int platform_count, const bool* raw_status) {
    for (unsigned int i = 0; i < platform_count; ++i) {
        if (raw_status[i]) { // 버스가 감지된 경우
            // 이전에 정차 상태가 아니었을 때 (새로 감지 시작)
            if (stable_status[i] == 0) {
                // 감지 시작 시간을 기록한 적이 없다면, 현재 시간을 기록
                if (detection_start_time[i].time_since_epoch().count() == 0) {
                    detection_start_time[i] = std::chrono::steady_clock::now();
                }

                // 경과 시간 계산
                auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                    std::chrono::steady_clock::now() - detection_start_time[i]
                );

                // 경과 시간이 임계값을 넘으면 정차로 확정
                if (elapsed_time.count() >= STABLE_TIME_THRESHOLD_S) {
                    stable_status[i] = 1;
                }
            }
        } else { // 버스가 감지되지 않은 경우
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
 * @brief 안정화된 상태를 공유 메모리에 업데이트합니다.
 */
void update_shared_status(unsigned int platform_count) {
    if (status_shm_ptr == nullptr) return;

    for (unsigned int i = 0; i < platform_count; ++i) {
        status_shm_ptr->platform_status[i] = stable_status[i];
    }
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

        // 오래된 프레임을 버려서 지연시간을 줄임
        while (frame_queue.size() > MAX_QUEUE_SIZE && running.load()) {
            frame_queue.try_pop(garbage);
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

        // 오래된 프레임을 버려서 지연시간을 줄임
        while (warped_queue.size() > MAX_QUEUE_SIZE && running.load()) {
            warped_queue.try_pop(garbage);
            warped_wasted++;
        }

        // 파란색 마스크 + 동적 흰색 마스크
        if (warped_queue.try_pop(frame)) {
            std::vector<cv::Mat> final_masks;
            generate_bus_mask(*frame, final_masks);
            masked_queue.push(std::make_shared<std::vector<cv::Mat>>(std::move(final_masks)));
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

    cv::Mat balanced;  // 밝기 조정 필요시 사용
    cv::Mat frame, resized;
    std::shared_ptr<cv::Mat> garbage;

    while (running.load()) {
        auto start = std::chrono::high_resolution_clock::now();

        if (!cap.read(frame) || frame.empty()) break;

        // 프레임 리사이즈
        cv::resize(frame, resized, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

        // ---- 디버그용 ROI 오버레이 ----
        {
            std::lock_guard<std::mutex> lock(rois_mutex);
            if (!platform_rois.empty()) {
                cv::polylines(resized, platform_rois, true, cv::Scalar(0, 255, 0), 2);
            }
        }

        if (debug_frame_queue.size() < 10) {
            debug_frame_queue.push(std::make_shared<cv::Mat>(resized.clone()));
        }
        // ----------------------------

        // 오래된 프레임을 버려서 지연시간을 줄임
        if (frame_queue.size() > MAX_QUEUE_SIZE) {
            frame_queue.try_pop(garbage);
            frame_wasted++;
        }

        auto_brightness_balance(resized, balanced);
        total_frame++;
        frame_queue.push(std::make_shared<cv::Mat>(std::move(balanced)));

        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        auto sleep_time = FRAME_DURATION - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        if (sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    cap.release();
    running.store(false);
}


int main(int argc, char *argv[]) {
    XInitThreads(); // GUI 멀티스레딩 안정성 확보
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (!setup_status_shm()) {
        return 1;
    }

    std::thread roi_config_thread(receive_roi_config_thread);
    std::thread reader_thread;

    // 프로그램 인자에 따라 영상 소스 결정
    if (argc > 1) {
        // 인자로 영상 파일 경로가 주어지면 비디오 파일 모드로 실행
        std::cout << "Starting in video file mode with: " << argv[1] << std::endl;
        reader_thread = std::thread(video_read_thread, "file://home/Qwd/platform_observer/video/" + std::string(argv[1]));
    } else {
        // 인자가 없으면 기본값인 공유 메모리 모드로 실행
        std::cout << "Starting in shared memory mode." << std::endl;
        reader_thread = std::thread(shm_read_thread);
    }

    // 처리 스레드들을 담을 변수
    std::thread warp_thread_;
    std::thread mask_thread_;
    bool processing_threads_started = false;

    std::cout << "✅ Main process running. Waiting for initial CGI configuration via socket..." << std::endl;

    // ---- debug: 변수 선언 ----
    std::vector<cv::Mat> last_masked_frames;
    cv::namedWindow("Debug View");
    cv::moveWindow("Debug View", 600, 280); // 마스크 창들 옆에 위치
    // -------------------------

    while (running.load()) {
        // ROI 설정이 완료되었고, 처리 스레드가 아직 시작되지 않았다면 시작
        if (is_config_ready.load() && !processing_threads_started) {
            std::cout << "✅ Initial configuration received. Starting processing threads..." << std::endl;
            warp_thread_ = std::thread(warp_thread);
            mask_thread_ = std::thread(mask_thread);
            processing_threads_started = true;
        }

        // ---- debug: 원본 프레임 가져와서 표시 ----
        std::shared_ptr<cv::Mat> debug_frame;
        if (debug_frame_queue.try_pop(debug_frame)) {
            cv::imshow("Debug View", *debug_frame);
        }
        // --------------------------------------

        if (is_config_ready.load()) {
            std::shared_ptr<std::vector<cv::Mat>> masked;
            if (masked_queue.try_pop(masked)) {
                check_bus_platform(*masked, BUS_PLATFORM_STATUS);
                process_bus_status(PLATFORM_SIZE, BUS_PLATFORM_STATUS);
                update_shared_status(PLATFORM_SIZE);
                
                // 콘솔 디버그 출력
                std::cout << "--- Platform Status Updated (Stable) ---" << std::endl;
                for (unsigned int i = 0; i < PLATFORM_SIZE; i++) {
                    std::cout << "  Platform " << i << ": " << (stable_status[i] ? "BUS DETECTED" : "Empty") << std::endl;
                }
                last_masked_frames = *masked;
            }
        }

        // Debug: 각 플랫폼의 마스크 창
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

        if (cv::waitKey(1) == 'q') {
            running = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // CPU 과점유 방지용
    }

    std::cout << "Shutting down all threads..." << std::endl;
    if(reader_thread.joinable()) reader_thread.join();
    if(warp_thread_.joinable()) warp_thread_.join();
    if(mask_thread_.joinable()) mask_thread_.join();

    std::cout << "Shutting down config thread..." << std::endl;
    if(roi_config_thread.joinable()) roi_config_thread.join();

    cleanup_status_shm();
    cv::destroyAllWindows();

    std::cout << "Program done. Total frames: " << total_frame << ", Wasted: " 
              << frame_wasted << "/" << warped_wasted << "/" << masked_wasted << std::endl;
    return 0;
}
