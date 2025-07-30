#include <opencv2/opencv.hpp>      // OpenCV 주요 헤더
#include <nlohmann/json.hpp>
#include <thread>                  // std::thread 사용
#include <mutex>                   // std::mutex, std::unique_lock
#include <condition_variable>      // std::condition_variable
#include <deque>                   // std::deque 컨테이너
#include <cstring>                 // std::memcpy
#include <iostream>                // std::cout, std::cerr
#include <csignal>                 // std::signal
#include <atomic>                  // std::atomic
#include <cstdlib>                 // setenv
#include <fstream>


#include <fcntl.h>                 // shm_open
#include <sys/mman.h>              // mmap
#include <unistd.h>                // ftruncate, close
#include <sys/stat.h>              // fchmod, umask
#include <sys/socket.h>            // socket API
#include <sys/un.h>                // sockaddr_un
#include <arpa/inet.h>             // htonl
#include <signal.h>

// ----- Shared Memory 및 Socket 설정 -----
#define CONFIG_FILE "camera_config.json"
#define SHM_NAME "/busbom_frame"
#define FRAME_WIDTH  1280
#define FRAME_HEIGHT 720
#define FRAME_CHANNELS 3
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT * FRAME_CHANNELS)
#define SOCKET_PATH "/tmp/camera_socket"


// ----- 전역 변수 및 동기화 객체 -----
std::mutex mtx;
std::condition_variable cv_response; // 캡처 -> 소켓, 프리뷰 응답 알림
std::condition_variable cv_writer;   // 캡처 -> 라이터, 새 프레임 알림

// 데이터 교환을 위한 공유 변수
std::deque<std::shared_ptr<cv::Mat>> frame_queue; // 라이터 스레드용 큐
nlohmann::json global_camera_json;         // "실적용"된 메인 스트림 설정
std::optional<nlohmann::json> request_json; // 소켓의 프리뷰 요청
std::optional<cv::Mat> before_frame_response;      // 소켓을 위한 프리뷰 응답

std::atomic<bool> running{true};
int server_fd = -1;
void* shm_ptr = nullptr;

/**
 * @brief SIGINT (Ctrl+C) 시그널을 처리하여 프로그램을 안전하게 종료합니다.
 */
void signal_handler(int signum) {
    if (!running.exchange(false)) return;
    std::cout << "\n[Signal Detected] Shutting down..." << std::endl;
    
    {
        std::ofstream config(CONFIG_FILE);
        if (config.is_open()) {
            std::unique_lock<std::mutex> lk(mtx);
            config << global_camera_json.dump(4);
        } else {
            std::cerr << "[ERROR] Cannot save config." << std::endl;
        }
    }

    cv_response.notify_all();
    cv_writer.notify_all();

    if (server_fd != -1) {
        shutdown(server_fd, SHUT_RDWR);
        close(server_fd);
    }
}

/**
 * @brief JSON 설정을 카메라 객체에 적용하는 함수
 * @param cap 설정할 카메라 객체 (참조)
 * @param settings 적용할 설정이 담긴 JSON 객체 (상수 참조)
 * @param b last_brightness 마지막 밝기 값 (참조)
 * @param c last_contrast 마지막 대비 값 (참조)
 * @param e last_exposure 마지막 노출 값 (참조)
 * @param s last_saturation 마지막 채도 값 (참조)
 * @return 설정값이 실제로 변경되었으면 true, 아니면 false
 */
bool apply_settings(cv::VideoCapture& cap, const nlohmann::json& settings, int& b, int& c, int& e, int& s) {
    bool changed = false;
    if (settings.contains("camera")) {
        const auto& cam_s = settings["camera"];
        int br = cam_s.value("brightness", b); if(br != b) { cap.set(cv::CAP_PROP_BRIGHTNESS, br); b = br; changed = true; }
        int ct = cam_s.value("contrast", c); if(ct != c) { cap.set(cv::CAP_PROP_CONTRAST, ct); c = ct; changed = true; }
        int ex = cam_s.value("exposure", e); if(ex != e) { cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); cap.set(cv::CAP_PROP_EXPOSURE, ex); e = ex; changed = true; }
        int st = cam_s.value("saturation", s); if(st != s) { cap.set(cv::CAP_PROP_SATURATION, st); s = st; changed = true; }
    }
    return changed;
}

/**
 * @brief 카메라에서 프레임을 지속적으로 캡처하여 큐에 추가하는 스레드 함수.
 * camera_json 객체를 참조하여 카메라 하드웨어 설정을 동적으로 변경합니다.
 */
void capture_thread() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "FATAL ERROR: Cannot open camera." << std::endl;
        running.store(false);
        cv_writer.notify_all();
        cv_response.notify_all();
        return;
    }
    std::cout << "[CAPTURE] Camera device opened successfully." << std::endl;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_FPS, 15);          // 프레임 제한
    std::cout << "[CAPTURE] Initial camera setup complete." << std::endl;

    // 초기 안정화
    std::cout << "[CAPTURE] Stabilizing camera..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        cv::Mat dummy_frame;
        if (!cap.read(dummy_frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    std::cout << "[CAPTURE] Stabilization complete. Starting main loop." << std::endl;

    int current_b = -1, current_c = -1, current_e = -1, current_s = -1;

    while (running.load()) {
        // 1. 현재 설정으로 메인 스트림 프레임 캡처
        bool settings_were_changed = apply_settings(cap, global_camera_json, current_b, current_c, current_e, current_s);
        
        // [핵심] 설정이 실제로 변경되었을 때만 안정화 작업을 수행
        if (settings_were_changed) {
            cap.grab();
            cap.grab();
            std::cout << "[CAPTURE] Settings changed:\n" << global_camera_json.dump(4) << std::endl;
        }
        
        cv::Mat current_frame;
        if (!cap.read(current_frame) || current_frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        

        // 소켓으로부터 변경 사항이 있는 지 확인
        {
            std::unique_lock<std::mutex> lk(mtx);

            if (request_json.has_value()) {
                // [Before] 현재 캡처한 프레임을 "Before" 스냅샷으로 사용
                before_frame_response = current_frame.clone();
                cv_response.notify_one(); // 소켓 스레드에 "Before" 스냅샷 준비 완료 알림

                // [After] 요청받은 새 설정을 메인 스트림에 적용
                std::cout << "[CAPTURE] Applying new settings..." << std::endl;
                global_camera_json.merge_patch(*request_json);
                request_json.reset();

            }

            //cv::imshow("Preview", current_frame);
            //cv::waitKey(1);

            // 메인 스트림 프레임을 writer 스레드에 전달
            if (frame_queue.size() >= 2) frame_queue.pop_front();
            frame_queue.push_back(std::make_shared<cv::Mat>(current_frame));
        }
        cv_writer.notify_one();
    }
    cap.release();
    std::cout << "[CAPTURE] Capture thread finished and camera resource released." << std::endl;
}

/**
 * @brief 큐에서 프레임을 가져와 Shared Memory에 쓰는 스레드 함수.
 */
void writer_thread(cv::VideoWriter* writer) {
    while (running.load()) {
        std::shared_ptr<cv::Mat> latest_frame;
        {
            std::unique_lock<std::mutex> lk(mtx);
            cv_writer.wait(lk, [] {
                return !frame_queue.empty() || !running.load();
            });

            if (!frame_queue.empty()) {
                latest_frame = frame_queue.back();  // 가장 최신 프레임 가져오기
                frame_queue.clear();                // 이전 프레임은 모두 삭제
            }
        }
        // 프레임이 있으면 처리
        if (latest_frame) {
            std::memcpy(shm_ptr, latest_frame->data, FRAME_SIZE); // 공유 메모리로 복사
            writer->write(*latest_frame);                         // 파일로 저장
        }
    }
}

/**
 * @brief Unix 소켓을 통해 클라이언트의 요청을 처리하는 스레드 함수.
 * JSON 설정을 수신하고, 적용 후 한 프레임을 클라이언트로 전송합니다.
 */

void socket_thread() {
    server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd == -1) { perror("socket"); running.store(false); cv_writer.notify_all(); return; }

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    unlink(SOCKET_PATH);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) { perror("bind"); close(server_fd); running.store(false); cv_writer.notify_all(); return; }
    if (listen(server_fd, 5) == -1) { perror("listen"); close(server_fd); running.store(false); cv_writer.notify_all(); return; }

    std::cout << "[SOCKET] Waiting for client connection on " << SOCKET_PATH << std::endl;

    while (running.load()) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd == -1) break;


        char buffer[1024] = {0};
        if (read(client_fd, buffer, sizeof(buffer) - 1) > 0) {
            try {
                nlohmann::json req = nlohmann::json::parse(buffer);
                std::cout << "[SOCKET] Settings change request received." << std::endl;
                {
                    std::unique_lock<std::mutex> lk(mtx);
                    request_json = req;
                    before_frame_response.reset();
                }

                std::optional<cv::Mat> before_frame;
                {
                    std::unique_lock<std::mutex> lk(mtx);
                    if (cv_response.wait_for(lk, std::chrono::seconds(2), []{ return before_frame_response.has_value(); })) {
                        before_frame = *before_frame_response;
                    } else {
                        std::cerr << "[SOCKET] WARNING: 'Before' frame response timeout!" << std::endl;
                    }
                }

                if (before_frame.has_value()) {
                    std::vector<uchar> jpeg_buffer;
                    std::vector<int> encode_params = { cv::IMWRITE_JPEG_QUALITY, 60 };  // 해상도 낮음
                    
                    cv::Mat preview_small;
                    cv::resize(*before_frame, preview_small, cv::Size(640, 360));

                    if (cv::imencode(".jpg", preview_small, jpeg_buffer, encode_params)) {
                        uint8_t type = 1;
                        uint32_t size = htonl(jpeg_buffer.size());

                        write(client_fd, &type, sizeof(type));
                        write(client_fd, &size, sizeof(size));
                        write(client_fd, jpeg_buffer.data(), jpeg_buffer.size());

                        std::cout << "[SOCKET] 'Before' snapshot sent successfully." << std::endl;
                    }
                }
            } catch (const std::exception& e) { std::cerr << "[SOCKET] Exception occurred: " << e.what() << std::endl; }
        }
        close(client_fd);
    }
    unlink(SOCKET_PATH);
    std::cout << "[SOCKET] Socket thread finished." << std::endl;
}


/**
 * @brief 프로그램 시작 시 설정을 불러오거나 하드웨어 기본값을 읽는 메인 함수
 */

int main() {
    // SIGINT(Ctrl+C) 핸들러 등록
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    // 프로그램 시작 시 설정 파일 또는 하드웨어 기본값 불러오기
    {
        std::unique_lock<std::mutex> lk(mtx);
        std::ifstream config_file(CONFIG_FILE);
        if (config_file.is_open()) {
            try {
                global_camera_json = nlohmann::json::parse(config_file);
                std::cout << "[LOAD] Loaded settings from " << CONFIG_FILE << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Failed to parse config file, resetting to hardware defaults: " << e.what() << std::endl;
                goto read_hardware_defaults;
            }
        } else {
read_hardware_defaults:
            std::cout << "[INIT] Config file not found. Reading hardware default values." << std::endl;
            cv::VideoCapture temp_cap(0, cv::CAP_V4L2);
            if (temp_cap.isOpened()) {
                global_camera_json["camera"]["brightness"] = temp_cap.get(cv::CAP_PROP_BRIGHTNESS);
                global_camera_json["camera"]["contrast"]   = temp_cap.get(cv::CAP_PROP_CONTRAST);
                global_camera_json["camera"]["exposure"]   = temp_cap.get(cv::CAP_PROP_EXPOSURE);
                global_camera_json["camera"]["saturation"] = temp_cap.get(cv::CAP_PROP_SATURATION);
                temp_cap.release();
                
                std::cout << "[CREATE] Creating " << CONFIG_FILE << " with read hardware default values." << std::endl;

                std::cout << global_camera_json.dump(4) << std::endl;
                std::ofstream new_config_file(CONFIG_FILE);
                new_config_file << global_camera_json.dump(4);
            } else {
                std::cerr << "[ERROR] Failed to read hardware defaults. Using safe defaults from code." << std::endl;
                global_camera_json = {{"camera", {{"brightness", 50}, {"contrast", 10}, {"exposure", 0}, {"saturation", 10}}}};
            }
        }
    }

    // ----- 공유 메모리 생성 및 설정 -----
    umask(0); // 공유 메모리 이름이 이미 존재할 경우 제거
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return 1;
    }

    // 크기 설정
    if (ftruncate(shm_fd, FRAME_SIZE) == -1) {
        perror("ftruncate");
        return 1;
    }

    if (fchmod(shm_fd, 0666) == -1) { 
        perror("fchmod"); /* 계속 진행 */
    }  

    // 메모리 매핑
    shm_ptr = mmap(0, FRAME_SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    // VideoWriter 객체 생성
    std::string filename = "output.mp4";
    cv::VideoWriter writer(filename,
                           cv::VideoWriter::fourcc('m','p','4','v'),
                           10,
                           cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

    if (!writer.isOpened()) {
        std::cerr << "Failed to initialize VideoWriter" << std::endl;
        return -1;
    }

    std::thread t_cap(capture_thread);                  // RTSP 캡처 스레드 시작
    std::thread t_write(writer_thread, &writer);        // Shared Memory 쓰기 스레드 시작
    std::thread t_sock(socket_thread);                  // Socket 통신 시작

    std::cout << "Starting frame capture, shared memory writer, and socket threads." << std::endl;
    std::cout << ">>>>>  Press Ctrl+C to exit  <<<<<" << std::endl;

    t_cap.join();       // 캡처 스레드 종료 대기
    t_write.join();     // 쓰기 스레드 종료 대기
    t_sock.join();    // 소켓 스레드 종료 대기

    // ----- 정리 -----
    munmap(shm_ptr, FRAME_SIZE);
    writer.release(); // VideoWriter 자원 해제
    close(shm_fd);
    shm_unlink(SHM_NAME); // 실제 운영 시에는 제거하지 않을 수도 있음

    return 0;
}