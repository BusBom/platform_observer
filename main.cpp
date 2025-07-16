//clone 하는 부분 최대한 출일 것
#include "safeQueue.hpp"
#include "filters.hpp"

#include <iostream>
#include <chrono>
#include <queue>
#include <thread>

#define MAX_QUEUE_SIZE 1000

const char cap_name[] = "test.MP4";
bool is_capture_thread_running = true;

static ThreadSafeQueue<cv::Mat> frame_queue;
static std::vector<cv::Point> clicked_points;
static ThreadSafeQueue<cv::Mat> warped_queue;
static ThreadSafeQueue<cv::Mat> masked_queue;

void onMouseClick(int event, int x, int y, int flags, void *userdata);
void capture_thread();
void warp_thread();
void mask_thread();

int main()
{
    cv::VideoCapture cap(cap_name);
    cv::Mat firstImage;
    cap >> firstImage;
    cap.release();
    
    cv::namedWindow("original", cv::WINDOW_NORMAL);
    cv::resizeWindow("original", 1000, 800);
    cv::setMouseCallback("original", onMouseClick, &clicked_points);
    cv::imshow("original", firstImage);

    cv::namedWindow("mask", cv::WINDOW_NORMAL);
    cv::resizeWindow("mask", 1000, 800);

    while(clicked_points.size() != 4){
        cv::waitKey(30);
    }

    std::thread tcapture(capture_thread);
    std::thread twarp(warp_thread);
    std::thread tmask(mask_thread);

    tcapture.join();
    twarp.join();
    tmask.join();

    cv::destroyAllWindows();
    return 0;
}

void onMouseClick(int event, int x, int y, int flags, void *userdata)
{
    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    std::vector<cv::Point> *points_vec = static_cast<std::vector<cv::Point> *>(userdata);

    if (points_vec->size() >= 4)
    {
        std::cout << "🔄 Point list is full. Clearing list." << std::endl;
        points_vec->clear();
    }

    points_vec->push_back(cv::Point(x, y));
    std::cout << "📌 Point added: (" << x << ", " << y << "). Total points: " << points_vec->size() << std::endl;
}

void capture_thread()
{
    cv::VideoCapture cap(cap_name);

    if (!cap.isOpened())
    {
        std::cout << "ERROR: Could not open camera\n";
        is_capture_thread_running = false;
        return;
    }
    else
    {
        is_capture_thread_running = true;
    }

    const int target_fps = 30;
    const std::chrono::milliseconds frame_duration(1000 / target_fps);
    cv::Mat frame, garbage;
    while (true)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        if (!cap.read(frame) || frame.empty())
        {
            break;
        }

        if (frame_queue.size() > MAX_QUEUE_SIZE)
        {
            frame_queue.try_pop(garbage);
        }
        frame_queue.push(frame.clone());

        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        std::cout << "org: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() 
            << " : " << frame_queue.size() << '\n';
        auto sleep_time = frame_duration - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

        if (sleep_time > std::chrono::milliseconds(0))
        {
            std::this_thread::sleep_for(sleep_time); // CPU 낭비 방지
        }
    }

    cap.release();
    is_capture_thread_running = false;
}

void warp_thread()
{
    const int target_fps = 30;
    const std::chrono::milliseconds frame_duration(1000 / target_fps);

    cv::Mat frame, warped, garbage;
    while (is_capture_thread_running)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        if (clicked_points.size() != 4) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;  // 아직 포인트가 준비 안됨
        }

        if (warped_queue.size() > MAX_QUEUE_SIZE)
        {
            warped_queue.try_pop(garbage);
        }

        if (frame_queue.try_pop(frame))
        {
            warp_rectified_area(frame, warped, clicked_points);
            warped_queue.push(warped);
        }

        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        std::cout << "warp: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
            << " : " << warped_queue.size() << '\n';
        auto sleep_time = frame_duration - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

        if (sleep_time > std::chrono::milliseconds(0))
        {
            std::this_thread::sleep_for(sleep_time); // CPU 낭비 방지
        }
    }
}

void mask_thread()
{
    const int target_fps = 30;
    const std::chrono::milliseconds frame_duration(1000 / target_fps);

    cv::Mat frame, masked_1, masked_2, garbage;
    while (is_capture_thread_running)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        if (masked_queue.size() > MAX_QUEUE_SIZE)
        {
            masked_queue.try_pop(garbage);
        }

        if (warped_queue.try_pop(frame))
        {
            remove_achromatic_area(frame, masked_1); // default : 0.15
            revive_white_area(masked_1, masked_2);   // default : 95%
            masked_queue.push(masked_2);
        }

        auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
        std::cout << "masked: " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
            << " : " << masked_queue.size() << '\n';
        auto sleep_time = frame_duration - std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);

        if (sleep_time > std::chrono::milliseconds(0))
        {
            std::this_thread::sleep_for(sleep_time); // CPU 낭비 방지
        }
    }
}

// DEBUG
/*
std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            warp_rectified_area(frame, warped, clicked_points);
            remove_achromatic_area(warped, mask, 0.15);
            revive_white_area(mask, remask);
            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();;
            std::chrono::duration<double, std::milli> msec = end - start;
*/
