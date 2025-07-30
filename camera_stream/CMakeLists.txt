cmake_minimum_required(VERSION 3.10)
project(camera_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_stream main.cpp)

target_link_libraries(camera_stream 
    ${OpenCV_LIBS} 
    Threads::Threads rt
    nlohmann_json::nlohmann_json
)
