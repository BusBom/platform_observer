cmake_minimum_required(VERSION 3.10)
project(station_checker)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 찾기
find_package(OpenCV REQUIRED)

# pkg-config 사용하여 libcamera 설정 (주석 해제 및 실제 사용)
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# 소스 파일 자동 수집 
file(GLOB_RECURSE SOURCES
    src/*.cpp
    main.cpp
)

# 포함 디렉토리 설정
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    # ${LIBCAMERA_INCLUDE_DIRS}
)

# 포함 디렉토리 설정
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    # ${LIBCAMERA_INCLUDE_DIRS}
)

# 실행 파일 생성
add_executable(station_checker ${SOURCES})

# 라이브러리 링크
target_link_libraries(station_checker
    ${OpenCV_LIBS}
    # ${LIBCAMERA_LIBRARIES}
    pthread
    rt
    X11
)