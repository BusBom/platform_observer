cmake_minimum_required(VERSION 3.10)
project(station_checker)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# OpenCV 찾기
find_package(OpenCV REQUIRED)

# pkg-config 사용하여 libcamera 설정 (주석 해제 및 실제 사용)
find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# 실행 파일 생성
add_executable(station_checker 
    main.cpp 
    filters.cpp 
    checker.cpp

    checker.hpp
    filters.hpp
    safeQueue.hpp
    stop_status.hpp
)

# 포함 디렉터리 설정
target_include_directories(station_checker PRIVATE
    ${OpenCV_INCLUDE_DIRS}
#    ${LIBCAMERA_INCLUDE_DIRS}
)

# 라이브러리 링크
target_link_libraries(station_checker PRIVATE
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    pthread      # POSIX 쓰레드를 사용하는 경우
    rt                  # 공유 메모리 함수용 librt
    X11
)