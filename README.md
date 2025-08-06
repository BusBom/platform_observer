# 정류장 버스 감지 시스템 (Platform Observer)

CCTV 영상 스트림과 동적 ROI(관심 영역) 설정을 기반으로 정류장 플랫폼의 버스 정차 유무를 실시간으로 감지하고, 그 결과를 공유 메모리에 기록하는 프로그램입니다.

## 주요 기능

-   **두 가지 영상 입력 모드 지원**
    -   **라이브 모드**: 다른 프로세스로부터 공유 메모리(` /busbom_frame`)를 통해 실시간 영상 프레임을 수신합니다.
    -   **비디오 파일 모드**: 프로그램 실행 시 로컬에 저장된 영상 파일(`.mp4` 등)을 함께 입력하여 디버깅 및 테스트를 수행할 수 있습니다.
-   **동적 ROI 설정**: 웹 CGI 등 외부 프로그램으로부터 유닉스 도메인 소켓(`/tmp/roi_socket`)을 통해 실시간으로 감지 영역(ROI)을 수신하고 적용합니다.
-   **멀티스레드 기반 병렬 처리**: 영상 읽기, 투시 변환, 필터링, 상태 판단 등 각 처리 단계를 별도의 스레드로 구성하여 성능을 최적화했습니다.
-   **상태 안정화 (Debouncing)**: 단순 프레임 카운트가 아닌, 설정된 시간(예: 2초) 이상 감지되어야 정차로 판단합니다. 또한, 버스가 일시적으로 가려지는 경우를 대비해 짧은 감지 손실을 허용하여 안정성을 높였습니다.
-   **공유 메모리를 통한 결과 출력**: 최종 감지 결과(`StopStatus` 구조체)를 공유 메모리(`/busbom_status`)에 기록하여, 웹 서버 CGI 스크립트 등 다른 프로세스가 쉽게 상태를 조회할 수 있도록 합니다.


## 시스템 구조도

<img width="5368" height="3008" alt="image" src="https://github.com/user-attachments/assets/2ae1cd16-7c6e-4799-9b81-b70dafae1975" />

## 요구 사항
-   C++11 이상을 지원하는 컴파일러 (g++)
-   OpenCV 4.x 라이브러리
-   CMake 3.10 이상
-   pthreads (멀티스레딩)

## 빌드 방법(CMake)

### 빌드 명령어
```
# 프로젝트 루트 디렉터리로 이동
cd /path/to/platform_observer

# 빌드 디렉터리 생성 및 이동
mkdir -p build
cd build

# CMake 실행 및 빌드
cmake ..
make
```

빌드가 성공하면 build 디렉터리 내에 station_checker 실행 파일이 생성됩니다.

### 컴파일 명령어 예시

```bash
g++ -o platform_observer platform_main.cpp `pkg-config --cflags --libs opencv4` -std=c++11 -pthread
```

---

## 실행 방법

프로그램은 실행 시 인자의 유무에 따라 두 가지 모드로 동작합니다.

### 1. 라이브 모드 (기본)

다른 프로세스(예: GStreamer)가 `/busbom_frame` 공유 메모리로 영상 프레임을 송출하는 환경에서 사용합니다.

```./platform_observer```


### 2. 비디오 파일 모드 (디버깅용)

저장된 영상 파일을 이용해 테스트할 때 사용합니다. 프로그램 실행 시 인자로 **영상 파일 이름**을 전달합니다.

-   **참고**: 코드 내에 기본 영상 경로가 `/home/Qwd/platform_observer/video/`로 설정되어 있습니다.

** /home/Qwd/platform_observer/video/IMG_4403.mp4 파일을 재생 **

```./platform_observer IMG_4403.mp4```

---

## 동작 방식

1.  **영상 입력**: 라이브 모드 또는 비디오 파일 모드로 영상 프레임을 `frame_queue`에 지속적으로 추가합니다.
2.  **ROI 설정**: `/tmp/roi_socket`을 통해 외부에서 ROI 설정 명령을 대기합니다. 설정이 수신되면 감지할 플랫폼의 개수와 좌표(`platform_rois`)가 갱신됩니다.
3.  **병렬 처리**:
    -   `warp_thread`: `frame_queue`에서 프레임을 가져와 설정된 ROI에 맞춰 투시 변환을 수행하고 `warped_queue`에 넣습니다.
    -   `mask_thread`: `warped_queue`에서 이미지를 가져와 색상 필터링 등 전처리를 수행하고 `masked_queue`에 넣습니다.
4.  **상태 판단 및 공유**:
    -   `main` 스레드는 `masked_queue`에서 최종 처리된 이미지를 가져와 버스 유무를 판단합니다.
    -   판단 결과는 시간 기반 및 감지 손실 허용 로직을 통해 안정화됩니다.
    -   최종 상태는 `StopStatus` 구조체 형식으로 `/busbom_status` 공유 메모리에 실시간으로 업데이트됩니다.
