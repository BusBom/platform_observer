# Platform Observer (Bus Detection System)

**Detects stopped buses at station platforms in real-time using CCTV video streams and dynamically configurable ROIs.**\
Detection results are written to shared memory for use by other processes (e.g., CGI scripts).

---

## Key Features

- **Dual Video Input Modes**

  - **Live Mode**: Receives real-time frames via shared memory (`/busbom_frame`)
  - **Video File Mode**: Reads local `.mp4` file for debugging (passed as command-line argument)

- **Dynamic ROI Configuration**

  - Receives ROI updates via Unix domain socket (`/tmp/roi_socket`) from external tools like web CGI
  - No restart required to apply new ROI settings

- **Multi-threaded Pipeline**

  - Independent threads for:
    - Video reading
    - Perspective warping
    - Filtering & masking
    - State analysis

- **Robust State Stabilization (Debouncing)**

  - Stop condition is confirmed only if the bus is detected continuously for a set duration (e.g., 3 seconds)
  - Up to 5 temporary detection losses (flickers) are tolerated within this window

- **Shared Memory Output**

  - Final result (`StopStatus`) is written to `/busbom_status`
  - Used by CGI scripts or monitoring processes

---

## System Architecture

<img width="5364" height="3004" alt="image" src="https://github.com/user-attachments/assets/68cb192d-a82f-4075-97bf-1ec9ec086581" />

---

## Requirements

- C++11 or newer
- OpenCV 4.x
- CMake ≥ 3.10
- POSIX threads (pthreads)

---

## Build Instructions

```bash
# Move to the project directory
cd /path/to/platform_observer

# Create and move into the build directory
mkdir -p build
cd build

# Run CMake and build the project
cmake ..
make
```

> Output binary: `build/station_checker`

---

## How to Run

### 1. Live Mode (Default)

Used in production where another process streams frames to `/busbom_frame`.

```bash
./station_checker
```

### 2. Video File Mode (Debug)

Used for testing with `.mp4` files. Video must be placed under:

```
/home/Qwd/platform_observer/video/
```

Example:

```bash
./station_checker IMG_4403.mp4
```

---

## Processing Pipeline Overview

```
          +----------------------+
          |   Video Source       | <- Shared Memory or File
          +----------+-----------+
                     |
                [frame_queue]
                     |
              warp_thread
                     ↓
              [warped_queue]
                     ↓
              mask_thread
                     ↓
              [masked_queue]
                     ↓
          Main Thread (Analysis)
                     ↓
         Shared Memory (/busbom_status)
```

---

## Detailed Flow Description

1. **Video Input**

   - Reads from shared memory or a video file into `frame_queue`

2. **ROI Configuration**

   - Listens to `/tmp/roi_socket` for ROI configuration
   - Updates platform count and ROI coordinates in real-time

3. **Processing Threads**

   - `warp_thread`: Applies perspective transformation
   - `mask_thread`: Performs color filtering and masking

4. **Detection & Stabilization**

   - Evaluates bus presence in each platform ROI
   - Debounces results with time-based confirmation and flicker tolerance

5. **Shared Memory Output**

   - Writes detection result to `/busbom_status`
   - Readable by other components such as CGI scripts
