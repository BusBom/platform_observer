# ROI Picker (Region of Interest Configuration Tool)

## 1. Purpose

This tool allows you to accurately specify the four vertices of each platform's ROI by simply clicking on a video file. The generated JSON file is intended to be used by a web-based configuration page to send ROI information to the main station\_checker program.

## 2. Build Instructions

### Requirements

* A C++11 (or newer) compatible compiler (e.g., g++)
* OpenCV 4.x library
* nlohmann/json library (for JSON handling)

### Preparing the nlohmann/json library

If not already installed on your system, you only need to download a single header file.

```bash
wget https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp
# Move the downloaded json.hpp to your project's include directory
```

### Example Compile Command

You may need to specify the path to the nlohmann/json header using the `-I` option.

```bash
g++ -o roi_picker roi_picker.cpp `pkg-config --cflags --libs opencv4` -std=c++11
```

## 3. Usage

### Prepare Video File

In the `roi_picker.cpp` source code, specify the path to the video file you want to use for configuration in the `video_path` variable.

### Run the Program

```bash
./roi_picker
```

### Configure ROIs

* When the program starts, the first frame of the video will be displayed in a paused state.
* Click on the four vertices of a platform's ROI in sequence.
* Once all four points are clicked, the platform ROI is complete and will be automatically saved to `platform_rois.json`.
* You can continue to add ROIs for other platforms.

### Shortcut Keys

| Key      | Function       | Description                                                  |
| -------- | -------------- | ------------------------------------------------------------ |
| Spacebar | Play / Pause   | Play or pause the video to find the desired scene.           |
| n        | Next Frame     | (In paused state) Move forward one frame at a time.          |
| b        | Previous Frame | (In paused state) Move backward one frame at a time.         |
| s        | Manual Save    | Manually save all ROIs created so far.                       |
| z        | Undo Point     | (In paused state) Undo the last point clicked.               |
| r        | Reset Platform | (In paused state) Clear all points for the current platform. |
| q or ESC | Quit           | Exit the program.                                            |

## 4. Output File

`platform_rois.json`: This is the output file containing the results of your work. It is saved in the following format:

```json
{
    "stop_rois": [
        [
            [x1, y1],
            [x2, y2],
            [x3, y3],
            [x4, y4]
        ],
        [
            [x1, y1],
            [x2, y2],
            [x3, y3],
            [x4, y4]
        ]
    ]
}
```
