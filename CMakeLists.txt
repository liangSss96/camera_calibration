cmake_minimum_required(VERSION 3.16)
project(camera_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

## 可执行目标：实时采集与标定工具。
add_executable(camera_calibration
    src/main.cpp
    src/calibration/calibrator.cpp
    src/calibration/frame_selector.cpp
    src/config/app_config.cpp
    src/report/report_writer.cpp
)

## 头文件路径与链接库。
target_include_directories(camera_calibration PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(camera_calibration PRIVATE ${OpenCV_LIBS})
