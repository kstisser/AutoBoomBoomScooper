#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <signal.h>

namespace collection {
class CameraCollector
{
    public:
    CameraCollector(const std::string& dir_path);
    ~CameraCollector() = default;

    void startCollecting(int sleep);
    static void signal_handler(int signal);

    private:
    void configureRealsense();
    void saveFrames(const cv::Mat& color, const cv::Mat& depth);

    std::string _output_dir;
    rs2::pipeline _camera_pipeline;

    protected:
};
}