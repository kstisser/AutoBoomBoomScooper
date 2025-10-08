#include "data_collection/camera_collector.h"

namespace collection {
CameraCollector::CameraCollector(const std::string& dir_path):_output_dir(dir_path) {
    configureRealsense();

    if (!std::filesystem::exists(_output_dir)) {
        if (!std::filesystem::create_directories(_output_dir)) {
            throw std::runtime_error("Failed to create directory: " + _output_dir);
        }
    }

    //handles control-C for clean exit
    signal(SIGINT, signal_handler);
} 

void CameraCollector::configureRealsense() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    _camera_pipeline.start(cfg);  
}

void CameraCollector::startCollecting(int sleep_time) {
    while (true) {
        rs2::frameset frames;
        if (_camera_pipeline.poll_for_frames(&frames)) {

            rs2::frameset frames = _camera_pipeline.wait_for_frames();

            rs2::frame color_frame = frames.get_color_frame();
            rs2::frame depth_frame = frames.get_depth_frame();

            // Convert to OpenCV matrices
            cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth(cv::Size(640, 480), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            // Visualize depth for display
            cv::Mat depth_colormap;
            cv::normalize(depth, depth_colormap, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(depth_colormap, depth_colormap, cv::COLORMAP_JET);

            // Show frames
            cv::imshow("Color Frame", color);
            cv::imshow("Depth Frame", depth_colormap);

            // Wait for key input
            char key = static_cast<char>(cv::waitKey(1));
            if (key == 'c') {
                this->saveFrames(color, depth);
            } else if (key == 'q') {
                std::cout << "Quitting...\n";
                break;
            }

            // Sleep for a short time so the CPU isn't hounded
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time)); 
        }         
    }
    _camera_pipeline.stop();
    cv::destroyAllWindows();  
}

void CameraCollector::saveFrames(const cv::Mat& color, const cv::Mat& depth) {
    static int frame_count = 0;
    ++frame_count;
    std::ostringstream filename;
    filename << std::setw(5) << std::setfill('0') << frame_count;

    std::string color_path = _output_dir + "/color_" + filename.str() + ".png";
    std::string depth_path = _output_dir + "/depth_" + filename.str() + ".png";

    cv::imwrite(color_path, color);
    cv::imwrite(depth_path, depth);

    std::cout << "Saved: " << color_path << " and " << depth_path << std::endl;
    frame_count++;
}

void CameraCollector::signal_handler(int signal) {
    std::cout << "Got signal " << signal << std::endl;
    std::exit(0);
}

}