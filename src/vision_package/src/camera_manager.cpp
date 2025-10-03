#include "vision_package/camera_manager.h"
#include <iostream>

namespace vision {
CameraManager::CameraManager()
    : rclcpp::Node("camera_manager") {
    _img_pub = image_transport::create_publisher(this, "/realsense_depth_img");
    _color_img_pub = image_transport::create_publisher(this, "/realsense_color_img");

    configureRealsense();
    
    //handles control-C for clean exit
    std::signal(SIGINT, signal_handler);
}

void CameraManager::configureRealsense() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    _camera_pipeline.start(cfg);  
}

void CameraManager::startDataFlow() {

    while(rclcpp::ok()) {
        rs2::frameset frames;
        if (_camera_pipeline.poll_for_frames(&frames)) {
            rs2::depth_frame df = frames.get_depth_frame();
            float width = df.get_width();
            float height = df.get_height();
            std::cout << "Width: " << width << ", Height: " << height << std::endl;
            cv::Mat depth_image(cv::Size(width, height), CV_16UC1, (void*)df.get_data(), cv::Mat::AUTO_STEP);

            //normalizing for viewing
            //cv::Mat normalized_image;
            //depth_image.convertTo(normalized_image, CV_8UC1, 255.0 / 10000.0);
            
            cv_bridge::CvImage cv_img;
            cv_img.header.stamp = this->now();               // Use ROS2 node's clock
            cv_img.header.frame_id = "camera_depth_frame";   // Frame ID
            //depth map for coding purposes
            cv_img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            //changing encoding for viewing purposes
            //cv_img.encoding = sensor_msgs::image_encodings::MONO8;
            cv_img.image = depth_image;

            sensor_msgs::msg::Image::SharedPtr ros_img_msg = cv_img.toImageMsg();

            _img_pub.publish(ros_img_msg);

            //Now focus on color image
            rs2::video_frame color_frame = frames.get_color_frame();
            cv::Mat color_image(
                cv::Size(color_frame.get_width(), color_frame.get_height()),
                CV_8UC3,
                (void*)color_frame.get_data(),
                cv::Mat::AUTO_STEP
            );
            cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
            cv_bridge::CvImage cv_img2;
            cv_img2.header.stamp = this->now();
            cv_img2.header.frame_id = "camera_color_frame";
            cv_img2.encoding = sensor_msgs::image_encodings::BGR8;
            cv_img2.image = color_image;

            sensor_msgs::msg::Image::SharedPtr ros_img_msg2 = cv_img2.toImageMsg();
            _color_img_pub.publish(ros_img_msg2);

            // Sleep for a short time so the CPU isn't hounded
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void CameraManager::signal_handler(int signal) {
    std::cout << "Got signal " << signal << std::endl;
    std::exit(0);
}
}