#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <chrono>
#include <thread> 

namespace vision {

class CameraManager: public rclcpp::Node
{
    public:
    CameraManager();
    ~CameraManager() = default;

    void startDataFlow();
    void configureRealsense();

    private:
        rs2::pipeline _camera_pipeline;
        rs2::config _cam_cfg;
        image_transport::Publisher _img_pub;
        image_transport::Publisher _color_img_pub;

        static void signal_handler(int signal);

    protected:

};

}