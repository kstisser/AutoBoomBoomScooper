#include <rclcpp/rclcpp.hpp>
#include "vision_package/camera_manager.h"

// start the CameraManager object and get the camera data flowing
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); //init ROS2

    std::shared_ptr<vision::CameraManager> camera_manager_ptr = std::make_shared<vision::CameraManager>();
    camera_manager_ptr->startDataFlow();

    rclcpp::spin(camera_manager_ptr);

    rclcpp::shutdown();
    return 0;
}