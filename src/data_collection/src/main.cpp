#include "data_collection/camera_collector.h"

// start the CameraManager object and get the camera data flowing
int main() {

    std::shared_ptr<collection::CameraCollector> camera_collector_ptr = std::make_shared<collection::CameraCollector>("Data/");
    camera_collector_ptr->startCollecting(100);
    return 0;
}