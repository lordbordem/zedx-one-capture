#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <signal.h>
#include <csignal>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"

// Global flag to indicate when the program should terminate
static std::atomic<bool> g_terminate(false);

// Signal handler for graceful termination
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nReceived termination signal. Shutting down gracefully..." << std::endl;
        g_terminate.store(true);
    }
}

// Thread function to handle a single camera
void cameraThread(int camera_id, int rq_width, int rq_height, int rq_fps) {
    // Create configuration for the camera
    oc::ArgusCameraConfig config;
    config.mDeviceId = camera_id;
    config.mFPS = rq_fps;
    config.mWidth = rq_width;
    config.mHeight = rq_height;
    config.verbose_level = 3;
    config.hdr = false;

    // Open the camera
    oc::ArgusBayerCapture camera;
    oc::ARGUS_STATE state = camera.openCamera(config);
    if (state != oc::ARGUS_STATE::OK) {
        std::cerr << "Camera " << camera_id << ": Failed to open Camera, error code " << ARGUS_STATE2str(state) << std::endl;
        return;
    }

    std::cout << "Camera " << camera_id << ": Camera opened successfully with resolution: " 
              << camera.getWidth() << "x" << camera.getHeight() 
              << ", channels: " << camera.getNumberOfChannels() << std::endl;

    cv::Mat rgb_frame;
    rgb_frame = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC4, 1);

    int image_count = 0;
    
    while (!g_terminate.load()) {
        if (camera.isNewFrame()) {
            memcpy(rgb_frame.data, camera.getPixels(),
                   camera.getWidth() * camera.getHeight() *
                   camera.getNumberOfChannels());

            // Save the image as JPEG with camera_id and frame number in the filename
            std::string filename = "camera_" + std::to_string(camera_id) + 
                                  "_frame_" + std::to_string(image_count) + ".jpg";
            
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(95); // JPEG quality (0-100)
            
            cv::imwrite(filename, rgb_frame, compression_params);
            
            std::cout << "Camera " << camera_id << ": Saved " << filename << std::endl;
            image_count++;
        }
        else {
            usleep(100);
        }
    }

    camera.closeCamera();
    std::cout << "Camera " << camera_id << ": Camera closed" << std::endl;
}

int main(int argc, char *argv[]) {
    // Set up signal handler for graceful termination
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    int rq_width=0;
    int rq_height=0;
    int rq_fps = 0;

    if (argc > 1) rq_width = atoi(argv[1]);
    if (argc > 2) rq_height = atoi(argv[2]);
    if (argc > 3) rq_fps = atoi(argv[3]);
    int major, minor, patch;
    oc::ArgusVirtualCapture::getVersion(major, minor, patch);
    std::cout << "Argus Capture Version: " << major << "." << minor << "." << patch << std::endl;

    // Get all Argus devices
    std::vector<oc::ArgusDevice> devs = oc::ArgusBayerCapture::getArgusDevices();
    std::cout << "Found " << devs.size() << " Argus devices:" << std::endl;
    for (int i = 0; i < devs.size(); i++) {
        std::cout << "##################" << std::endl;
        std::cout << " Device : " << devs.at(i).id << std::endl;
        std::cout << " Name : " << devs.at(i).name << std::endl;
        std::cout << " Badge : " << devs.at(i).badge << std::endl;
        std::cout << " Available : " << devs.at(i).available << std::endl;
    }
    std::cout << "***********************" << std::endl;

    // Find available cameras
    std::vector<int> available_camera_ids;
    for (const auto& dev : devs) {
        if (dev.available) {
            available_camera_ids.push_back(dev.id);
        }
    }
    
    if (available_camera_ids.empty()) {
        std::cerr << "No available cameras found!" << std::endl;
        return -1;
    }
    
    std::cout << "Found " << available_camera_ids.size() << " available cameras" << std::endl;
    
    // Create threads for each camera
    std::vector<std::thread> camera_threads;
    for (int camera_id : available_camera_ids) {
        std::cout << "Starting thread for camera " << camera_id << std::endl;
        camera_threads.emplace_back(cameraThread, camera_id, rq_width, rq_height, rq_fps);
    }
    
    std::cout << "All camera threads started. Press Ctrl+C to exit." << std::endl;
    
    // Wait for all threads to complete
    for (auto& thread : camera_threads) {
        thread.join();
    }
    
    std::cout << "All camera threads have completed." << std::endl;

    return 0;
}
