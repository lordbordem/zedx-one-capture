#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Structure to hold camera pipeline data
struct CameraPipeline {
    GstElement *pipeline = nullptr;
    GstElement *appsrc = nullptr;
    int frame_count = 0;
    int frame_width = 0;
    int frame_height = 0;
    std::string socket_path;
    std::string stream_name;
    std::mutex mutex;
};

// Global vector to store camera pipelines
static std::vector<std::unique_ptr<CameraPipeline>> camera_pipelines;

// Function to create GStreamer shmsink pipeline
bool createGstreamerShmsinkPipeline(int camera_index, const std::string &socket_path, 
                                   const std::string &stream_name, 
                                   int width = 0, int height = 0) {
    // Ensure camera_index is valid
    if (camera_index >= camera_pipelines.size()) {
        std::cerr << "Invalid camera index: " << camera_index << std::endl;
        return false;
    }

    // Get reference to the camera pipeline
    CameraPipeline *pipeline_data = camera_pipelines[camera_index].get();
    std::lock_guard<std::mutex> lock(pipeline_data->mutex);
    
    // Initialize GStreamer if not already initialized
    static bool gst_initialized = false;
    if (!gst_initialized) {
        GError *error = nullptr;
        if (!gst_init_check(nullptr, nullptr, &error)) {
            std::cerr << "Failed to initialize GStreamer: " << (error ? error->message : "Unknown error") << std::endl;
            if (error) g_error_free(error);
            return false;
        }
        gst_initialized = true;
    }
    
    // Store frame dimensions
    pipeline_data->frame_width = width;
    pipeline_data->frame_height = height;
    pipeline_data->socket_path = socket_path;
    pipeline_data->stream_name = stream_name;
    
    // Create pipeline elements
    pipeline_data->pipeline = gst_pipeline_new(("opencv-to-shmsink-" + std::to_string(camera_index)).c_str());
    pipeline_data->appsrc = gst_element_factory_make("appsrc", ("source-" + std::to_string(camera_index)).c_str());
    GstElement *videoconvert = gst_element_factory_make("videoconvert", ("converter-" + std::to_string(camera_index)).c_str());
    GstElement *shmsink = gst_element_factory_make("shmsink", ("sink-" + std::to_string(camera_index)).c_str());
    
    if (!pipeline_data->pipeline || !pipeline_data->appsrc || !videoconvert || !shmsink) {
        std::cerr << "Failed to create GStreamer elements for camera " << camera_index << std::endl;
        return false;
    }
    
    // Configure appsrc
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                       "format", G_TYPE_STRING, "BGR",
                                       "width", G_TYPE_INT, width,
                                       "height", G_TYPE_INT, height,
                                       "framerate", GST_TYPE_FRACTION, 30, 1,
                                       nullptr);
    
    g_object_set(G_OBJECT(pipeline_data->appsrc),
                "caps", caps,
                "format", GST_FORMAT_TIME,
                "is-live", TRUE,
                nullptr);
    gst_caps_unref(caps);
    
    // Configure shmsink
    g_object_set(G_OBJECT(shmsink),
                "socket-path", socket_path.c_str(),
                "sync", FALSE,
                "wait-for-connection", FALSE,
                "shm-size", 10 * 1024 * 1024, // 10MB buffer
                "stream-name", stream_name.c_str(),
                nullptr);
    
    // Add elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline_data->pipeline), pipeline_data->appsrc, videoconvert, shmsink, nullptr);
    
    // Link elements
    if (!gst_element_link_many(pipeline_data->appsrc, videoconvert, shmsink, nullptr)) {
        std::cerr << "Failed to link GStreamer elements for camera " << camera_index << std::endl;
        gst_object_unref(pipeline_data->pipeline);
        pipeline_data->pipeline = nullptr;
        return false;
    }
    
    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_data->pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "Failed to start GStreamer pipeline for camera " << camera_index << std::endl;
        gst_object_unref(pipeline_data->pipeline);
        pipeline_data->pipeline = nullptr;
        return false;
    }
    
    std::cout << "Camera " << camera_index << ": GStreamer shmsink pipeline initialized with stream name: " << stream_name 
              << " at socket path: " << socket_path << std::endl;
    
    return true;
}

// Function to write OpenCV Mat to GStreamer shmsink pipeline
bool writeToGstreamerShmsink(int camera_index, cv::Mat &frame) {
    // Ensure camera_index is valid
    if (camera_index >= camera_pipelines.size()) {
        std::cerr << "Invalid camera index: " << camera_index << std::endl;
        return false;
    }

    // Get reference to the camera pipeline
    CameraPipeline *pipeline_data = camera_pipelines[camera_index].get();
    std::lock_guard<std::mutex> lock(pipeline_data->mutex);
    
    // Check if frame is valid
    if (frame.empty()) {
        std::cerr << "Error: Cannot write empty frame to GStreamer pipeline for camera " << camera_index << std::endl;
        return false;
    }
    
    // Initialize GStreamer pipeline if not already initialized
    if (pipeline_data->pipeline == nullptr) {
        std::cerr << "Pipeline not initialized for camera " << camera_index << std::endl;
        return false;
    }
    
    // Check if frame dimensions match the pipeline configuration
    if (frame.cols != pipeline_data->frame_width || frame.rows != pipeline_data->frame_height) {
        std::cerr << "Frame dimensions do not match pipeline configuration for camera " << camera_index << std::endl;
        return false;
    }
    
    // Create GstBuffer from OpenCV Mat
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, frame.total() * frame.elemSize(), nullptr);
    if (!buffer) {
        std::cerr << "Failed to allocate GStreamer buffer for camera " << camera_index << std::endl;
        return false;
    }
    
    // Map buffer for writing
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        // Copy frame data to buffer
        memcpy(map.data, frame.data, frame.total() * frame.elemSize());
        gst_buffer_unmap(buffer, &map);
        
        // Set buffer timestamp and duration
        GST_BUFFER_PTS(buffer) = pipeline_data->frame_count * 33333333; // 30fps (in nanoseconds)
        GST_BUFFER_DURATION(buffer) = 33333333;
        
        // Push buffer to appsrc
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(pipeline_data->appsrc), buffer);
        if (ret != GST_FLOW_OK) {
            std::cerr << "Failed to push buffer to GStreamer pipeline for camera " << camera_index << std::endl;
            return false;
        }
        
        pipeline_data->frame_count++;
        return true;
    } else {
        std::cerr << "Failed to map GStreamer buffer for camera " << camera_index << std::endl;
        gst_buffer_unref(buffer);
        return false;
    }
}

// Function to clean up GStreamer resources for a specific camera
void cleanupGstreamerPipeline(int camera_index) {
    if (camera_index >= 0 && camera_index < camera_pipelines.size()) {
        CameraPipeline *pipeline_data = camera_pipelines[camera_index].get();
        if (pipeline_data) {
            std::lock_guard<std::mutex> lock(pipeline_data->mutex);
            
            if (pipeline_data->pipeline) {
                gst_element_set_state(pipeline_data->pipeline, GST_STATE_NULL);
                gst_object_unref(pipeline_data->pipeline);
                pipeline_data->pipeline = nullptr;
                std::cout << "GStreamer pipeline for camera " << camera_index << " cleaned up" << std::endl;
            }
        }
    }
}

// Function to clean up all GStreamer resources
void cleanupAllGstreamerPipelines() {
    for (size_t i = 0; i < camera_pipelines.size(); i++) {
        cleanupGstreamerPipeline(i);
    }
}

// Thread function to handle a camera
void cameraThread(int camera_id, int width, int height, int fps) {
    std::string socket_path = "/dev/shm/sensor_" + std::to_string(camera_id);
    std::string stream_name = "argus_camera_" + std::to_string(camera_id) + "_stream";
    
    // Ensure socket-path is not link, if it is then unlink (remove) it
    if (access(socket_path.c_str(), F_OK) != -1) {
        std::cerr << "Socket path already exists, unlinking " << socket_path << std::endl;
        if (unlink(socket_path.c_str()) != 0) {
            std::cerr << "Unlinking failed, manually deleting the path " << strerror(errno) << std::endl;
            if (rmdir(socket_path.c_str()) != 0) {
                std::cerr << "Remove Dir failed " << strerror(errno) << std::endl;
                return;
            }
        }
    }
    
    // Create camera object
    oc::ArgusBayerCapture camera;
    oc::ArgusCameraConfig config;
    
    // Configure camera
    config.mDeviceId = camera_id;
    config.mFPS = fps;
    config.mWidth = width;
    config.mHeight = height;
    config.verbose_level = 4;
    config.hdr = false;
    
    // Additional configuration to address frequency range issues
    if (fps >= 20) {
        std::cout << "Camera " << camera_id << ": Trying with 15 FPS instead of " << fps << " FPS to address frequency range issue" << std::endl;
        config.mFPS = 15;
    }
    
    // Open the camera
    std::cout << "Camera " << camera_id << ": Opening camera with requested resolution: " 
              << (width > 0 ? std::to_string(width) : "default") 
              << "x" << (height > 0 ? std::to_string(height) : "default")
              << ", requested FPS: " << (fps > 0 ? std::to_string(fps) : "default") << std::endl;
    
    oc::ARGUS_STATE state = camera.openCamera(config);
    if (state != oc::ARGUS_STATE::OK) {
        std::cerr << "Camera " << camera_id << ": Failed to open Camera, error code " << ARGUS_STATE2str(state) << std::endl;
        return;
    }
    
    std::cout << "Camera " << camera_id << ": Camera opened successfully with resolution: " 
              << camera.getWidth() << "x" << camera.getHeight() 
              << ", channels: " << camera.getNumberOfChannels() << std::endl;
    
    // Initialize GStreamer pipeline
    if (!createGstreamerShmsinkPipeline(camera_id, socket_path, stream_name, camera.getWidth(), camera.getHeight())) {
        std::cerr << "Camera " << camera_id << ": Failed to create GStreamer pipeline, exiting..." << std::endl;
        camera.closeCamera();
        return;
    }
    
    cv::Mat frame = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC4, 1);
    int image_count = 0;
    
    // Variable to track the last time a frame was received
    auto last_frame_time = std::chrono::steady_clock::now();
    
    while (true) {
        if (camera.isNewFrame()) {
            // Update the last frame time when a new frame is received
            last_frame_time = std::chrono::steady_clock::now();
            
            memcpy(frame.data, camera.getPixels(),
                   camera.getWidth() * camera.getHeight() * camera.getNumberOfChannels());
            
            // Check if the frame is valid before writing to GStreamer
            if (!frame.empty()) {
                // Write the frame to GStreamer shmsink pipeline
                if (!writeToGstreamerShmsink(camera_id, frame)) {
                    std::cerr << "Camera " << camera_id << ": Failed to write frame to GStreamer pipeline" << std::endl;
                }
            } else {
                std::cerr << "Camera " << camera_id << ": Warning: Skipping empty frame" << std::endl;
            }
            
            std::cout << "Camera " << camera_id << ": Frame Generated: " << image_count << std::endl;
            image_count++;
        } else {
            // Check if it's been more than 10 seconds since the last frame
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_frame_time).count();
            
            if (elapsed_time >= 10) {
                std::cout << "Camera " << camera_id << ": No new frame received for 10 seconds. Exiting camera thread..." << std::endl;
                break;
            }
            
            usleep(100);
        }
    }
    
    camera.closeCamera();
    cleanupGstreamerPipeline(camera_id);
}

int main(int argc, char *argv[]) {
    // Initialize GStreamer early to catch any initialization errors
    GError *gst_error = nullptr;
    if (!gst_init_check(nullptr, nullptr, &gst_error)) {
        std::cerr << "Failed to initialize GStreamer: " << (gst_error ? gst_error->message : "Unknown error") << std::endl;
        if (gst_error) g_error_free(gst_error);
        return -1;
    }
    std::cout << "GStreamer initialized successfully" << std::endl;

    int rq_width = 960;
    int rq_height = 600;
    int rq_fps = 30; // Default to 30 fps instead of 0

    if (argc > 1) rq_width = atoi(argv[1]);
    if (argc > 2) rq_height = atoi(argv[2]);
    if (argc > 3) rq_fps = atoi(argv[3]);
    
    // Ensure FPS is within a valid range (typically 15-60 for most cameras)
    if (rq_fps < 15 || rq_fps > 60) {
        std::cout << "Warning: Requested FPS (" << rq_fps << ") might be out of valid range. Setting to 30 FPS." << std::endl;
        rq_fps = 30;
    }

    try {
        int major, minor, patch;
        oc::ArgusVirtualCapture::getVersion(major, minor, patch);
        std::cout << "Argus Capture Version: " << major << "." << minor << "." << patch << std::endl;

        // Get Argus devices with error handling for invalid camera provider
        std::vector<oc::ArgusDevice> devs;
        bool devices_found = false;
        int retry_count = 0;
        const int max_retries = 2; // Try once, then retry once after restarting service
        
        while (!devices_found && retry_count < max_retries) {
            try {
                devs = oc::ArgusBayerCapture::getArgusDevices();
                if (devs.empty()) {
                    std::cerr << "No Argus devices found!" << std::endl;
                    
                    if (retry_count < max_retries - 1) {
                        std::cout << "Attempting to restart nvargus-daemon service..." << std::endl;
                        // Restart the nvargus-daemon service
                        int restart_result = system("sudo systemctl restart nvargus-daemon");
                        if (restart_result == 0) {
                            std::cout << "nvargus-daemon service restarted successfully" << std::endl;
                            // Wait a moment for the service to fully initialize
                            std::cout << "Waiting for service to initialize..." << std::endl;
                            sleep(5);
                        } else {
                            std::cerr << "Failed to restart nvargus-daemon service (error code: " << restart_result << ")" << std::endl;
                        }
                    } else {
                        std::cerr << "Still no Argus devices found after restarting service" << std::endl;
                        return -1;
                    }
                } else {
                    devices_found = true;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error getting Argus devices: " << e.what() << std::endl;
                
                if (retry_count < max_retries - 1) {
                    std::cout << "Attempting to restart nvargus-daemon service..." << std::endl;
                    int restart_result = system("sudo systemctl restart nvargus-daemon");
                    if (restart_result == 0) {
                        std::cout << "nvargus-daemon service restarted successfully" << std::endl;
                        sleep(5);
                    } else {
                        std::cerr << "Failed to restart nvargus-daemon service (error code: " << restart_result << ")" << std::endl;
                    }
                } else {
                    return -1;
                }
            } catch (...) {
                std::cerr << "Unknown error getting Argus devices. This might be due to an invalid camera provider." << std::endl;
                std::cerr << "Make sure the Argus camera service is running and the camera is properly connected." << std::endl;
                
                if (retry_count < max_retries - 1) {
                    std::cout << "Attempting to restart nvargus-daemon service..." << std::endl;
                    int restart_result = system("sudo systemctl restart nvargus-daemon");
                    if (restart_result == 0) {
                        std::cout << "nvargus-daemon service restarted successfully" << std::endl;
                        sleep(5);
                    } else {
                        std::cerr << "Failed to restart nvargus-daemon service (error code: " << restart_result << ")" << std::endl;
                    }
                } else {
                    return -1;
                }
            }
            
            retry_count++;
        }
        
        if (!devices_found) {
            std::cerr << "Failed to find Argus devices after " << max_retries << " attempts" << std::endl;
            return -1;
        }
        
        // Print available devices
        std::cout << "Found " << devs.size() << " Argus devices:" << std::endl;
        for (int i = 0; i < devs.size(); i++) {
            std::cout << "##################" << std::endl;
            std::cout << " Device : " << devs.at(i).id << std::endl;
            std::cout << " Name : " << devs.at(i).name << std::endl;
            std::cout << " Badge : " << devs.at(i).badge << std::endl;
            std::cout << " Available : " << devs.at(i).available << std::endl;
        }
        std::cout << "***********************" << std::endl;

        // Count available cameras
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
        
        // Limit to maximum 5 cameras (sensor_0 to sensor_4)
        const int max_cameras = 5;
        if (available_camera_ids.size() > max_cameras) {
            std::cout << "Limiting to " << max_cameras << " cameras" << std::endl;
            available_camera_ids.resize(max_cameras);
        }
        
        // Initialize camera_pipelines vector with the number of available cameras
        for (size_t i = 0; i < available_camera_ids.size(); i++) {
            camera_pipelines.push_back(std::make_unique<CameraPipeline>());
        }
        
        // Create threads for each camera
        std::vector<std::thread> camera_threads;
        for (size_t i = 0; i < available_camera_ids.size(); i++) {
            int camera_id = available_camera_ids[i];
            std::cout << "Starting thread for camera " << camera_id << " (index " << i << ")" << std::endl;
            camera_threads.emplace_back(cameraThread, camera_id, rq_width, rq_height, rq_fps);
        }
        
        // Wait for all threads to complete
        for (auto& thread : camera_threads) {
            thread.join();
        }
        
        // Clean up all GStreamer pipelines
        cleanupAllGstreamerPipelines();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during camera initialization: " << e.what() << std::endl;
        cleanupAllGstreamerPipelines();
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception during camera initialization" << std::endl;
        cleanupAllGstreamerPipelines();
        return -1;
    }

    return 0;
}
