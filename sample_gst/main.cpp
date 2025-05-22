#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <signal.h>
#include <csignal>
#include <algorithm>
#include <deque>
#include "ArgusCapture.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Global flag to indicate when the program should terminate
static std::atomic<bool> g_terminate(false);

// Vector to store camera objects for cleanup
static std::vector<oc::ArgusBayerCapture*> g_cameras;
static std::mutex g_cameras_mutex;

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

// Structure to hold temporary frame data
struct FrameBuffer {
    std::vector<unsigned char> data;
    int width;
    int height;
    int channels;
};

// Deque to store temporary frames for each camera
static std::vector<std::deque<FrameBuffer>> g_temp_frames;

// Global vector to store camera pipelines
static std::vector<std::unique_ptr<CameraPipeline>> camera_pipelines;

// Signal handler for graceful termination
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nReceived termination signal. Shutting down gracefully..." << std::endl;
        g_terminate.store(true);
    }
}

// Function to create GStreamer shmsink pipeline
bool createGstreamerShmsinkPipeline(int camera_index, const std::string &socket_path, 
                                   const std::string &stream_name, 
                                   int width = 0, int height = 0, 
                                   int channels = 4) {
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
    pipeline_data->pipeline = gst_pipeline_new(("argus-to-shmsink-" + std::to_string(camera_index)).c_str());
    pipeline_data->appsrc = gst_element_factory_make("appsrc", ("source-" + std::to_string(camera_index)).c_str());
    GstElement *videoconvert = gst_element_factory_make("videoconvert", ("converter-" + std::to_string(camera_index)).c_str());
    GstElement *shmsink = gst_element_factory_make("shmsink", ("sink-" + std::to_string(camera_index)).c_str());
    
    if (!pipeline_data->pipeline || !pipeline_data->appsrc || !videoconvert || !shmsink) {
        std::cerr << "Failed to create GStreamer elements for camera " << camera_index << std::endl;
        return false;
    }
    
    // Configure appsrc with the appropriate format based on the number of channels
    const char* format_str = (channels == 4) ? "BGRA" : (channels == 3) ? "RGB" : "GRAY8";
    
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                       "format", G_TYPE_STRING, format_str,
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
                "shm-size", 100 * 2024 * 2024, // 100MB buffer
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

// Function to write raw pixel data to GStreamer shmsink pipeline
bool writeToGstreamerShmsink(int camera_index, const unsigned char* pixel_data, int width, int height, int channels) {
    // Ensure camera_index is valid
    if (camera_index >= camera_pipelines.size()) {
        std::cerr << "Invalid camera index: " << camera_index << std::endl;
        return false;
    }

    // Get reference to the camera pipeline
    CameraPipeline *pipeline_data = camera_pipelines[camera_index].get();
    std::lock_guard<std::mutex> lock(pipeline_data->mutex);
    
    // Check if pixel data is valid
    if (pixel_data == nullptr) {
        std::cerr << "Error: Cannot write null pixel data to GStreamer pipeline for camera " << camera_index << std::endl;
        return false;
    }
    
    // Initialize GStreamer pipeline if not already initialized
    if (pipeline_data->pipeline == nullptr) {
        std::cerr << "Pipeline not initialized for camera " << camera_index << std::endl;
        return false;
    }
    
    // Check if frame dimensions match the pipeline configuration
    if (width != pipeline_data->frame_width || height != pipeline_data->frame_height) {
        std::cerr << "Frame dimensions do not match pipeline configuration for camera " << camera_index << std::endl;
        return false;
    }
    
    // Calculate buffer size based on dimensions and channels
    size_t buffer_size = width * height * channels;
    
    // Create GstBuffer from pixel data
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, buffer_size, nullptr);
    if (!buffer) {
        std::cerr << "Failed to allocate GStreamer buffer for camera " << camera_index << std::endl;
        return false;
    }
    
    // Map buffer for writing
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        // Copy pixel data to buffer
        memcpy(map.data, pixel_data, buffer_size);
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

// Function to clean up all camera resources
void cleanupAllCameras() {
    std::lock_guard<std::mutex> lock(g_cameras_mutex);
    for (auto camera : g_cameras) {
        if (camera) {
            camera->closeCamera();
            std::cout << "Camera closed" << std::endl;
        }
    }
    g_cameras.clear();
}

// Function to clean up all resources
void cleanupAllResources() {
    cleanupAllGstreamerPipelines();
    cleanupAllCameras();
}

// Thread function to handle a single camera
void cameraThread(int camera_id, int rq_width, int rq_height, int rq_fps) {
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
    oc::ArgusBayerCapture* camera = new oc::ArgusBayerCapture();
    
    // Register camera for cleanup
    {
        std::lock_guard<std::mutex> lock(g_cameras_mutex);
        g_cameras.push_back(camera);
    }
    
    // Check if we need to run the camera with default settings first
    // (This is needed for 960x600 to work properly)
    bool needs_two_stage_init = (rq_width == 960 && rq_height == 600);
    
    if (needs_two_stage_init) {
        std::cout << "Camera " << camera_id << ": Using two-stage initialization for 960x600@15FPS" << std::endl;
        
        // First stage: Run with default settings (0, 0, 0)
        oc::ArgusCameraConfig default_config;
        default_config.mDeviceId = camera_id;
        default_config.mFPS = 0;  // default
        default_config.mWidth = 0;  // default
        default_config.mHeight = 0;  // default
        default_config.verbose_level = 3;
        default_config.hdr = false;
        
        std::cout << "Camera " << camera_id << ": Stage 1 - Opening camera with default settings" << std::endl;
        
        oc::ARGUS_STATE state = camera->openCamera(default_config);
        if (state != oc::ARGUS_STATE::OK) {
            std::cerr << "Camera " << camera_id << ": Failed to open Camera with default settings, error code " 
                      << ARGUS_STATE2str(state) << std::endl;
            
            // Remove camera from cleanup list
            {
                std::lock_guard<std::mutex> lock(g_cameras_mutex);
                auto it = std::find(g_cameras.begin(), g_cameras.end(), camera);
                if (it != g_cameras.end()) {
                    g_cameras.erase(it);
                }
            }
            
            delete camera;
            return;
        }
        
        std::cout << "Camera " << camera_id << ": Camera opened with default settings. Resolution: " 
                  << camera->getWidth() << "x" << camera->getHeight() 
                  << ", channels: " << camera->getNumberOfChannels() << std::endl;
        
        // Run for 10 frames with default settings
        std::cout << "Camera " << camera_id << ": Running 10 frames with default settings..." << std::endl;
        int frame_count = 0;
        while (frame_count < 10 && !g_terminate.load()) {
            if (camera->isNewFrame()) {
                frame_count++;
                std::cout << "Camera " << camera_id << ": Default settings frame " << frame_count << "/10" << std::endl;
            } else {
                usleep(100);
            }
        }
        
        // Close the camera after running with default settings
        std::cout << "Camera " << camera_id << ": Closing camera after default settings run" << std::endl;
        camera->closeCamera();
        
        // Small delay before reopening
        usleep(500000);  // 500ms delay
    }
    
    // Create configuration for the camera with requested settings
    oc::ArgusCameraConfig config;
    config.mDeviceId = camera_id;
    config.mFPS = rq_fps;
    config.mWidth = rq_width;
    config.mHeight = rq_height;
    config.verbose_level = 3;
    config.hdr = false;

    // Open the camera with requested settings
    std::cout << "Camera " << camera_id << ": Opening camera with requested resolution: " 
              << (rq_width > 0 ? std::to_string(rq_width) : "default") 
              << "x" << (rq_height > 0 ? std::to_string(rq_height) : "default")
              << ", requested FPS: " << (rq_fps > 0 ? std::to_string(rq_fps) : "default") << std::endl;
    
    oc::ARGUS_STATE state = camera->openCamera(config);
    if (state != oc::ARGUS_STATE::OK) {
        std::cerr << "Camera " << camera_id << ": Failed to open Camera, error code " << ARGUS_STATE2str(state) << std::endl;
        
        // Remove camera from cleanup list
        {
            std::lock_guard<std::mutex> lock(g_cameras_mutex);
            auto it = std::find(g_cameras.begin(), g_cameras.end(), camera);
            if (it != g_cameras.end()) {
                g_cameras.erase(it);
            }
        }
        
        delete camera;
        return;
    }

    std::cout << "Camera " << camera_id << ": Camera opened successfully with resolution: " 
              << camera->getWidth() << "x" << camera->getHeight() 
              << ", channels: " << camera->getNumberOfChannels() << std::endl;
    
    // Ensure the temporary frames vector for this camera is initialized
    if (camera_id >= g_temp_frames.size()) {
        g_temp_frames.resize(camera_id + 1);
    }
    
    // Clear any existing frames
    g_temp_frames[camera_id].clear();

    printf("requested gstreamer width %d", camera->getWidth());
    printf("requested gstreamer height %d", camera->getHeight());
    
    // Now initialize GStreamer pipeline
    if (!createGstreamerShmsinkPipeline(camera_id, socket_path, stream_name, 
                                       camera->getWidth(), camera->getHeight(), 
                                       camera->getNumberOfChannels())) {
        std::cerr << "Camera " << camera_id << ": Failed to create GStreamer pipeline, exiting..." << std::endl;
        camera->closeCamera();
        
        // Remove camera from cleanup list
        {
            std::lock_guard<std::mutex> lock(g_cameras_mutex);
            auto it = std::find(g_cameras.begin(), g_cameras.end(), camera);
            if (it != g_cameras.end()) {
                g_cameras.erase(it);
            }
        }
        
        delete camera;
        return;
    }
    
    // No temporary frames to process
    
    int image_count = 0;
    
    // Variable to track the last time a frame was received
    auto last_frame_time = std::chrono::steady_clock::now();
    
    while (!g_terminate.load()) {
        if (camera->isNewFrame()) {
            // Update the last frame time when a new frame is received
            last_frame_time = std::chrono::steady_clock::now();
            
            // Get the raw pixel data directly from the camera
            unsigned char* pixel_data = camera->getPixels();
            
            // Check if pixel data is valid before writing to GStreamer
            if (pixel_data != nullptr) {
                // Write the pixel data directly to GStreamer shmsink pipeline
                if (!writeToGstreamerShmsink(camera_id, pixel_data, 
                                           camera->getWidth(), camera->getHeight(), 
                                           camera->getNumberOfChannels())) {
                    std::cerr << "Camera " << camera_id << ": Failed to write frame to GStreamer pipeline" << std::endl;
                }
            } else {
                std::cerr << "Camera " << camera_id << ": Warning: Skipping null pixel data" << std::endl;
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
    
    std::cout << "Camera " << camera_id << ": Thread exiting, cleaning up resources..." << std::endl;
    
    // Clean up resources
    cleanupGstreamerPipeline(camera_id);
    camera->closeCamera();
    
    // Remove camera from cleanup list
    {
        std::lock_guard<std::mutex> lock(g_cameras_mutex);
        auto it = std::find(g_cameras.begin(), g_cameras.end(), camera);
        if (it != g_cameras.end()) {
            g_cameras.erase(it);
        }
    }
    
    delete camera;
    std::cout << "Camera " << camera_id << ": Resources cleaned up" << std::endl;
}

int main(int argc, char *argv[]) {
    // Set up signal handler for graceful termination
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

int rq_width = 960;
int rq_height = 600;
int rq_fps = 15;

if (argc > 1) rq_width = atoi(argv[1]);
if (argc > 2) rq_height = atoi(argv[2]);
if (argc > 3) rq_fps = atoi(argv[3]);

    try {
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
        
        std::cout << "All camera threads started. Press Ctrl+C to exit." << std::endl;
        
        // Wait for all threads to complete
        for (auto& thread : camera_threads) {
            thread.join();
        }
        
        std::cout << "All camera threads have completed." << std::endl;
        
        // Clean up all resources
        cleanupAllResources();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception during camera initialization: " << e.what() << std::endl;
        cleanupAllResources();
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception during camera initialization" << std::endl;
        cleanupAllResources();
        return -1;
    }

    std::cout << "Program exiting cleanly" << std::endl;
    return 0;
}
