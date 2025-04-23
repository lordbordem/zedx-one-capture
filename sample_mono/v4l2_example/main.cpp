#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Function to write OpenCV Mat to GStreamer shmsink pipeline
bool writeToGstreamerShmsink(cv::Mat &frame, const std::string &socket_path = "/dev/shm/sensor_1",
                             const std::string &stream_name = "camera_stream", int width = 0, int height = 0)
{
    // Check if frame is valid
    if (frame.empty())
    {
        std::cerr << "Error: Cannot write empty frame to GStreamer pipeline" << std::endl;
        return false;
    }
    static GstElement *pipeline = nullptr;
    static GstElement *appsrc = nullptr;
    static int frame_count = 0;
    static int frame_width = 0;
    static int frame_height = 0;

    // If dimensions are not provided, use the frame dimensions
    if (width <= 0)
        width = frame.cols;
    if (height <= 0)
        height = frame.rows;

    // Initialize GStreamer pipeline if not already initialized
    if (pipeline == nullptr)
    {
        // Store frame dimensions
        frame_width = width;
        frame_height = height;

        // Initialize GStreamer
        GError *error = nullptr;
        if (!gst_init_check(nullptr, nullptr, &error))
        {
            std::cerr << "Failed to initialize GStreamer: " << (error ? error->message : "Unknown error") << std::endl;
            if (error)
                g_error_free(error);
            return false;
        }

        // Create pipeline elements
        pipeline = gst_pipeline_new("opencv-to-shmsink");
        appsrc = gst_element_factory_make("appsrc", "source");
        GstElement *videoconvert = gst_element_factory_make("videoconvert", "converter");
        GstElement *shmsink = gst_element_factory_make("shmsink", "sink");

        if (!pipeline || !appsrc || !videoconvert || !shmsink)
        {
            std::cerr << "Failed to create GStreamer elements" << std::endl;
            return false;
        }

        // Configure appsrc
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                            "format", G_TYPE_STRING, "BGR",
                                            "width", G_TYPE_INT, width,
                                            "height", G_TYPE_INT, height,
                                            "framerate", GST_TYPE_FRACTION, 30, 1,
                                            nullptr);

        g_object_set(G_OBJECT(appsrc),
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
        gst_bin_add_many(GST_BIN(pipeline), appsrc, videoconvert, shmsink, nullptr);

        // Link elements
        if (!gst_element_link_many(appsrc, videoconvert, shmsink, nullptr))
        {
            std::cerr << "Failed to link GStreamer elements" << std::endl;
            gst_object_unref(pipeline);
            pipeline = nullptr;
            return false;
        }

        // Start pipeline
        GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE)
        {
            std::cerr << "Failed to start GStreamer pipeline" << std::endl;
            gst_object_unref(pipeline);
            pipeline = nullptr;
            return false;
        }

        std::cout << "GStreamer shmsink pipeline initialized with stream name: " << stream_name
                  << " at socket path: " << socket_path << std::endl;
    }

    // Check if frame dimensions match the pipeline configuration
    if (frame.cols != frame_width || frame.rows != frame_height)
    {
        std::cerr << "Frame dimensions do not match pipeline configuration" << std::endl;
        return false;
    }

    // Create GstBuffer from OpenCV Mat
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, frame.total() * frame.elemSize(), nullptr);
    if (!buffer)
    {
        std::cerr << "Failed to allocate GStreamer buffer" << std::endl;
        return false;
    }

    // Map buffer for writing
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE))
    {
        // Copy frame data to buffer
        memcpy(map.data, frame.data, frame.total() * frame.elemSize());
        gst_buffer_unmap(buffer, &map);

        // Set buffer timestamp and duration
        GST_BUFFER_PTS(buffer) = frame_count * 33333333; // 30fps (in nanoseconds)
        GST_BUFFER_DURATION(buffer) = 33333333;

        // Push buffer to appsrc
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK)
        {
            std::cerr << "Failed to push buffer to GStreamer pipeline" << std::endl;
            return false;
        }

        frame_count++;
        return true;
    }
    else
    {
        std::cerr << "Failed to map GStreamer buffer" << std::endl;
        gst_buffer_unref(buffer);
        return false;
    }
}

// Function to clean up GStreamer resources
void cleanupGstreamerPipeline()
{
    static GstElement *pipeline = nullptr;

    if (pipeline)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
        pipeline = nullptr;
        std::cout << "GStreamer pipeline cleaned up" << std::endl;
    }
}

// Function to display usage information
void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [camera_id] [width] [height] [fps] [blue_scale] [green_scale] [red_scale]" << std::endl;
    std::cout << "  camera_id   : Camera device ID (default: 0)" << std::endl;
    std::cout << "  width       : Requested width (default: use camera default)" << std::endl;
    std::cout << "  height      : Requested height (default: use camera default)" << std::endl;
    std::cout << "  fps         : Requested FPS (default: 30)" << std::endl;
    std::cout << "  blue_scale  : Blue channel scaling factor (default: 1.3)" << std::endl;
    std::cout << "  green_scale : Green channel scaling factor (default: 0.75)" << std::endl;
    std::cout << "  red_scale   : Red channel scaling factor (default: 0.85)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example to fix yellow tint: " << programName << " 0 0 0 30 1.3 0.75 0.85" << std::endl;
    std::cout << "Example to fix blue tint  : " << programName << " 0 0 0 30 0.9 1.0 1.1" << std::endl;
}

int main(int argc, char *argv[])
{
    // Check if help was requested
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printUsage(argv[0]);
        return 0;
    }

    // Initialize GStreamer early to catch any initialization errors
    GError *gst_error = nullptr;
    if (!gst_init_check(nullptr, nullptr, &gst_error))
    {
        std::cerr << "Failed to initialize GStreamer: " << (gst_error ? gst_error->message : "Unknown error") << std::endl;
        if (gst_error)
            g_error_free(gst_error);
        return -1;
    }
    std::cout << "GStreamer initialized successfully" << std::endl;

    int camera_id_0 = 0;
    int rq_width = 0;
    int rq_height = 0;
    int rq_fps = 30; // Default to 30 fps instead of 0
    
    // Default color correction factors
    double blue_scale = 1.3;   // Boost blue by 30%
    double green_scale = 0.75; // Reduce green by 25%
    double red_scale = 0.85;   // Reduce red by 15%

    if (argc > 1)
        camera_id_0 = atoi(argv[1]);
    if (argc > 2)
        rq_width = atoi(argv[2]);
    if (argc > 3)
        rq_height = atoi(argv[3]);
    if (argc > 4)
        rq_fps = atoi(argv[4]);
    if (argc > 5)
        blue_scale = atof(argv[5]);
    if (argc > 6)
        green_scale = atof(argv[6]);
    if (argc > 7)
        red_scale = atof(argv[7]);
        
    std::cout << "Using color correction factors - Blue: " << blue_scale 
              << ", Green: " << green_scale 
              << ", Red: " << red_scale << std::endl;

    // Ensure FPS is within a valid range (typically 15-60 for most cameras)
    if (rq_fps < 15 || rq_fps > 60)
    {
        std::cout << "Warning: Requested FPS (" << rq_fps << ") might be out of valid range. Setting to 30 FPS." << std::endl;
        rq_fps = 30;
    }

    oc::ArgusV4l2Capture camera_0;
    oc::ArgusCameraConfig config;

    try
    {
        int major, minor, patch;
        oc::ArgusVirtualCapture::getVersion(major, minor, patch);
        std::cout << " Argus Capture Version : " << major << "." << minor << "." << patch << std::endl;

        // Get Argus devices with error handling for invalid camera provider
        std::vector<oc::ArgusDevice> devs;
        bool devices_found = false;
        int retry_count = 0;
        const int max_retries = 2; // Try once, then retry once after restarting service

        while (!devices_found && retry_count < max_retries)
        {
            try
            {
                devs = oc::ArgusV4l2Capture::getV4l2Devices();
                if (devs.empty())
                {
                    std::cerr << "No V4L2 devices found!" << std::endl;

                    if (retry_count < max_retries - 1)
                    {
                        std::cout << "Attempting to restart nvargus-daemon service..." << std::endl;
                        // Restart the nvargus-daemon service
                        int restart_result = system("sudo systemctl restart nvargus-daemon");
                        if (restart_result == 0)
                        {
                            std::cout << "nvargus-daemon service restarted successfully" << std::endl;
                            // Wait a moment for the service to fully initialize
                            std::cout << "Waiting for service to initialize..." << std::endl;
                            sleep(5);
                        }
                        else
                        {
                            std::cerr << "Failed to restart nvargus-daemon service (error code: " << restart_result << ")" << std::endl;
                        }
                    }
                    else
                    {
                        std::cerr << "Still no V4L2 devices found after restarting service" << std::endl;
                        return -1;
                    }
                }
                else
                {
                    devices_found = true;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error getting V4L2 devices: " << e.what() << std::endl;

                if (retry_count < max_retries - 1)
                {
                    std::cout << "Attempting to restart nvargus-daemon service..." << std::endl;
                    int restart_result = system("sudo systemctl restart nvargus-daemon");
                    if (restart_result == 0)
                    {
                        std::cout << "nvargus-daemon service restarted successfully" << std::endl;
                        sleep(5);
                    }
                    else
                    {
                        std::cerr << "Failed to restart nvargus-daemon service (error code: " << restart_result << ")" << std::endl;
                    }
                }
                else
                {
                    return -1;
                }
            }
            catch (...)
            {
                std::cerr << "Unknown error getting V4L2 devices. This might be due to an invalid camera provider." << std::endl;
                std::cerr << "Make sure the Argus camera service is running and the camera is properly connected." << std::endl;

                if (retry_count < max_retries - 1)
                {
                    std::cout << "Attempting to restart nvargus-daemon service..." << std::endl;
                    int restart_result = system("sudo systemctl restart nvargus-daemon");
                    if (restart_result == 0)
                    {
                        std::cout << "nvargus-daemon service restarted successfully" << std::endl;
                        sleep(5);
                    }
                    else
                    {
                        std::cerr << "Failed to restart nvargus-daemon service (error code: " << restart_result << ")" << std::endl;
                    }
                }
                else
                {
                    return -1;
                }
            }

            retry_count++;
        }

        if (!devices_found)
        {
            std::cerr << "Failed to find V4L2 devices after " << max_retries << " attempts" << std::endl;
            return -1;
        }

        for (int i = 0; i < devs.size(); i++)
        {
            std::cout << "##################" << std::endl;
            std::cout << " Device : " << devs.at(i).id << std::endl;
            std::cout << " Name : " << devs.at(i).name << std::endl;
            std::cout << " Badge : " << devs.at(i).badge << std::endl;
            std::cout << " Available : " << devs.at(i).available << std::endl;
        }
        std::cout << "***********************" << std::endl;

        // Check if the requested camera is available
        bool camera_available = false;
        for (const auto &dev : devs)
        {
            if (dev.id == camera_id_0 && dev.available)
            {
                camera_available = true;
                break;
            }
        }

        if (!camera_available)
        {
            std::cerr << "Camera ID " << camera_id_0 << " is not available!" << std::endl;
            return -1;
        }

        /// Create configuration for the camera
        config.mDeviceId = (camera_id_0);
        config.mFPS = rq_fps;
        config.mWidth = rq_width;
        config.mHeight = rq_height;
        config.verbose_level = 4;
        config.mode = oc::PixelMode::RAW10;

        // Additional configuration to address frequency range issues
        // These settings might need to be adjusted based on the specific camera model
        // Try different FPS values that might be supported by the camera
        if (rq_fps >= 20)
        {
            std::cout << "Trying with 15 FPS instead of " << rq_fps << " FPS to address frequency range issue" << std::endl;
            config.mFPS = 15;
        }

        /// Open the camera
        std::cout << "Opening camera with ID: " << camera_id_0
                  << ", requested resolution: " << (rq_width > 0 ? std::to_string(rq_width) : "default")
                  << "x" << (rq_height > 0 ? std::to_string(rq_height) : "default")
                  << ", requested FPS: " << (rq_fps > 0 ? std::to_string(rq_fps) : "default") << std::endl;

        oc::ARGUS_STATE state_cam0 = camera_0.openCamera(config);
        if (state_cam0 != oc::ARGUS_STATE::OK)
        {
            std::cerr << "Failed to open Camera, error code " << ARGUS_STATE2str(state_cam0) << std::endl;
            return -1;
        }

        std::cout << "Camera opened successfully with resolution: "
                  << camera_0.getWidth() << "x" << camera_0.getHeight()
                  << ", channels: " << camera_0.getNumberOfChannels() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception during camera initialization: " << e.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cerr << "Unknown exception during camera initialization" << std::endl;
        return -1;
    }

    // ensure socket-path is not link, if it is then unlink (remove) it
    // This is required as gstreamer will not do this, and will instead create
    // the socket_path with '.1' suffix to the socket_path
    if (access("/dev/shm/sensor_1", F_OK) != -1)
    {
        std::cerr << "Socket path already exists, unlinking " << "/dev/shm/sensor_1" << std::endl;
        ;
        if (unlink("/dev/shm/sensor_1") != 0)
        {
            std::cerr << "Unlinking failed, manually deleting the path " << strerror(errno) << std::endl;
            if (rmdir("/dev/shm/sensor_1") != 0)
            {
                // TODO: At this point, should it just exit and fail?
                std::cerr << "Remove Dir failed " << strerror(errno) << std::endl;
            }
        };
    }

    cv::Mat rgb_d, bayer_frame;
    bayer_frame = cv::Mat(camera_0.getHeight(), camera_0.getWidth(), CV_16UC1, 1);

    int image_count = 0;

    // Variable to track the last time a frame was received
    auto last_frame_time = std::chrono::steady_clock::now();

    // Variable to track the last time a frame was saved to a file
    auto last_save_time = std::chrono::steady_clock::now();

    // Create a directory for saved frames if it doesn't exist
    const std::string save_dir = "saved_frames";
    struct stat st;
    if (stat(save_dir.c_str(), &st) != 0)
    {
        // Directory doesn't exist, create it
        if (mkdir(save_dir.c_str(), 0755) == 0)
        {
            std::cout << "Created directory for saved frames: " << save_dir << std::endl;
        }
        else
        {
            std::cerr << "Failed to create directory for saved frames: " << strerror(errno) << std::endl;
        }
    }

    while (true)
    {
        if (camera_0.isNewFrame())
        {
            // Update the last frame time when a new frame is received
            last_frame_time = std::chrono::steady_clock::now();
   
            // Copy the raw pixel data
            memcpy(bayer_frame.data, camera_0.getPixels(),
                   camera_0.getWidth() * camera_0.getHeight() *
                       camera_0.getNumberOfChannels() * camera_0.getPixelDepth());
            
            cv::Mat rgb_frame;
            
            cv::cvtColor(bayer_frame, rgb_frame, cv::COLOR_BayerGB2BGR); // RGGB pattern

            // Check if the frame is valid before writing to GStreamer
            if (!rgb_frame.empty())
            {
                // Write the converted RGB frame to GStreamer shmsink pipeline
                if (!writeToGstreamerShmsink(rgb_frame, "/dev/shm/sensor_1", "argus_camera_stream"))
                {
                    std::cerr << "Failed to write frame to GStreamer pipeline" << std::endl;
                }
            }
            else
            {
                std::cerr << "Warning: Skipping empty frame" << std::endl;
            }

            std::cout << "Frame Generated: " << image_count << std::endl;
            image_count++;

            // Check if a minute has passed since the last save
            auto current_time = std::chrono::steady_clock::now();
            auto time_since_last_save = std::chrono::duration_cast<std::chrono::seconds>(
                                            current_time - last_save_time)
                                            .count();

            // if (time_since_last_save >= 10)
            // { // 60 seconds = 1 minute
            //     // Get current time for filename
            //     auto now = std::chrono::system_clock::now();
            //     auto now_time_t = std::chrono::system_clock::to_time_t(now);
            //     std::stringstream ss;
            //     ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
            //     std::string timestamp = ss.str();

            //     // Create filename with timestamp
            //     std::string filename = save_dir + "/frame_" + timestamp + ".png";

            //     // Save the frame to a file
            //     cv::imwrite(filename, rgb_frame);
            //     std::cout << "Frame saved to file: " << filename << std::endl;

            //     // Update the last save time
            //     last_save_time = current_time;
            // }

        }
        else
        {
            // Check if it's been more than 10 seconds since the last frame
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_frame_time).count();

            if (elapsed_time >= 10)
            {
                std::cout << "No new frame received for 10 seconds. Rebooting camera..." << std::endl;
                oc::ARGUS_STATE reboot_state = camera_0.reboot();
                std::cout << "Camera reboot result: " << ARGUS_STATE2str(reboot_state) << std::endl;

                // Reset the timer after attempting to reboot
                last_frame_time = std::chrono::steady_clock::now();
            }

            usleep(100);
        }
    }
    camera_0.closeCamera();

    // Clean up GStreamer resources
    cleanupGstreamerPipeline();

    return 0;
}
