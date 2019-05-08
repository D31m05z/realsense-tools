// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

int main(int argc, char * argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    //cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 15);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);
	cfg.enable_device_from_file("a.bag", false);
	
    // Start streaming with default recommended configuration
    pipe.start(cfg);


    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::spatial_filter spat_filter(0.5f, 20.0f, 2.0f, 1.0f);    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);


    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    
    auto intrinsics = rs2::video_stream_profile(pipe.get_active_profile().get_stream(RS2_STREAM_COLOR)).get_intrinsics();;
    auto intrinsics_dep = rs2::video_stream_profile(pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH)).get_intrinsics();;
    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
		rs2::frameset data;
		try 
		{
			data = pipe.wait_for_frames(1000); // Wait for next set of frames from the camera
		}
		catch (rs2::error e)
		{
			std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
			break;
		}
		
        data = align_to_color.process(data);
        
        rs2::depth_frame depth = data.get_depth_frame().as<rs2::depth_frame>();
        //depth = depth_to_disparity.process(depth);
        //depth = spat_filter.process(depth);
        //depth = temp_filter.process(depth);
        //depth = disparity_to_depth.process(depth);
        rs2::frame color_frame = data.get_color_frame();
        rs2::frame color_depth = depth.apply_filter(color_map);
        std::cout << depth.get_distance(30, 30) << std::endl;

        // Query frame size (width and height)
        const int w = color_depth.as<rs2::video_frame>().get_width();
        const int h = color_depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)color_depth.get_data(), Mat::AUTO_STEP);
        Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(window_name, image);
        imshow("color", color_image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



