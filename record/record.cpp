// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

int main(int argc, char* argv[]) try
{
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
	cfg.enable_record_to_file("a.bag");
	// Start streaming with default recommended configuration
	pipe.start(cfg);


	using namespace cv;
	const auto window_name = "Display Image";
	namedWindow(window_name, WINDOW_AUTOSIZE);

	while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
	{
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

		rs2::frame color_frame = data.get_color_frame();

		// Query frame size (width and height)
		const int w = color_frame.as<rs2::video_frame>().get_width();
		const int h = color_frame.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		// Update the window with new data
		imshow(window_name, color_image);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}



