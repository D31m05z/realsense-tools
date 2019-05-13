// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API


using namespace std;
using namespace cv;


static void saveCameraParams(const string& filename,
    Size imageSize, 
    float fx, float fy, float cx, float cy, const std::vector<float> & dist_coeffs)
{
    FileStorage fs(filename, FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;


    fs << "fx" << fx;
    fs << "fy" << fy;
    fs << "cx" << cx;
    fs << "cy" << cy;
    Mat dist(1, dist_coeffs.size(), CV_32FC1,(void*) dist_coeffs.data());
    fs << "distortion_coefficients" << dist;
}

int main(int argc, char* argv[]) try
{
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	rs2::config cfg;
	//cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	//cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 0, 1280, 720, RS2_FORMAT_Y8, 0);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 0);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8);
    cfg.disable_stream(RS2_STREAM_DEPTH);
    //cfg.
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8);
	//cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 15);
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8);
	//cfg.enable_record_to_file("a.bag");
	// Start streaming with default recommended configuration
    rs2::pipeline_profile selection = pipe.start(cfg);

    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser


	using namespace cv;
	const std::string window_name = "Display Image";
	namedWindow(window_name, WINDOW_AUTOSIZE);

    auto stream_profile_left = selection.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    auto stream_profile_right = selection.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
    auto stream_profile_color = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    auto intinsics_left = stream_profile_left.get_intrinsics();
    auto intinsics_right = stream_profile_right.get_intrinsics();
    auto intinsics_color = stream_profile_color.get_intrinsics();

    std::vector<float> dist_left(intinsics_left.coeffs, intinsics_left.coeffs + 5);
    saveCameraParams("left_cam_int.yml", Size(intinsics_left.width, intinsics_left.height),
        intinsics_left.fx, intinsics_left.fy, intinsics_left.ppx, intinsics_left.ppy, dist_left);

    std::vector<float> dist_right(intinsics_right.coeffs, intinsics_right.coeffs + 5);
    saveCameraParams("right_cam_int.yml", Size(intinsics_right.width, intinsics_right.height),
        intinsics_right.fx, intinsics_right.fy, intinsics_right.ppx, intinsics_right.ppy, dist_right);

    std::vector<float> dist_color(intinsics_color.coeffs, intinsics_color.coeffs + 5);
    saveCameraParams("color_cam_int.yml", Size(intinsics_color.width, intinsics_color.height),
                     intinsics_color.fx, intinsics_color.fy, intinsics_color.ppx, intinsics_color.ppy, dist_color);

	while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
	{
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

		//rs2::frame color_frame = data.get_color_frame();
		rs2::frame left_frame = data.get_infrared_frame(1);
		rs2::frame right_frame = data.get_infrared_frame(2);
		rs2::frame color_frame = data.get_color_frame();

		// Query frame size (width and height)
		const int w = left_frame.as<rs2::video_frame>().get_width();
		const int h = left_frame.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat left_image_l(Size(w, h), CV_8UC1, (void*)left_frame.get_data(), Mat::AUTO_STEP);
		Mat left_image_r(Size(w, h), CV_8UC1, (void*)right_frame.get_data(), Mat::AUTO_STEP);
		Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        static int fc = 0;
        std::stringstream ss;
        ss << std::setw(5) << std::setfill('0') << fc;
        cv::imwrite("left/" + ss.str() + ".jpg", left_image_l);
        cv::imwrite("right/" + ss.str() + ".jpg", left_image_r);
        cv::imwrite("color/" + ss.str() + ".jpg", color_image);
        ++fc;
		// Update the window with new data
		imshow(window_name, left_image_l);
		//imshow(window_name + " right", left_image_r);
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



