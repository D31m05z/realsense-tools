// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <System.h>



void exportToFileYML(std::ofstream & filestream, 
	const std::vector<std::pair<Eigen::Vector2i, Eigen::Vector3f>> & coordss,
	const cv::Mat & color,
	const Eigen::Matrix4f & T_world_cam) 
{
	// MARK: Header writing
	filestream << "ply" << std::endl <<
		"format " << "ascii" << " 1.0" << std::endl <<
		"comment file created using code by Cedric Menard" << std::endl <<
		"element vertex " << coordss.size() << std::endl <<
		"property float x" << std::endl <<
		"property float y" << std::endl <<
		"property float z" << std::endl <<
		"property uchar red" << std::endl <<
		"property uchar green" << std::endl <<
		"property uchar blue" << std::endl <<
		"end_header" << std::endl;

	// MARK: Data writing

	// Pointer to data

	// Output format switch
	for (size_t j  = 0; j < coordss.size(); ++j)
	{                            // Loop through all elements
		auto& e = coordss[j];
		{
			cv::Vec3b c = color.at<cv::Vec3b>(e.first.y(), e.first.x());
			Eigen::Vector4f P = T_world_cam * e.second.homogeneous();
			for (unsigned int k = 0; k < 3; ++k) 
			{                                // Loop through 3 coordinates
				filestream << std::setprecision(9) << P[k] << " ";
			}
			for (int k = 2; k >= 0; k--) 
			{
				// OpenCV uses BGR format, so the order of writing is reverse to comply with the RGB format
				filestream << (unsigned short)c[k] << (k == 0 ? "" : " ");                     // Loop through RGB
			}
			filestream << std::endl;                                            // End if element line
		}
	}
}




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
	std::string rec_file = "i:/a.bag";
	cfg.enable_device_from_file(rec_file, false);
	
    // Start streaming with default recommended configuration
    auto pipeline_prof = pipe.start(cfg);


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
    auto intrinsics_dep = rs2::video_stream_profile(pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH)).get_intrinsics();
    //auto intrinsics_dep = rs2::video_stream_profile(pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH)).get_extrinsics_to(;

	ORB_SLAM2::System::Params p;
	p.tracking_params.fps = 15;
	p.tracking_params.fx = intrinsics.fx;
	p.tracking_params.fy = intrinsics.fy;
	p.tracking_params.cx = intrinsics.ppx;
	p.tracking_params.cy = intrinsics.ppy;
	p.tracking_params.init_thrs_fast = 20.0f;
	p.tracking_params.min_thrs_fast = 7.0f;
	p.tracking_params.k1 = intrinsics.coeffs[0];
	p.tracking_params.k2 = intrinsics.coeffs[1];
	p.tracking_params.p1 = intrinsics.coeffs[2];
	p.tracking_params.p2 = intrinsics.coeffs[3];
	p.tracking_params.k3 = intrinsics.coeffs[4];
	p.tracking_params.bf = p.tracking_params.fx * 0.1f;
	p.tracking_params.depth_map_factor = 1.0f;
	p.tracking_params.num_features = 2000;
	p.tracking_params.num_levels = 8;
	p.tracking_params.scale_factor = 1.2f;
	p.tracking_params.th_depth = 20.0f;
	p.voc_file_path = "d:/tmp/orb_vocabulary.bin";
	p.viewer_params.fps = p.tracking_params.fps;
	p.viewer_params.width = intrinsics.width;
	p.viewer_params.height = intrinsics.height;
	ORB_SLAM2::System system(p, ORB_SLAM2::System::RGBD, true);
	cv::Mat depth_map(intrinsics.height, intrinsics.width, CV_32FC1);
	auto sensor = pipeline_prof.get_device().first<rs2::depth_sensor>();
	auto depth_scale = sensor.get_depth_scale();

	int width = intrinsics.width;
	int height = intrinsics.height;
	size_t frame_num = 0;
	std::vector<cv::Mat> imgs;
	imgs.reserve(1000);
	std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector3f>>> coords;
	coords.reserve(1000);
	size_t frame_step = 30;
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
		static int fc = 0;
		++fc;
		if (fc < 50)
			continue;


        data = align_to_color.process(data);
        
        rs2::depth_frame depth = data.get_depth_frame().as<rs2::depth_frame>();
        depth = depth_to_disparity.process(depth);
        depth = spat_filter.process(depth);
        depth = temp_filter.process(depth);
        depth = disparity_to_depth.process(depth);
        rs2::frame color_frame = data.get_color_frame();
        //rs2::frame color_depth = depth.apply_filter(color_map);
		cv::Mat cv_depth(depth.get_height(), depth.get_width(), CV_16SC1, (void*)depth.get_data());
		cv_depth.convertTo(depth_map, CV_32FC1, depth_scale);



        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(width, height), CV_8UC3, (void*)color_depth.get_data(), Mat::AUTO_STEP);
        Mat color_image(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		//cv::Mat 

		system.TrackRGBD(color_image, depth_map, color_frame.get_timestamp());

		if (frame_num % frame_step == 0)
		{
			imgs.push_back(color_image.clone());
			std::vector<std::pair<Eigen::Vector2i,Eigen::Vector3f>> pts;
			pts.reserve(width* height);
			for (size_t j = 0; j < height; ++j)
			{
				for (size_t i = 0; i < width; ++i)
				{
					float z = depth.get_distance(i, j);
					if (z < 2.0f && z > 0.1f)
					{

						pts.push_back({ Eigen::Vector2i(i, j) , Eigen::Vector3f(
							z * (i - intrinsics.ppx) / intrinsics.fx,
							z * (j - intrinsics.ppy) / intrinsics.fy,
							z
						) });
					}
				}
			}
			coords.emplace_back(std::move(pts));
		}

		++frame_num;
        // Update the window with new data
        imshow(window_name, color_image);
    }

	system.Shutdown();
	system.SaveTrajectoryKITTI("alma.txt");
	std::vector<bool> is_keyframes;
	auto T_cam_world = system.getFramePoses(is_keyframes);
	std::cout << T_cam_world.size() << "  " << frame_num << std::endl;

	for (size_t i = 0; i < T_cam_world.size(); ++i)
	{
		if (i % frame_step == 0)
		{
			size_t idx = i / frame_step;
			std::stringstream ss;
			ss << std::setw(5) << std::setfill('0') << idx;
			cv::imwrite("imgs/" + ss.str() + ".jpg", imgs[idx]);
			std::ofstream yml("ymls/" + ss.str() + ".ply");
			exportToFileYML(yml, coords[idx], imgs[idx], T_cam_world[i].inverse());
		}
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



