#include <string.h>
#include <fstream>
#include <string>

//#include <opencv2/rgbd.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_utils.h>
#include <pcl/io/ply_io.h>
#include <util_pcl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

bool save_flag (false);
bool show_flag (false);
std::string input_file_name;
std::string output_file_name;

void showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " file.dpt [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -w:				       Save compute point normal" << std::endl;
	std::cout << "     -s:				       display the cloud" << std::endl;
	std::cout << "     --i val:         	   input file" << std::endl;
	std::cout << "     --o val:         	   output file name" << std::endl;
}

void parseCommandLine(int argc, char *argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}
	if (pcl::console::find_switch(argc, argv, "-s"))
	{
		show_flag = true;
	}
	if (pcl::console::find_switch(argc, argv, "-w"))
	{
		save_flag = true;
	}
	pcl::console::parse_argument(argc, argv, "--i", input_file_name);
	pcl::console::parse_argument(argc, argv, "--o", output_file_name);
	if (input_file_name.empty())
	{
		std::cout << "An file must be input!" << std::endl;
		exit(0);
	}
	if (output_file_name.empty())
	{
		output_file_name = input_file_name + ".ply";
	}
}

IplImage * loadDepth(std::string a_name)
{
	std::ifstream l_file(a_name.c_str(), std::ofstream::in | std::ofstream::binary);

	if (l_file.fail() == true)
	{
		printf("cv_load_depth: could not open file for writing!\n");
		return NULL;
	}
	int l_row;
	int l_col;

	l_file.read((char*)&l_row, sizeof(l_row));
	l_file.read((char*)&l_col, sizeof(l_col));

	IplImage * lp_image = cvCreateImage(cvSize(l_col, l_row), IPL_DEPTH_16U, 1);

	for (int l_r = 0; l_r < l_row; ++l_r)
	{
		for (int l_c = 0; l_c < l_col; ++l_c)
		{
			l_file.read((char*)&CV_IMAGE_ELEM(lp_image, unsigned short, l_r, l_c), sizeof(unsigned short));
		}
	}
	l_file.close();

	return lp_image;
}

void cv_mat_cloud_to_pcl_cloud(cv::Mat pc, pcl::PointCloud<PointType>::Ptr pt)
{
	for (int i = 0; i < pc.rows; i++)
	{
		for (int j = 0; j < pc.cols; ++j)
		{
			PointType _tem;
			_tem.x = pc.at<cv::Vec3f>(i, j)[0];
			_tem.y = pc.at<cv::Vec3f>(i, j)[1];
			_tem.z = pc.at<cv::Vec3f>(i, j)[2];
			pt->push_back(_tem);
		}

	}
}

void cv_depth_2_pcl_cloud(std::string depth_file_name, cv::Mat intrinsicK, pcl::PointCloud<PointType>::Ptr out)
{
	IplImage *depth_in = loadDepth(depth_file_name);
	cv::Mat depth = cv::cvarrToMat(depth_in);
	// if not change type cv::depthTo2d will include a scale
	if (depth.depth() != CV_32F)
	{
		depth.convertTo(depth, CV_32F);
	}
	cv::Mat pc;
    cv::depthTo3d(depth, intrinsicK, pc);

	cv_mat_cloud_to_pcl_cloud(pc, out);

}


using namespace std;
int main(int argc, char**argv)
{
	parseCommandLine(argc, argv);
	if (show_flag)
	{
		IplImage* im = loadDepth(input_file_name);
		cv::Mat depth_in;

		depth_in = cv::cvarrToMat(im);
		double min, max;
		cv::minMaxIdx(depth_in, &min, &max);
		depth_in.convertTo(depth_in, CV_32F);

		cv::Mat adjMap;
		depth_in.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

		cv::Mat test = 255 - adjMap;
		cv::Mat falseColorsMap;
		applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

		cv::imshow("Out", adjMap);
		cv::waitKey();

		cv::imshow("Out", falseColorsMap);
		cv::waitKey();

		cv::imshow("Out", test);
		cv::waitKey();
	}
	{

		// ---------- read and transform to pcl cloud
		cv::Mat intrinsicK = cv::Mat::zeros(3, 3, CV_32F);
		intrinsicK.at<float>(0, 0) = 572.41140;
		intrinsicK.at<float>(1, 1) = 573.57043;
		intrinsicK.at<float>(2, 2) = 1;
		intrinsicK.at<float>(0, 2) = 325.26110;
		intrinsicK.at<float>(1, 2) = 242.04899;


		pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>);

		cv_depth_2_pcl_cloud(input_file_name, intrinsicK, scene);

		//pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
		//zyk::readPointCloud("D:/Documents/Projects/PCL/code/datafile/ACCV3D/lamp/mesh.ply",model);

		//
		//make transform to model
		//
		//Eigen::Matrix4f rt;
		////rt << -0.0963063, 0.994044, -0.0510079, -111.814,
		////	-0.573321, -0.0135081, 0.81922, -78.3622,
		////	0.813651, 0.10814, 0.571207, 1036.12,
		////	0, 0, 0, 1;
		//// lamp 0
		//rt << -0.00145487, -0.993196, 0.116445, 98.7207,
		//	0.89669, 0.0502498, 0.439798, -120.88,
		//	-0.442657, 0.105055, 0.890516, 1087.96,
		//	0, 0, 0, 1;
		if (show_flag)
		{
			pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
			//pcl::transformPointCloud(*model, *rotated_model, rt);

			pcl::visualization::PCLVisualizer keyPointVisual("Depth2PC");
			keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 0.0, 255.0, 0.0), "scene");
			//keyPointVisual.addPointCloud(rotated_model, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 255.0, 0.0, 0.0), "model");
			keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
			keyPointVisual.spin();
		}

		//save the ply scne
		if(save_flag)
			pcl::io::savePLYFile(output_file_name, *scene);
	}

	return 0;
}
