#include <string.h>
#include <fstream>
#include <opencv2/rgbd.hpp>
#include <common.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
int main(int argc, char**argv)
{
	string s("D:/Documents/Projects/PCL/code/datafile/ACCV3D/ape/data/depth44.dpt");
	IplImage* im = loadDepth(s);
	cv::Mat depth_in;
	depth_in = cv::cvarrToMat(im);
	double min, max;
	cv::minMaxIdx(depth_in, &min, &max);
	depth_in.convertTo(depth_in, CV_32F);
	
	cv::Mat adjMap;
	depth_in.convertTo(adjMap, CV_8UC1, 255/ (max - min), -min);

	cv::Mat test = 255 - adjMap;
	cv::Mat falseColorsMap;
	applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

	cv::imshow("Out", adjMap);
	cv::waitKey();

	cv::imshow("Out", falseColorsMap);
	cv::waitKey();

	cv::imshow("Out", test);
	cv::waitKey();
	//{
	// // ---------- read and transform to pcl cloud
	//	cv::Mat intrinsicK = cv::Mat::zeros(3, 3, CV_32F);
	//	intrinsicK.at<float>(0, 0) = 572.41140;
	//	intrinsicK.at<float>(1, 1) = 573.57043;
	//	intrinsicK.at<float>(2, 2) = 1;
	//	intrinsicK.at<float>(0, 2) = 325.26110;
	//	intrinsicK.at<float>(1, 2) = 242.04899;


	//	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>);

	//	cv_depth_2_pcl_cloud(s, intrinsicK, scene, 1);

	//	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	//	readPointCloud("D:/Documents/Projects/PCL/code/datafile/ACCV3D/ape/mesh.ply", "ply", model);

	//	//
	//	//make transform to model
	//	//
	//	Eigen::Matrix4f rt;
	//	rt << -0.0963063, 0.994044, -0.0510079, -111.814,
	//		-0.573321, -0.0135081, 0.81922, -78.3622,
	//		0.813651, 0.10814, 0.571207, 1036.12,
	//		0, 0, 0, 1;
	//	pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
	//	pcl::transformPointCloud(*model, *rotated_model, rt);

	//	pcl::visualization::PCLVisualizer keyPointVisual("Depth2PC");
	//	keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 0.0, 255.0, 0.0), "scene");
	//	keyPointVisual.addPointCloud(rotated_model, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 255.0, 0.0, 0.0), "model");
	//	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
	//	keyPointVisual.spin();
	//}

	return 0;
}
