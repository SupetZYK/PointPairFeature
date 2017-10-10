#include <string.h>
#include <fstream>
#include <opencv2/rgbd.hpp>
#include <common.h>
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
	if (pc.depth() != CV_32F)
	{
		pc.convertTo(pc, CV_32F);
	}
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

int main(int argc, char**argv)
{
	IplImage* pImg =  loadDepth("D:/Documents/Projects/PCL/code/datafile/ACCV3D/ape/data/depth0.dpt");
	cv::Mat depth_in;
	depth_in = cv::cvarrToMat(pImg);
	cv::Mat outPC;
	cv::Mat intrinsicK=cv::Mat::zeros(3, 3, CV_32F);
	intrinsicK.at<float>(0, 0) = 572.41140;
	intrinsicK.at<float>(1, 1) = 573.57043;
	intrinsicK.at<float>(2, 2) = 1;
	intrinsicK.at<float>(0, 2) = 325.26110;
	intrinsicK.at<float>(1, 2) = 242.04899;
	cv::rgbd::depthTo3d(depth_in, intrinsicK, outPC);


	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>);
	cv_mat_cloud_to_pcl_cloud(outPC, scene);


	pcl::visualization::PCLVisualizer keyPointVisual("Depth2PC");
	keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 0.0, 255.0, 0.0), "scene_keypoints");
	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
	keyPointVisual.spin();

	return 0;
}
