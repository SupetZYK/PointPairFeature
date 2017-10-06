
#include "common.h"
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"
#include "SmartSampling.hpp"
#include <pcl/filters/normal_space.h>
std::string model_file_name_="../../../datafile/plat.ply";

#define test_my_smart_sampling

int main(int argc, char**argv)
{
	//test ppf compute
#ifdef test_ppf_compute

	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<PointType>::Ptr model_new(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	//pcl::PointCloud<NormalType>::Ptr model_normals_new(new pcl::PointCloud<NormalType>());
	readPointCloud(model_file_name_, "ply", model, model_normals);

	float x = 0.0001;
	bool b = (x > -FLT_EPSILON) && (x < FLT_EPSILON);
	cout << "b: " << b << endl;
	for (int i = 2000; i < 2020; i++)
	{
		zyk::PPF ppf1;
		zyk::PPF_Space::computeSinglePPF(model, model_normals, i + 5000, i, ppf1);
		cout << "current ppf is: " << ppf1.ppf.f1 << '\t' << ppf1.ppf.f2 << '\t' << ppf1.ppf.f3 << '\t' << ppf1.ppf.f4 << '\t' << ppf1.ppf.alpha_m << endl;
	}

#endif

#ifdef test_get_neiboring
	Eigen::Vector3i grid_div(3, 3, 4);
	Eigen::Vector3i grid_div_mul(1, 3, 9);
	Eigen::Vector3i ijk(1, 2, 3);
	cout << "grid div is : " << grid_div.transpose() << endl;
	//std::vector<int> v1;
	//zyk::getNeiboringBoxIndex3D(ijk, grid_div,v1);
	////cout << "first vector: " << v1 << endl;
	int index = grid_div_mul.dot(ijk);
	std::vector<int> v2;
	v2.push_back(99999999);
	zyk::getNeiboringBoxIndex3D(index, grid_div,v2);

#endif
#ifdef test_eigen_copy

	Eigen::Vector3f a(2, 3, 4);
	Eigen::Vector3f& b=a;
	b(0) = 3;
	cout << a << endl;

	PointType pt;
	pt.x = 3;
	pt.y = 4;
	pt.z = 5;
	const Eigen::Vector3f& c = pt.getVector3fMap();

	pt.x = 100;
	cout << c << endl;
#endif
#ifdef test_normal_sampling
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());

	readPointCloud(model_file_name_, "ply", model, model_normals);

	pcl::PointCloud<PointType>::Ptr model_new(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals_new(new pcl::PointCloud<NormalType>());

	pcl::NormalSpaceSampling<PointType, NormalType> normal_space_sampling;

	normal_space_sampling.setInputCloud(model);
	normal_space_sampling.setNormals(model_normals);
	normal_space_sampling.setBins(4, 4, 4);
	normal_space_sampling.setSeed(0);
	normal_space_sampling.setSample(static_cast<unsigned int> (model->size()) / 10);

	normal_space_sampling.filter(*model_new);
	
	//visualize
	pcl::visualization::PCLVisualizer key_visual("keyPoint detect");
	key_visual.addCoordinateSystem(20);
	//key_visual.addPointCloudNormals<PointType, NormalType>(model_new, model_no_zero_curvature_keyNormals, 1, 10, "model_no_zero_normals");
	key_visual.addPointCloud(model_new, pcl::visualization::PointCloudColorHandlerCustom<PointType>(model_new, 0.0, 0.0, 255.0), "model_new");
	key_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "model_new");
	key_visual.spin();


#endif

#ifdef test_my_smart_sampling
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());

	readPointCloud(model_file_name_, "ply", model, model_normals);

	pcl::PointCloud<PointType>::Ptr model_new(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals_new(new pcl::PointCloud<NormalType>());
	pcl::SmartSampling<PointType,NormalType> smart_samp;

	smart_samp.setInputCloud(model);
	smart_samp.setNormals(model_normals);
	smart_samp.setRadiusSearch(8.5);
	smart_samp.setAngleThresh(0.08);
	smart_samp.filter(*model_new);

	//visualize
	pcl::visualization::PCLVisualizer key_visual("keyPoint detect");
	key_visual.addCoordinateSystem(20);
	//key_visual.addPointCloudNormals<PointType, NormalType>(model_new, model_no_zero_curvature_keyNormals, 1, 10, "model_no_zero_normals");
	key_visual.addPointCloud(model_new, pcl::visualization::PointCloudColorHandlerCustom<PointType>(model_new, 0.0, 0.0, 255.0), "model_new");
	key_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "model_new");
	key_visual.spin();


#endif
	getchar();

		
		

	return 0;
}
