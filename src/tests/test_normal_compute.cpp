#include "common.h"
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;
using namespace pcl;
std::string model_filename_;


//Algorithm params
bool show_keypoints_ (false);
bool use_cloud_resolution_ (false);
bool use_mls_(false);
bool use_existing_normal_data_(false);
bool normal_reorient_switch_ (false);
bool save_compute_point_normal(false);
float scene_ss_ (0.03f);
float normal_r = 5;

//iss
//float iss_sr_ (6);
//float iss_nr_ (4);
void showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " scene_filename.ply [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show keypoints." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "     --mls:                  Using mls" << std::endl;
  std::cout << "     --rn:					 Reorient normal" << std::endl;
  std::cout << "     --in:					 Use existing normal data" << std::endl;
  std::cout << "     --save:				 Save compute point normal" << std::endl;
  std::cout << "     --ss val:         		 Uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --nr val:         		 Radius for compute normals (default 5)" << std::endl;

}

void parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--mls"))
  {
	  use_mls_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--in"))
  {
	  use_existing_normal_data_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--rn"))
  {
	  normal_reorient_switch_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--save"))
  {
	  save_compute_point_normal = true;
  }
	std::vector<int> filenames;

	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	if (filenames.size () >= 1)
	{
		model_filename_ = argv[filenames[0]];
		cout << "Input file: " << model_filename_ << endl;
	}
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() >= 1)
	{
		model_filename_ = argv[filenames[0]];
		cout << "Input file: " << model_filename_ << endl;
	}
	if (model_filename_.empty()) {
		cout << "No file input!" << endl;
		exit(-1);
	}
	pcl::console::parse_argument(argc, argv, "--ss", scene_ss_);
	pcl::console::parse_argument(argc, argv, "--nr", normal_r);
}





int
main(int argc, char *argv[])
{

	parseCommandLine(argc, argv);

	showHelp(argv[0]);
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr compute_keyNormals(new pcl::PointCloud<NormalType>());
	if (use_existing_normal_data_)
	{
		if (!readPointCloud(model_filename_, "ply", scene, scene_normals))
		{
			return(-1);
		}
	}
	else
	{
		if (!readPointCloud(model_filename_, "ply", scene))
		{
			return(-1);
		}
	}
	cout<<"Read finish"<<endl;
	double max_coord[3];
	double min_coord[3];
	float resolution = static_cast<float> (computeCloudResolution(scene,max_coord, min_coord));
	cout<<"Scene resolution : "<<resolution<<endl;
	if (use_cloud_resolution_)
	{
		if (resolution != 0.0f)
		{
			//model_ss_ *= resolution;
			scene_ss_ *= resolution;
			normal_r *= resolution;
		}
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "Normal compute radius is:    " << normal_r << std::endl;
	}
	Eigen::Vector3f model_approximate_center;
	model_approximate_center(0) = (max_coord[0] + min_coord[0]) / 2;
	model_approximate_center(1) = (max_coord[1] + min_coord[1]) / 2;
	model_approximate_center(2) = (max_coord[2] + min_coord[2]) / 2;
	pcl::IndicesPtr sampled_index_ptr;
	if (scene_ss_ < 0) {
		scene_keypoints = scene;
		scene_keyNormals = scene_keyNormals;
		std::cout << "No sapling" << std::endl;
	}
	else {
		if (scene_normals->empty())
			sampled_index_ptr = uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);
		else
			sampled_index_ptr = uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints, scene_keyNormals);

		//sampled_index_ptr=uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints, scene_keyNormals);
		std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;
	}
	

	//
	//visualize keypoints
	//

	if (show_keypoints_ && use_existing_normal_data_)
	{
		pcl::visualization::PCLVisualizer keyPointVisual("Existing normals");

		keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
		//		keyPointVisual.addPointCloudNormals<PointType, NormalType>(model_keypoints, model_keyNormals, 1, 10, "model_normals");
		keyPointVisual.addPointCloudNormals<PointType, NormalType>(scene_keypoints, scene_keyNormals, 1, 0.1, "scene_normals");
		//model
		// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
		//keyPointVisual.addPointCloud(model_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model_keypoints, 0.0, 0.0, 255.0), "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
		keyPointVisual.spin();
	}


	if (use_mls_)
	{
		pcl::PointCloud<PointType>::Ptr pnts_tmp(new pcl::PointCloud<PointType>());
		MovingLeastSquares<PointType, PointType> mls;
		search::KdTree<PointType>::Ptr tree;
		// Set parameters
		mls.setInputCloud(scene);
		mls.setComputeNormals(true);
		mls.setPolynomialFit(true);
		mls.setSearchMethod(tree);
		if (scene_ss_ > 0) {
			mls.setIndices(sampled_index_ptr);
		}
		mls.setSearchRadius(normal_r);
		mls.process(*pnts_tmp);
		scene_keypoints = pnts_tmp;
		compute_keyNormals = mls.getNormals();
		//compute_keyNormals = mls.getNormals();
		//for (size_t i = 0; i < compute_keyNormals->size(); ++i)
		//{
		//	flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, compute_keyNormals->at(i).normal[0], compute_keyNormals->at(i).normal[1], compute_keyNormals->at(i).normal[2]);
		//}
	}
	else
	{
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		search::KdTree<PointType>::Ptr tree;
		if (scene_ss_ > 0) {
			norm_est.setIndices(sampled_index_ptr);
		}
		norm_est.setSearchMethod(tree);
		norm_est.setRadiusSearch(normal_r);
			
		//norm_est.setKSearch(10);
		norm_est.setInputCloud(scene);
		norm_est.compute(*compute_keyNormals);

	}
	cout << "Normal compute complete£¡" << endl;

	if (normal_reorient_switch_)
	{
		for (int32_t i = 0; i < compute_keyNormals->size(); ++i)
		{
			Eigen::Vector3f pnt_temp = scene_keypoints->points[i].getVector3fMap();
			Eigen::Vector3f normal_temp = compute_keyNormals->points[i].getNormalVector3fMap();
			if ((pnt_temp - model_approximate_center).dot(normal_temp) < 0)
			{
				compute_keyNormals->points[i].normal_x = -normal_temp(0);
				compute_keyNormals->points[i].normal_y = -normal_temp(1);
				compute_keyNormals->points[i].normal_z = -normal_temp(2);
			}
		}
	}

	if (show_keypoints_)
	{
		pcl::visualization::PCLVisualizer keyPointVisual("Compute normals");
		keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
		//		keyPointVisual.addPointCloudNormals<PointType, NormalType>(model_keypoints, model_keyNormals, 1, 10, "model_normals");
		keyPointVisual.addPointCloudNormals<PointType, NormalType>(scene_keypoints, compute_keyNormals, 1, 10, "scene_normals");
		//model
		// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
		//keyPointVisual.addPointCloud(model_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model_keypoints, 0.0, 0.0, 255.0), "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
		keyPointVisual.spin();
	}

	if (save_compute_point_normal) {

		pcl::PointCloud<pcl::PointNormal>::Ptr save_pnt_normal(new pcl::PointCloud<pcl::PointNormal>());
		for (size_t i = 0; i < scene_keypoints->size(); ++i) {
			pcl::PointNormal _tem;
			_tem.x = scene_keypoints->points[i].x;
			_tem.y = scene_keypoints->points[i].y;
			_tem.z = scene_keypoints->points[i].z;
			_tem.normal_x = compute_keyNormals->points[i].normal_x;
			_tem.normal_y = compute_keyNormals->points[i].normal_y;
			_tem.normal_z = compute_keyNormals->points[i].normal_z;
			save_pnt_normal->push_back(_tem);
		}
		int pos = model_filename_.find_last_of('.');
		std::string save_filename_ = model_filename_.substr(0, pos);
		save_filename_ += "_compute_normal.ply";
		pcl::io::savePLYFile(save_filename_, *save_pnt_normal);
		std::cout << "save success!" << endl;
	}


	return (0);
}
