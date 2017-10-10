#include "common.h"
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"
#include <pcl/surface/mls.h>


std::string model_filename_;


//Algorithm params
bool show_keypoints_ (false);
bool use_cloud_resolution_ (false);
bool use_mls_(false);
float scene_ss_ (0.03f);


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
  std::cout << "Usage: " << filename << " ppfs_filename.ppfs scene_filename.ply [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show keypoints." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "     --mls:                  Using mls" << std::endl;
  std::cout << "     --ss val:         		 Uniform sampling radius (default 0.03)" << std::endl;
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

	std::vector<int> filenames;

	 filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	if (filenames.size () < 1)
	{
	 std::cout << "Filenames missing.\n";
	 showHelp (argv[0]);
	 exit (-1);
	}
	//model_filename_ = argv[filenames[0]];
	model_filename_ = argv[filenames[0]];
	cout << "Input file: " << model_filename_ << endl;
	pcl::console::parse_argument (argc, argv, "--ss", scene_ss_);
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
	if (!readPointCloud(model_filename_, "ply", scene, scene_normals))
	{
		return(-1);
	}
	cout<<"Read finish"<<endl;
	
	float resolution = static_cast<float> (computeCloudResolution(scene));
	cout<<"Scene resolution : "<<resolution<<endl;
	if (use_cloud_resolution_)
	{
		if (resolution != 0.0f)
		{
			//model_ss_ *= resolution;
			scene_ss_ *= resolution;
		}
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
	}
	
	pcl::IndicesPtr sampled_index_ptr = uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints,scene_keyNormals);

	//sampled_index_ptr=uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints, scene_keyNormals);
	std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;

	//
	//visualize keypoints
	//

	if (show_keypoints_)
	{
		pcl::visualization::PCLVisualizer keyPointVisual("Halcon normals");

		keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
		//		keyPointVisual.addPointCloudNormals<PointType, NormalType>(model_keypoints, model_keyNormals, 1, 10, "model_normals");
		keyPointVisual.addPointCloudNormals<PointType, NormalType>(scene_keypoints, scene_keyNormals, 1, 10, "scene_normals");
		//model
		// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
		//keyPointVisual.addPointCloud(model_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model_keypoints, 0.0, 0.0, 255.0), "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
		keyPointVisual.spin();
	}


	if (use_mls_)
	{
		MovingLeastSquares<PointType, PointType> mls;
		search::KdTree<PointType>::Ptr tree;
		// Set parameters
		mls.setInputCloud(scene);
		mls.setIndices(sampled_index_ptr);
		mls.setComputeNormals(true);
		mls.setPolynomialFit(true);
		mls.setSearchMethod(tree);
		mls.setSearchRadius(scene_ss_);
		mls.process(*scene_keypoints);

		compute_keyNormals = mls.getNormals();
		for (size_t i = 0; i < compute_keyNormals->size(); ++i)
		{
			flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, compute_keyNormals->at(i).normal[0], compute_keyNormals->at(i).normal[1], compute_keyNormals->at(i).normal[2]);
		}
	}
	else
	{
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		norm_est.setIndices(sampled_index_ptr);
		norm_est.setRadiusSearch(scene_ss_);
		//norm_est.setKSearch(10);
		norm_est.setInputCloud(scene);
		norm_est.compute(*compute_keyNormals);


	}
	cout << "Normal compute complete£¡" << endl;



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

	return (0);
}
