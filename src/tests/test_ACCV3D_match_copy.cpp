//#include <OtherHeaders.h>
#include <common.h>
#include "PPFFeature.h"
#include "pose_cluster.h"
//pcl
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>




//std::string model_filename_;
std::string model_ppfs_filename_;
std::string scene_filename_;
//std::string model_normal_filename_;
std::string scene_normal_filename_;

using namespace pcl;
//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool show_cluster_result_ (false);
bool show_cluster_together_(false);
bool use_cloud_resolution_ (false);
bool two_ball_switch_  (false);
//bool use_ply_filetype_ (false);
//bool use_iss_(false);
bool use_existing_normal_data_ (false);
bool spread_ppf_switch_(false);
bool use_mls_(false);
//bool use_ppfs_file_  (false);
//float model_ss_ (0.01f);
float scene_ss_ (1.0f);
float angle_thresh = M_PI / 15;
float cluster_dis_thresh = 0.2;
float recopute_score_dis_thresh = 2;
float recopute_score_ang_thresh = -1;
float relativeReferencePointsNumber = 0.2;
float icp_dis_thresh=0.3;
float max_vote_thresh (0.5);
float max_vote_percentage (0.8);
int num_clusters_per_group = 2;
float show_vote_thresh (0.3);
int max_show_number (10);
float curvature_radius_(2.0f);


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
  std::cout << "     -h:                    Show this help." << std::endl;
  std::cout << "     -k:                    Show used keypoints." << std::endl;
  std::cout << "     -c:                    Show used correspondences." << std::endl;
  std::cout << "     -r:                    Use the model cloud resolution and multiply to get scene sampling resolution" << std::endl;
  std::cout << "     --sc:                  Show cluster results" << std::endl;
  std::cout << "     --st:                  Show cluster results together, only when 'sc' is input" << std::endl;
  std::cout << "     --tb:					Two ball switch" << std::endl;
  std::cout << "     --in:					Use existing normal files" << std::endl;
  std::cout << "     --mls:					Use moving least squares" << std::endl;
  std::cout << "     --sppf:				Spread discretized ppf" << std::endl;
  std::cout << "     --scene_ss val:        Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --angle_thresh val:    angle thresh when do ppf clustering" << std::endl;
  std::cout << "     --dis_thresh val:		first distance thresh(relative to radius)" << std::endl;
  std::cout << "     --para_1 val:			relative reference points number in ppf matching"<< std::endl;
  std::cout << "     --para_2 val:			max vote thresh, relative to the number of points in the current box"<< std::endl;
  std::cout << "     --para_3 val:			if the vote in the accumulator is greater than a certain thresh, then the instance is considered, this is the ratio of thresh to max_vote" << std::endl;
  std::cout << "     --para_4 val:			Number of clusters per group.set to '-1' to close group" << std::endl;
  std::cout << "     --re_d val:			recompute score distance thresh, relative to model resolution.Set to '-1' to disable recompute score" << std::endl;
  std::cout << "     --re_a val:			recompute score angle thresh, Set to '-1' to disable using angle thresh" << std::endl;
  std::cout << "     --show_thresh val:     the clusters whose vote is greater than this will be displayed" << std::endl;
  std::cout << "     --icp_thresh val:		max clusters per pose can be in" << std::endl;
  std::cout << "     --curv_r val:			curvature radius" << std::endl;
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
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "--sc"))
  {
    show_cluster_result_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--st"))
  {
	  show_cluster_together_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--in"))
  {
	  use_existing_normal_data_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--mls"))
  {
	  use_mls_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--sppf"))
  {
	  spread_ppf_switch_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--tb"))
  {
	  two_ball_switch_ = true;
  }

	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ppfs");
	if (filenames.size() < 1)
	{
		std::cout << "template filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}
	model_ppfs_filename_ = argv[filenames[0]];
	cout<<"Input ppfs file: "<<model_ppfs_filename_<<endl;


  //Model & scene filenames
	filenames.clear()	;
  //if(use_ply_filetype_==false)
	  //filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  //else
	 filenames = pcl::console::parse_file_extension_argument (argc, argv, ".dpt");
	if (filenames.size () < 1)
	{
	 std::cout << "Filenames missing.\n";
	 showHelp (argv[0]);
	 exit (-1);
	}
	//model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[0]];
	cout<<"Input scene file: "<<scene_filename_<<endl;
  //General parameters
  //pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument(argc, argv, "--angle_thresh", angle_thresh);
  pcl::console::parse_argument(argc, argv, "--dis_thresh", cluster_dis_thresh);
  pcl::console::parse_argument(argc, argv, "--para_1", relativeReferencePointsNumber);
  pcl::console::parse_argument(argc, argv, "--para_2", max_vote_thresh);
  pcl::console::parse_argument(argc, argv, "--para_3", max_vote_percentage);
  pcl::console::parse_argument(argc, argv, "--para_4", num_clusters_per_group);
  pcl::console::parse_argument(argc, argv, "--re_d", recopute_score_dis_thresh);
  pcl::console::parse_argument(argc, argv, "--re_a", recopute_score_ang_thresh);
  pcl::console::parse_argument(argc, argv, "--show_thresh", show_vote_thresh);
  pcl::console::parse_argument(argc, argv, "--icp_thresh", icp_dis_thresh);
  pcl::console::parse_argument(argc, argv, "--curv_r", curvature_radius_);
}





int
main(int argc, char *argv[])
{

	//parseCommandLine(argc, argv);

	//showHelp(argv[0]);
	////pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	////pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	//pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());


	//cv::Mat intrinsicK=cv::Mat::zeros(3, 3, CV_32F);
	//intrinsicK.at<float>(0, 0) = 572.41140;
	//intrinsicK.at<float>(1, 1) = 573.57043;
	//intrinsicK.at<float>(2, 2) = 1;
	//intrinsicK.at<float>(0, 2) = 325.26110;
	//intrinsicK.at<float>(1, 2) = 242.04899;
	//
	//cv_depth_2_pcl_cloud(scene_filename_,intrinsicK,scene);
	////pcl::io::savePLYFile(scene_filename_ + "_changed", *scene);
	//cout<<"Read finish"<<endl;
	//double max_coord[3];
	//double min_coord[3];
	//float resolution = static_cast<float> (computeCloudResolution(scene, max_coord, min_coord));
	//cout << "res: " << resolution << endl;
	//cout << "max:" << max_coord[0] << ',' << max_coord[1] << ',' << max_coord[2] << endl;
	//cout << "max:" << min_coord[0] << ',' << min_coord[1] << ',' << min_coord[2] << endl;

	////
	//// some global variables
	////
	//zyk::PPF_Space model_feature_space;
	//pcl::PointCloud<NormalType>::Ptr model_keyNormals(new pcl::PointCloud<NormalType>());
	//pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());

	//if (!model_feature_space.load(model_ppfs_filename_))
	//{
	//	std::cout << "reading ppfs file fail!" << std::endl;
	//	return -1;
	//}
	//cout << "ppfs file load finish" << endl;
	//	//CFile fileload;
	//	//fileload.Open(model_ppfs_filename_.c_str(), CFile::modeRead);
	//	//CArchive ar(&fileload, CArchive::load);
	//	//model_feature_space.Serialize(ar);
	//	//ar.Close();
	//	//fileload.Close();
	//	model_feature_space.getPointCloud(model_keypoints);
	//	model_feature_space.getPointNormalCloud(model_keyNormals);
	////}

	//float model_length=model_feature_space.model_size[0];
	//float model_width=model_feature_space.model_size[1];
	//float model_height=model_feature_space.model_size[2];
	//std::cout << "Model resolution: " << model_feature_space.model_res << std::endl;

	//std::cout << "Model length: " << model_length << std::endl;
	//std::cout << "Model width: " << model_width << std::endl;
	//std::cout << "Model height: " << model_height << std::endl;

	//if (use_cloud_resolution_)
	//{
	//	scene_ss_ *= model_feature_space.model_res;
	//	cout << "Scene sampling resolution is: " << scene_ss_ << endl;
	//}




	////
	////  Downsample Clouds to Extract keypoints
	////
	//pcl::IndicesPtr sampled_index_ptr;

	//sampled_index_ptr = uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);
	//std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;

	////
	////  Compute Normals
	////
	//if (!use_mls_)
	//{
	//	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	//	norm_est.setIndices(sampled_index_ptr);
	//	//norm_est.setKSearch(20);
	//	norm_est.setRadiusSearch(scene_ss_);
	//	norm_est.setInputCloud(scene);
	//	norm_est.compute(*scene_keyNormals);
	//}
	//else
	//{
	//	MovingLeastSquaresOMP<PointType, PointType> mls;
	//	search::KdTree<PointType>::Ptr tree;
	//	// Set parameters
	//	mls.setInputCloud(scene);
	//	mls.setIndices(sampled_index_ptr);
	//	mls.setComputeNormals(true);
	//	mls.setPolynomialOrder(2);
	//	mls.setSearchMethod(tree);
	//	mls.setSearchRadius(scene_ss_);
	//	mls.process(*scene_keypoints);

	//	scene_keyNormals = mls.getNormals();
	//	for (size_t i = 0; i < scene_keyNormals->size(); ++i)
	//	{
	//		flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
	//	}
	//}

	//cout << "Normal compute complete£¡" << endl;


	//
	//visualize keypoints
	//

//	if (show_keypoints_)
//	{
//		pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
//		//keyPointVisual.addCoordinateSystem(0.2);
//		//scene
//		// keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(scene, 255.0, 0.0, 0.0), "scene");
//		keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
//		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_keypoints");
//		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
////		keyPointVisual.addPointCloudNormals<PointType, NormalType>(model_keypoints, model_keyNormals, 1, 10, "model_normals");
//		keyPointVisual.addPointCloudNormals<PointType, NormalType>(scene_keypoints, scene_keyNormals, 1, 10, "scene_normals");
//		//model
//		// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
//		//keyPointVisual.addPointCloud(model_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model_keypoints, 0.0, 0.0, 255.0), "model_keypoints");
//		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
//		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
//		keyPointVisual.spin();
//	}

	//
	//  Compute Model Descriptors  PPF Space for model
	//

	//vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> pose_clusters;

	//cout <<">>Begin match!"<<endl;
	//cout <<">cluster angle thresh: "<<angle_thresh<<endl;
	//cout <<">cluster distance thresh: "<<cluster_dis_thresh<<endl;
	//cout <<">relativeReferencePointsNumber: "<<relativeReferencePointsNumber<<endl;
	//cout <<">max_vote_thresh: "<<max_vote_thresh<<endl;
	//cout <<">max_vote_percentage: "<<max_vote_percentage<<endl;
	//cout <<">recompute score distance thresh: " << recopute_score_dis_thresh << endl;
	//cout << ">recompute score angle thresh: " << recopute_score_ang_thresh << endl;
	//cout << "num clusters per group: " << num_clusters_per_group << endl;
	////model_feature_space.setScenePntsFlag(&scene_pnt_flag);
	//model_feature_space.match(scene_keypoints, scene_keyNormals, spread_ppf_switch_, two_ball_switch_,relativeReferencePointsNumber, max_vote_thresh, max_vote_percentage, angle_thresh, cluster_dis_thresh, recopute_score_dis_thresh, recopute_score_ang_thresh, num_clusters_per_group, pose_clusters);
	//cout << "clusters size : " << pose_clusters.size() << endl;

  return (0);
}



