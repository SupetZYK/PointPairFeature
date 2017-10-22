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
//bool use_ply_filetype_ (false);
//bool use_iss_(false);
bool use_existing_normal_data_ (false);
bool use_mls_(false);
//bool use_ppfs_file_  (false);
//float model_ss_ (0.01f);
float scene_ss_ (1.0f);
float angle_thresh=M_PI/15;
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
//iss
//float iss_sr_ (6);
//float iss_nr_ (4);
class test_ppf_feature_space :public zyk::PPF_Space
{
public:
	void match(pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<NormalType>::Ptr scene_normals, float relativeReferencePointsNumber, float max_vote_thresh, float max_vote_percentage, float angle_thresh, float first_dis_thresh, float recompute_score_dis_thresh, float recompute_score_ang_thresh, int num_clusters_per_group, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> &pose_clusters);
	void setScenePntsFlag(vector<int>*flag_vec) { scene_pnt_flag = flag_vec; };
protected:
	vector<int>*scene_pnt_flag = NULL;
};

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
  std::cout << "     --in:					Use existing normal files" << std::endl;
  std::cout << "     --mls:					Use moving least squares" << std::endl;
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

	parseCommandLine(argc, argv);

	showHelp(argv[0]);
	//pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());


	cv::Mat intrinsicK=cv::Mat::zeros(3, 3, CV_32F);
	intrinsicK.at<float>(0, 0) = 572.41140;
	intrinsicK.at<float>(1, 1) = 573.57043;
	intrinsicK.at<float>(2, 2) = 1;
	intrinsicK.at<float>(0, 2) = 325.26110;
	intrinsicK.at<float>(1, 2) = 242.04899;
	
	cv_depth_2_pcl_cloud(scene_filename_,intrinsicK,scene);
	//pcl::io::savePLYFile(scene_filename_ + "_changed", *scene);
	cout<<"Read finish"<<endl;
	double max_coord[3];
	double min_coord[3];
	float resolution = static_cast<float> (computeCloudResolution(scene, max_coord, min_coord));
	cout << "res: " << resolution << endl;
	cout << "max:" << max_coord[0] << ',' << max_coord[1] << ',' << max_coord[2] << endl;
	cout << "max:" << min_coord[0] << ',' << min_coord[1] << ',' << min_coord[2] << endl;

	//
	// some global variables
	//
	test_ppf_feature_space model_feature_space;
	pcl::PointCloud<NormalType>::Ptr model_keyNormals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());

	if (!model_feature_space.load(model_ppfs_filename_))
	{
		std::cout << "reading ppfs file fail!" << std::endl;
		return -1;
	}
	cout << "ppfs file load finish" << endl;
		//CFile fileload;
		//fileload.Open(model_ppfs_filename_.c_str(), CFile::modeRead);
		//CArchive ar(&fileload, CArchive::load);
		//model_feature_space.Serialize(ar);
		//ar.Close();
		//fileload.Close();
		model_feature_space.getPointCloud(model_keypoints);
		model_feature_space.getPointNormalCloud(model_keyNormals);
	//}

	float model_length=model_feature_space.model_size[0];
	float model_width=model_feature_space.model_size[1];
	float model_height=model_feature_space.model_size[2];
	std::cout << "Model resolution: " << model_feature_space.model_res << std::endl;

	std::cout << "Model length: " << model_length << std::endl;
	std::cout << "Model width: " << model_width << std::endl;
	std::cout << "Model height: " << model_height << std::endl;

	if (use_cloud_resolution_)
	{
		scene_ss_ *= model_feature_space.model_res;
		cout << "Scene sampling resolution is: " << scene_ss_ << endl;
	}




	//
	//  Downsample Clouds to Extract keypoints
	//
	pcl::IndicesPtr sampled_index_ptr;

	sampled_index_ptr = uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;

	//
	//  Compute Normals
	//
	if (!use_mls_)
	{
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		norm_est.setIndices(sampled_index_ptr);
		//norm_est.setKSearch(20);
		norm_est.setRadiusSearch(scene_ss_);
		norm_est.setInputCloud(scene);
		norm_est.compute(*scene_keyNormals);
	}
	else
	{
		MovingLeastSquaresOMP<PointType, PointType> mls;
		search::KdTree<PointType>::Ptr tree;
		// Set parameters
		mls.setInputCloud(scene);
		mls.setIndices(sampled_index_ptr);
		mls.setComputeNormals(true);
		mls.setPolynomialFit(true);
		mls.setSearchMethod(tree);
		mls.setSearchRadius(scene_ss_);
		mls.process(*scene_keypoints);

		scene_keyNormals = mls.getNormals();
		for (size_t i = 0; i < scene_keyNormals->size(); ++i)
		{
			flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
		}
	}

	cout << "Normal compute complete��" << endl;

	////
	//// Compute curvature
	////

	//pcl::PrincipalCurvaturesEstimation<PointType, NormalType> curv_est;
	//pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv(new pcl::PointCloud<pcl::PrincipalCurvatures>());
	//curv_est.setInputCloud(scene_keypoints);
	//curv_est.setInputNormals(scene_keyNormals);
	//curv_est.setRadiusSearch(curvature_radius_ * scene_ss_);
	//curv_est.compute(*curv);


	////
	//// show those curvature 0 and none_zero separately
	////
	//pcl::PointCloud<PointType>::Ptr pnts_zero(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<NormalType>::Ptr normals_zero(new pcl::PointCloud<NormalType>());
	//pcl::PointCloud<PointType>::Ptr pnts_no_zero(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<NormalType>::Ptr normals_no_zero(new pcl::PointCloud<NormalType>());
	////flag vec
	//vector<int> scene_pnt_flag;
	//scene_pnt_flag.resize(scene_keypoints->size(),0);
	//for (int i = 0; i < scene_keypoints->size(); ++i)
	//{
	//	pcl::PrincipalCurvatures c = curv->at(i);
	//	if (abs(c.pc1) < 0.1 && abs(c.pc2) < 0.1)
	//	{
	//		scene_pnt_flag[i] = 1;
	//		pnts_zero->push_back(scene_keypoints->at(i));
	//		normals_zero->push_back(scene_keyNormals->at(i));
	//	}
	//	else
	//	{
	//		pnts_no_zero->push_back(scene_keypoints->at(i));
	//		normals_no_zero->push_back(scene_keyNormals->at(i));
	//	}
	//}
	////
	//// visualize split
	////
	////
	////visualize zero
	////

	//pcl::visualization::PCLVisualizer zeroVisual("zero detect");
	//zeroVisual.addCoordinateSystem(20);
	//zeroVisual.addPointCloud(pnts_zero, pcl::visualization::PointCloudColorHandlerCustom<PointType>(pnts_zero, 0.0, 0.0, 255.0), "zero_pnts");
	//zeroVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "zero_pnts");
	//zeroVisual.spin();

	////
	////visualize no-zero
	////

	//pcl::visualization::PCLVisualizer nozeroVisual("no-zero detect");
	//nozeroVisual.addCoordinateSystem(20);
	//nozeroVisual.addPointCloud(pnts_no_zero, pcl::visualization::PointCloudColorHandlerCustom<PointType>(pnts_no_zero, 0.0, 0.0, 255.0), "pnts_no_zero");
	//nozeroVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pnts_no_zero");
	//nozeroVisual.spin();


	//
	//visualize keypoints
	//

	if (show_keypoints_)
	{
		pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
		//keyPointVisual.addCoordinateSystem(0.2);
		//scene
		// keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(scene, 255.0, 0.0, 0.0), "scene");
		keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_keypoints");
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

	//
	//  Compute Model Descriptors  PPF Space for model
	//

	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> pose_clusters;

	cout <<">>Begin match!"<<endl;
	cout <<">cluster angle thresh: "<<angle_thresh<<endl;
	cout <<">cluster distance thresh: "<<cluster_dis_thresh<<endl;
	cout <<">relativeReferencePointsNumber: "<<relativeReferencePointsNumber<<endl;
	cout <<">max_vote_thresh: "<<max_vote_thresh<<endl;
	cout <<">max_vote_percentage: "<<max_vote_percentage<<endl;
	cout <<">recompute score distance thresh: " << recopute_score_dis_thresh << endl;
	cout << ">recompute score angle thresh: " << recopute_score_ang_thresh << endl;
	cout << "num clusters per group: " << num_clusters_per_group << endl;
	//model_feature_space.setScenePntsFlag(&scene_pnt_flag);
	model_feature_space.match(scene_keypoints, scene_keyNormals, relativeReferencePointsNumber, max_vote_thresh, max_vote_percentage, angle_thresh, cluster_dis_thresh, recopute_score_dis_thresh, recopute_score_ang_thresh, num_clusters_per_group, pose_clusters);
	cout << "clusters size : " << pose_clusters.size() << endl;

	//
	//test true
	//
	Eigen::Matrix4f rt;
	//ape 0
	rt << -0.0963063, 0.994044, -0.0510079, -111.814,
		-0.573321, -0.0135081, 0.81922, -78.3622,
		0.813651, 0.10814, 0.571207, 1036.12,
		0, 0, 0, 1;
	////lamp 81
	//rt << -0.401065, -0.873948, 0.27452, -82.4209,
	//	0.775171, -0.164104, 0.610066, 11.6872,
	//	-0.488116, 0.457476, 0.743275, 792.471,
	//	0, 0, 0, 1;
	Eigen::Affine3f arf(rt);
	//true cluster
	zyk::pose_cluster true_cluster(arf,0);
	float dis_thresh = 0.3 * model_feature_space.getModelDiameter();
	cout << "true dis thresh is: " << dis_thresh << endl;
	for (int i = 0; i < pose_clusters.size(); ++i)
	{
		if (true_cluster.checkAndPutIn_test_ver(pose_clusters[i].mean_transformation, pose_clusters[i].vote_count, dis_thresh, 0.3))
			cout << "clusters info" << endl << "num trans: " << pose_clusters[i].size() << endl << "num votes: " << pose_clusters[i].vote_count << endl;

	}
	cout << "true clusters size: " << true_cluster.size() << endl;
	for (int i = 0; i < true_cluster.size(); ++i)
	{
		cout << true_cluster.transformations[i].matrix() << endl;
	}
	pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*model_keypoints, *rotated_model, rt);
	pcl::visualization::PCLVisualizer keyPointVisual("Depth2PC");
	keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 0.0, 255.0, 0.0), "scene");
	keyPointVisual.addPointCloud(rotated_model, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene, 255.0, 0.0, 0.0), "model");
	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
	keyPointVisual.spin();


	int number = pose_clusters.size();
	////if (number > 12) number = 12;
	if(show_cluster_result_)
	{
		if (!show_cluster_together_)
		{
			for (size_t i = 0; i < number; ++i)
			{
				pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
				pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
				viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");

				pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
				pcl::transformPointCloud(*model_keypoints, *rotated_model, pose_clusters[i].mean_transformation);
				cout << "-----------------------" << endl;

				for (int j = 0; j < pose_clusters[i].size(); j++)
				{
					cout << pose_clusters[i].transformations[j].matrix() << endl;
					Eigen::AngleAxisf tmp_rot(pose_clusters[i].transformations[j].rotation());
					float angle = tmp_rot.angle();
					if (angle>2 * M_PI)angle -= 2 * M_PI;
					if (angle < 0)angle += 2 * M_PI;
					Eigen::Vector3f tmp_angle_axis = angle*tmp_rot.axis();
					cout << "Rot:" << tmp_angle_axis.transpose() << endl;
				}

				cout << ">>>>>>clusters " << i << "<<<<<<<<" << endl;
				cout << "mean " << "\r\n" << pose_clusters[i].mean_transformation.matrix() << endl << endl;
				cout << "rot: " << pose_clusters[i].mean_rot.transpose() << endl;
				cout << "trans: " << pose_clusters[i].mean_trans.transpose() << endl;
				cout << "Vote: " << pose_clusters[i].vote_count << endl << "Num Of Transformations: " << pose_clusters[i].size() << endl;
				std::stringstream ss_cloud;
				ss_cloud << "instance" << i;

				pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
				viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_cloud.str());
				viewer.spin();
			}
		}
		else
		{
			while (1)
			{
				char key = 0;
				int cnt = 0;
				pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
				pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
				viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");
				cout << ">>Show vote thresh is : " << show_vote_thresh << endl;
				for (size_t i = 0; i < number; ++i)
				{
					if (pose_clusters[i].getVote() < show_vote_thresh)continue;
					pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
					pcl::transformPointCloud(*model_keypoints, *rotated_model, pose_clusters[i].mean_transformation);
					std::stringstream ss_cloud;

					cout << ">>>>>>clusters " << i << "<<<<<<<<" << endl;
					cout << "mean " << "\r\n" << pose_clusters[i].mean_transformation.matrix() << endl << endl;
					cout << "rot: " << pose_clusters[i].mean_rot.transpose() << endl;
					cout << "trans: " << pose_clusters[i].mean_trans.transpose() << endl;
					cout << "Vote: " << pose_clusters[i].vote_count << endl << "Num Of Transformations: " << pose_clusters[i].size() << endl;

					ss_cloud << "instance" << i;
					if (cnt++ >= max_show_number)break;
					pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
					viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
					viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_cloud.str());
				}
				viewer.spin();
				cout << ">>>>>>>>>>>>>>Please input key+Enter to continue" << endl;
				cout << ">> q: quit" << endl;
				cout << ">> c: change vote thresh" << endl;
				cout << ">> n: change max show number" << endl;
				scanf("  %c", &key);
				if (key == 'q')
					break;
				if (key == 'c')
				{
					cout << "PLS input new thresh(0-1):" << endl;
					scanf("  %f", &show_vote_thresh);
				}
				if (key == 'n')
				{
					cout << "PLS input new number:" << endl;
					scanf("  %d", &max_show_number);
				}
					
				if (show_vote_thresh <= 0)show_vote_thresh = 0.1;
				if (show_vote_thresh > 1)show_vote_thresh = 1;
				if (max_show_number <= 0)max_show_number = 1;
			}
		}
		
	}
  return (0);
}





void test_ppf_feature_space:: match(pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<Normal>::Ptr scene_normals, float relativeReferencePointsNumber, float max_vote_thresh, float max_vote_percentage, float angle_thresh, float first_dis_thresh, float recompute_score_dis_thresh, float recompute_score_ang_thresh, int num_clusters_per_group, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> &pose_clusters)
{
	int scene_steps = floor(1.0 / relativeReferencePointsNumber);
	if (scene_steps < 1)scene_steps = 1;

	float model_diameter = max_p[3];
	float box_radius = sqrt(model_size[0] * model_size[0] + model_size[1] * model_size[1] + model_size[2] * model_size[2]);
	first_dis_thresh *= box_radius;
	float second_dis_thresh = 0.5 * box_radius;
	recompute_score_dis_thresh *= model_res;
	//build grid to accelerate matching process
	zyk::CVoxel_grid scene_grid(model_diameter, model_diameter, model_diameter, scene);
	Eigen::Vector3i grid_div;
	scene_grid.getGridDiv(grid_div);
	vector<zyk::box*>*box_vector = scene_grid.getBox_vector();
	Eigen::Vector3i div_mul(1, grid_div(0), grid_div(0)*grid_div(1));


	int32_t num_poses_count = 0;
	//all raw poses
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> rawClusters;

	///////// info
	int32_t match_count = 0;
	int32_t match_percent = 0;
	int32_t info_step = 0.05*scene->size();

	int32_t tst_cnt1 = 0;
	int32_t tst_cnt2 = 0;
	//vote flag
	vector<int32_t>vote_flag(ppf_box_vector.size(), 0);
	/////////////////////begin iterate boxes
	for (int32_t box_index = 0; box_index < box_vector->size(); ++box_index)
	{
		/////////get box ptr
		zyk::box*p_current_point_box = box_vector->at(box_index);
		if (p_current_point_box == NULL)
			continue;
		/////////get neighboring
		vector<int32_t>neiboringBoxIndexVector;
		//for convenience, push back current index too!
		neiboringBoxIndexVector.push_back(box_index);
		zyk::getNeiboringBoxIndex3D(box_index, grid_div, neiboringBoxIndexVector);
		bool success_flag = false;
		for (int32_t cnt1 = 0; cnt1 < box_vector->at(box_index)->size(); cnt1+= (success_flag? scene_steps:1))
		{
			if ((match_count += (success_flag ? scene_steps : 1)) > info_step)
			{
				match_count = 0;
				match_percent += 5;
				cout << "Match : " << match_percent << "%." << endl;
				//cout << "tst_cnt1: " << tst_cnt1 << endl;
				//cout << "tst_cunt2: " << tst_cnt2 << endl;

			}
			success_flag = false;
			int32_t reference_pnt_index = (*p_current_point_box)[cnt1];
			//check if it is plane point
			if (scene_pnt_flag != NULL) {
				if (scene_pnt_flag->at(reference_pnt_index) == 1) {
					continue;
				}
			}
			/////////build an accumulator for this reference point
			// in matlab the row size is size+1, why?
			zyk::PPF_Accumulator acum(input_point_cloud->size(), 30);
#ifdef use_eigen
			const Eigen::Vector3f& rp = scene->at(reference_pnt_index).getVector3fMap();
			const Eigen::Vector3f& rn = scene_normals->at(reference_pnt_index).getNormalVector3fMap();
#else
			const PointType& rp = scene->at(reference_pnt_index);
			const NormalType& rn = scene_normals->at(reference_pnt_index);
#endif
			vote_flag.assign(ppf_vector.size(), 0);
			// loop through neighboring boxes to get scene pnt
			for (int32_t cnt2 = 0; cnt2 < neiboringBoxIndexVector.size(); ++cnt2)
			{
				int32_t neiboringBoxIndex = neiboringBoxIndexVector[cnt2];
				zyk::box*p_current_neiboring_point_box = box_vector->at(neiboringBoxIndex);
				if (p_current_neiboring_point_box == NULL)
					continue;
				for (int32_t cnt3 = 0; cnt3 < p_current_neiboring_point_box->size(); ++cnt3)
				{
					int32_t scene_pnt_index = (*p_current_neiboring_point_box)[cnt3];
					if (scene_pnt_index == reference_pnt_index)
						continue;
#ifdef use_eigen
					const Eigen::Vector3f& sp = scene->at(scene_pnt_index).getVector3fMap();
					const Eigen::Vector3f& sn = scene_normals->at(scene_pnt_index).getNormalVector3fMap();
#else
					const PointType& sp = scene->at(scene_pnt_index);
					const NormalType& sn = scene_normals->at(scene_pnt_index);
#endif
					if (neiboringBoxIndex != box_index)
					{
#ifdef use_eigen
						if ((sp - rp).norm() > model_diameter)
							continue;
#else
						if (dist(sp.data, rp.data, 3) > model_diameter)
							continue;
#endif
					}
					//calculate ppf
					zyk::PPF current_ppf;
					zyk::PPF_Space::computeSinglePPF(rp, rn, sp, sn, current_ppf);
					if (abs(current_ppf.ppf.f3) < 0.02 && abs(current_ppf.ppf.f1 - M_PI_2) < 0.02&&abs(current_ppf.ppf.f2 - M_PI_2) < 0.02)
						continue;
					//do hash
					int32_t ppf_box_index = getppfBoxIndex(current_ppf);
					if (ppf_box_index == -1)
						continue;
					zyk::box* current_ppf_box = ppf_box_vector.at(ppf_box_index);
					if (current_ppf_box == NULL)
						continue;
					int32_t scene_rotation_discretized = floor((current_ppf.ppf.alpha_m + M_PI) / 2 / M_PI * 32);
					if (vote_flag[ppf_box_index] & (1 << scene_rotation_discretized))
						continue;
					else
						vote_flag[ppf_box_index] |= 1 << scene_rotation_discretized;
					//now calculate alpha of current_ppf
					double current_alpha = computeAlpha(rp, rn, sp);
					//loop hash list
					for (int32_t node = 0; node < current_ppf_box->size(); ++node)
					{
						zyk::PPF& correspond_model_ppf = ppf_vector.at((*current_ppf_box)[node]);
						float alpha = correspond_model_ppf.ppf.alpha_m - current_alpha;
						if (alpha >= M_PI) alpha -= 2 * M_PI;
						if (alpha < -M_PI)alpha += 2 * M_PI;
						float alpha_bin = (alpha + M_PI) / 2 / M_PI * 30;
						int32_t middle_bin = int(alpha_bin + 0.5f);
						if (middle_bin >= 30) middle_bin = 0;
						//calculate vote_val
						//float tmp = cos(correspond_model_ppf.ppf.f3);
						//if (1 - tmp < 0.003)
						//{
						//	continue;
						//}
						float vote_val = 1 - 0.98 * fabs(cos(correspond_model_ppf.ppf.f3));
						acum.acumulator(correspond_model_ppf.first_index, middle_bin) += vote_val;
						tst_cnt2++;
					}

				}

			}
			//check the accumulator and extract poses
			Eigen::MatrixXi::Index maxRow, maxCol;
			float maxVote = acum.acumulator.maxCoeff(&maxRow, &maxCol);
			//if (max_vote_thresh > 0){
			if (maxVote < max_vote_thresh)
				continue;
			//}
			float vote_thresh = max_vote_percentage*maxVote;

			//visit the accumulator and extract poses
			for (int32_t row = 0; row < acum.acumulator.rows(); ++row)
			{
				for (int32_t col = 0; col < acum.acumulator.cols(); ++col)
				{
					if (acum.acumulator(row, col) < vote_thresh)
						continue;
					num_poses_count++;
					tst_cnt1++;
					float alpha = col / 30.0 * 2 * M_PI - M_PI;
					Eigen::Affine3f transformation;
					if (!zyk::PPF_Space::getPoseFromPPFCorresspondence(input_point_cloud->at(row), input_point_normal->at(row), scene->at(reference_pnt_index), scene_normals->at(reference_pnt_index), alpha, transformation))
						continue;
					rawClusters.push_back(zyk::pose_cluster(transformation, acum.acumulator(row, col)));
				}
			}//end of visit accumulator
			success_flag = true;
		}

	}
	std::sort(rawClusters.begin(), rawClusters.end(), zyk::pose_cluster_comp);
	cout << "all poses number : " << rawClusters.size() << endl;
	////////////Now do clustering
	// raw cluster first
	for (int32_t i = 0; i < rawClusters.size(); i++)
	{
		int result = 0;
		for (int32_t j = 0; j < pose_clusters.size(); j++)
		{
			if (pose_clusters[j].checkAndPutIn(rawClusters[i].transformations[0], rawClusters[i].vote_count, first_dis_thresh, angle_thresh))
			{
				//if(++result>=max_clusters_per_pose_can_be_in)
				result = 1;
				break;
			}
		}
		if (!result)
			pose_clusters.push_back(rawClusters[i]);
	}
	std::sort(pose_clusters.begin(), pose_clusters.end(), zyk::pose_cluster_comp);

	//////recompute score
	if (recompute_score_dis_thresh > 0)
	{
		cout << "Now recompute scores, distance thresh is: " << recompute_score_dis_thresh << endl;
		recomputeClusterScore(scene_grid, *scene_normals, recompute_score_dis_thresh, recompute_score_ang_thresh, pose_clusters);
		std::sort(pose_clusters.begin(), pose_clusters.end(), zyk::pose_cluster_comp);
	}

	// second cluster by distance
	// group those clear clusters together and rank them
	if (num_clusters_per_group > 0) {
		cout << "Now do grouping, number of clusters per group is : " << num_clusters_per_group << endl;
		vector<vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>>> groups;
		for (int32_t i = 0; i < pose_clusters.size(); i++)
		{
			int result = 0;
			for (int32_t j = 0; j < groups.size(); j++)
			{
				if ((groups[j][0].mean_trans - pose_clusters[i].mean_trans).norm() < second_dis_thresh)
				{
					groups[j].push_back(pose_clusters[i]);
					result = 1;
					break;
				}
			}
			if (!result)
			{
				vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> tmp_clusters;
				tmp_clusters.push_back(pose_clusters[i]);
				groups.push_back(tmp_clusters);
			}
		}

		cout << "group size is: " << groups.size() << endl;
		pose_clusters.clear();
		for (int i = 0; i < groups.size(); i++)
		{
			for (int j = 0; j < min(num_clusters_per_group, int(groups[i].size())); ++j)
			{
				pose_clusters.push_back(groups[i][j]);
			}

		}
	}
}