#include <util_pcl.h>
#include "PPFFeature.h"
#include "pose_cluster.h"
//pcl
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
using namespace pcl;
//std::string model_filename_;
std::string model_ppfs_filename_;
std::string scene_filename_;
//std::string model_normal_filename_;
std::string scene_normal_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool show_cluster_result_ (false);
bool show_cluster_together_(false);
bool spread_ppf_switch_ (false);
bool two_ball_switch_ (false);
//bool use_ply_filetype_ (false);
//bool use_iss_(false);
bool use_existing_normal_data_ (false);

int icp_type  (-1);
int mls_order (2);
//bool use_ppfs_file_  (false);
//float model_ss_ (0.01f);
float scene_ds_(-1.0f);
float angle_thresh=M_PI/15;
float cluster_dis_thresh = 0.1;
float recopute_score_dis_thresh = 0.05;
float recopute_score_ang_thresh = -1;
float relativeReferencePointsNumber = 0.2;
float icp_dis_thresh=0.3;
float max_vote_thresh (0.5);
float max_vote_percentage (0.8);
float second_distance_thresh(0.5);
int num_clusters_per_group = 2;
float show_vote_thresh (0.3);
int max_show_number (10);
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
  std::cout << "Options:" << std::endl;
  std::cout << "     --mod [string]:        Input the model file name." << std::endl;
  std::cout << "     --tar [string]:        Input the target cloud file name." << std::endl;
  std::cout << "     -h:                    Show this help." << std::endl;
  std::cout << "     -k:                    Show used keypoints." << std::endl;
  std::cout << "     -c:                    Show used correspondences." << std::endl;
  std::cout << "     --sc:                  Show cluster results" << std::endl;
  std::cout << "     --st:                  Show cluster results together, only when 'sc' is input" << std::endl;
  std::cout << "     --in:					Use existing normal files" << std::endl;
  std::cout << "     --tb:					Two ball switch" << std::endl;
  std::cout << "     --sppf:				Spread discretized ppf" << std::endl;
  std::cout << "     --mls val:				Moving least squares order, set to 0 to disable" << std::endl;
  std::cout << "     --scene_ds val:        Scene uniform sampling radius (default same as model)" << std::endl;
  std::cout << "     --angle_thresh val:    angle thresh when do ppf clustering" << std::endl;
  std::cout << "     --dis_thresh val:		first distance thresh(relative to radius)" << std::endl;
  std::cout << "     --para_1 val:			relative reference points number in ppf matching"<< std::endl;
  std::cout << "     --para_2 val:			max vote thresh, relative to the number of points in the current box"<< std::endl;
  std::cout << "     --para_3 val:			if the vote in the accumulator is greater than a certain thresh, then the instance is considered, this is the ratio of thresh to max_vote" << std::endl;
  std::cout << "     --para_4 val:			Number of clusters per group.set to '-1' to close group" << std::endl;
  std::cout << "     --re_d val:			recompute score distance thresh, relative to model resolution.Set to '-1' to disable recompute score" << std::endl;
  std::cout << "     --re_a val:			recompute score angle thresh, Set to '-1' to disable using angle thresh" << std::endl;
  std::cout << "     --se_d val:			second distance thresh, for overlapping elimination(default 0.5)" << std::endl;
  std::cout << "     --show_thresh val:     the clusters whose vote is greater than this will be displayed" << std::endl;
  std::cout << "     --icp val:				icp type(default -1 disabled)" << std::endl;
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
  //if (pcl::console::find_switch (argc, argv, "-r"))
  //{
  //  use_cloud_resolution_ = true;
  //}
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
  //if (pcl::console::find_switch(argc, argv, "--mls"))
  //{
	 // use_mls_ = true;
  //}
  if (pcl::console::find_switch(argc, argv, "--tb"))
  {
	  two_ball_switch_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--sppf"))
  {
	  spread_ppf_switch_ = true;
  }
#if 0 //older prompt style
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ppfs");
  if (filenames.size() < 1)
  {
	  std::cout << "template filenames missing.\n";
	  showHelp(argv[0]);
	  exit(-1);
  }
  model_ppfs_filename_ = argv[filenames[0]];
  cout << "Input ppfs file: " << model_ppfs_filename_ << endl;
  //Model & scene filenames
	filenames.clear();
	 filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	if (filenames.size () < 1)
	{
	 std::cout << "Filenames missing.\n";
	 showHelp (argv[0]);
	 exit (-1);
	}
	scene_filename_ = argv[filenames[0]];
	cout << "Input scene file: " << scene_filename_ << endl;
#else // new stype prompt style
  pcl::console::parse_argument(argc, argv, "--mod", model_ppfs_filename_);
  pcl::console::parse_argument(argc, argv, "--tar", scene_filename_);
  if (model_ppfs_filename_.empty() || scene_filename_.empty()) {
	  std::cout << "Two few files input!" << std::endl;
	  system("pause");
	  exit(-1);
  }
#endif
  //General parameters
  //pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ds", scene_ds_);
  pcl::console::parse_argument (argc, argv, "--angle_thresh", angle_thresh);
  pcl::console::parse_argument (argc, argv, "--dis_thresh", cluster_dis_thresh);
  pcl::console::parse_argument (argc, argv, "--para_1", relativeReferencePointsNumber);
  pcl::console::parse_argument (argc, argv, "--para_2", max_vote_thresh);
  pcl::console::parse_argument (argc, argv, "--para_3", max_vote_percentage);
  pcl::console::parse_argument (argc, argv, "--para_4", num_clusters_per_group);
  pcl::console::parse_argument (argc, argv, "--re_d", recopute_score_dis_thresh);
  pcl::console::parse_argument (argc, argv, "--re_a", recopute_score_ang_thresh);
  pcl::console::parse_argument (argc, argv, "--se_d", second_distance_thresh);
  pcl::console::parse_argument (argc, argv, "--show_thresh", show_vote_thresh);
  pcl::console::parse_argument (argc, argv, "--icp_thresh", icp_dis_thresh);
  pcl::console::parse_argument(argc, argv, "--mls", mls_order);
  pcl::console::parse_argument(argc, argv, "--icp", icp_type);
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


	if (use_existing_normal_data_) {
		if (!zyk::readPointCloud(scene_filename_, scene, scene_normals))
			return(-1);
	}
	else {
		if (!zyk::readPointCloud(scene_filename_, scene))
			return(-1);
	}
	cout<<"Read finish"<<endl;
	//
	// some global variables
	//
	zyk::PPF_Space model_feature_space;
	pcl::PointCloud<NormalType>::Ptr model_keyNormals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());

	//
	//  Set up resolution invariance
	//

	//double max_coord[3];
	//double min_coord[3];
	//if (use_ppfs_file_)
	//{
		if (!model_feature_space.load(model_ppfs_filename_))
		{
			std::cout << "reading ppfs file fail!" << std::endl;
			return -1;
		}
		cout<<"ppfs file load finish"<<endl;
		//CFile fileload;
		//fileload.Open(model_ppfs_filename_.c_str(), CFile::modeRead);
		//CArchive ar(&fileload, CArchive::load);
		//model_feature_space.Serialize(ar);
		//ar.Close();
		//fileload.Close();
		model_feature_space.getModelPointCloud(model_keypoints);
		model_feature_space.getPointNormalCloud(model_keyNormals);
	//}

	float model_length=model_feature_space.model_size[0];
	float model_width=model_feature_space.model_size[1];
	float model_height=model_feature_space.model_size[2];

	std::cout << "Model resolution: " << model_feature_space.model_res << std::endl;
	std::cout << "Model length: " << model_length << std::endl;
	std::cout << "Model width: " << model_width << std::endl;
	std::cout << "Model height: " << model_height << std::endl;

	//compute original scene resolution
	double original_scene_res = zyk::computeCloudResolution(scene);
	std::cout << "Original scene resolution is: " << original_scene_res << std::endl;
	double scene_ss_ = 1;
	if (scene_ds_ < 0)
		scene_ss_ = model_feature_space.model_res;
	else
		scene_ss_ *= (scene_ds_/model_feature_space.getSampleRatio())*model_feature_space.model_res;
	cout << "Scene sampling resolution is: " << scene_ss_ << endl;
	
	//
	//  Downsample Clouds to Extract keypoints
	//
	pcl::IndicesPtr sampled_index_ptr;
	if (use_existing_normal_data_)
		sampled_index_ptr = zyk::uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints, scene_keyNormals);
	else
		sampled_index_ptr = zyk::uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;

	//
	//  Compute Normals
	//

	if (!use_existing_normal_data_)
	{
		if (mls_order==0)
		{
			pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
			norm_est.setIndices(sampled_index_ptr);
			//norm_est.setKSearch(20);
			norm_est.setRadiusSearch(model_feature_space.model_res);
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
			mls.setPolynomialOrder(mls_order);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(2*model_feature_space.model_res);
			//mls.setSearchRadius(0.7*model_feature_space.model_res);
			mls.process(*scene_keypoints);
			scene_keyNormals = mls.getNormals();
			for (size_t i = 0; i < scene_keyNormals->size(); ++i)
			{
				flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
			}
		}

		cout << "Normal compute complete！" << endl;
	}
	else
	{
		cout<<"Using existing normals"<<endl;
	}


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

	//
	//  Compute Model Descriptors  PPF Space for model
	//

	
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > pose_clusters;
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > refined_pose_clusters;
	cout <<">>Begin match!"<<endl;
	cout <<">cluster angle thresh: "<<angle_thresh<<endl;
	cout <<">cluster distance thresh: "<<cluster_dis_thresh<<endl;
	cout <<">relativeReferencePointsNumber: "<<relativeReferencePointsNumber<<endl;
	cout <<">max_vote_thresh: "<<max_vote_thresh<<endl;
	cout <<">max_vote_percentage: "<<max_vote_percentage<<endl;
	cout <<">recompute score distance thresh: " << recopute_score_dis_thresh << endl;
	cout << ">recompute score angle thresh: " << recopute_score_ang_thresh << endl;
	cout << "second distance thresh: " << second_distance_thresh << endl;
	cout << "num clusters per group: " << num_clusters_per_group << endl;
	model_feature_space.match(scene_keypoints, scene_keyNormals, spread_ppf_switch_, two_ball_switch_, relativeReferencePointsNumber, max_vote_thresh, max_vote_percentage, angle_thresh, cluster_dis_thresh, recopute_score_dis_thresh, recopute_score_ang_thresh, second_distance_thresh, num_clusters_per_group, pose_clusters);
	
	if(num_clusters_per_group<0)
	{
		//gather some information for paper, 
		//use pipe scene, 0.05, no group
		vector<int> correct_pose(100,0);
		vector<int> incorrect_pose(100,0);
		float correct_vote=0.0, incorrect_vote=0.0;
		for (int i = 0; i < pose_clusters.size(); ++i) {
			int vote = pose_clusters[i].old_vote_count;
			if (pose_clusters[i].vote_count > show_vote_thresh) {
				correct_pose[vote]+=pose_clusters[i].size();
				correct_vote += pose_clusters[i].old_vote_count;
			}
			else
			{
				incorrect_pose[vote]+=pose_clusters[i].size();
				incorrect_vote += pose_clusters[i].old_vote_count;
			}
		}
		//write to txt file
		ofstream ofs;
		ofs.open("res1.txt");
		for (int i = 0; i < 100; ++i) {
			ofs << correct_pose[i] << "  " << incorrect_pose[i] << endl;
		}
		ofs << "correct vote: " << correct_vote << endl;
		ofs << "incorrect vote: " << incorrect_vote << endl;
		ofs.close();
	}
	cout << "clusters size : " << pose_clusters.size() << endl;

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

				//pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
				//pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

				pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
				pcl::transformPointCloud(*model_keypoints, *rotated_model, pose_clusters[i].mean_transformation);
				cout << "-----------------------" << endl;
				//vector<Eigen::Vector3f> tmp_rot_vec;
				//Eigen::Vector3f tmp_mean_rot_axis;
				//tmp_mean_rot_axis.setZero();
				for (int j = 0; j < pose_clusters[i].size(); j++)
				{
					cout << pose_clusters[i].transformations[j].matrix() << endl;
					Eigen::AngleAxisf tmp_rot(pose_clusters[i].transformations[j].rotation());
					float angle = tmp_rot.angle();
					Eigen::Vector3f axis = tmp_rot.axis();
					std::cout << "ïnitial angle: " << angle << " Initial rot: " << axis.transpose() << std::endl;

					if (fabs(angle - pose_clusters[i].first_angle) > 2*angle_thresh) {
						angle = 2 * M_PI - angle;
					}
					if (axis.dot(pose_clusters[i].first_axis) < 0)
						axis = -axis;
					Eigen::Vector3f tmp_angle_axis = angle*axis;
					cout << "Rot:" << tmp_angle_axis.transpose() << endl;
					Eigen::Quaternionf q(pose_clusters[i].transformations[j].rotation());
					cout << "Quatanion: " << q.w() <<" "<<q.vec().transpose()<< endl;
					//tmp_mean_rot_axis+=tmp_angle_axis;
				}

				//Eigen::Vector3f tmp_mean_rot_axis;
				//for(int j=0;j<tmp_rot_vec.size();j++)
				//{
				//tmp_mean_rot_axis+=tmp_rot_vec[j];
				//}
				//tmp_mean_rot_axis/=pose_clusters[i].size();
				//cout<<"tmp_mean_rot_axis: "<<tmp_mean_rot_axis.transpose()<<endl;

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
				//if (show_correspondences_)
				//{
				//  for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
				//  {
				//    std::stringstream ss_line;
				//    ss_line << "correspondence_line" << i << "_" << j;
				//    PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
				//    PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

				//    //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				//    viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
				//  }
				//}
				//cout << "contain?: " << viewer.contains(ss_cloud.str()) << endl;
				viewer.spin();
				//viewer.removePointCloud(ss_cloud.str());
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
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud");
				cout << ">>Show vote thresh is : " << show_vote_thresh << endl;

				//
				//check whether icp is needed
				//
				if (icp_type==1) {
					refined_pose_clusters.clear();
					model_feature_space.ICP_Refine(scene, pose_clusters, refined_pose_clusters, max_show_number, original_scene_res);
				}
				else if (icp_type == 2) {
					refined_pose_clusters.clear();
					model_feature_space.ICP_Refine2_0(scene, pose_clusters, refined_pose_clusters, max_show_number, original_scene_res, 6);
				}
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


					if (icp_type>0) {
						pcl::PointCloud<PointType>::Ptr fine_rotated_model(new pcl::PointCloud<PointType>());
						pcl::transformPointCloud(*model_keypoints, *fine_rotated_model, refined_pose_clusters[i].mean_transformation);
						pcl::visualization::PointCloudColorHandlerCustom<PointType> fine_rotated_model_color_handler(fine_rotated_model, 0, 0, 255);
						viewer.addPointCloud(fine_rotated_model, fine_rotated_model_color_handler, ss_cloud.str()+"icp");
						viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_cloud.str()+"icp");
						cout << "ICP_rot: " << refined_pose_clusters[i].mean_rot.transpose() << endl;
						cout << "ICP_trans: " << refined_pose_clusters[i].mean_trans.transpose() << endl;
						std::cout << "ICP score: " << refined_pose_clusters[i].vote_count << std::endl;
					}
				}
				viewer.spin();
				cout << ">>>>>>>>>>>>>>Please input key+Enter to continue" << endl;
				cout << ">> q: quit" << endl;
				cout << ">> c: change vote thresh" << endl;
				cout << ">> n: change max show number" << endl;
				cout << ">> i: enable icp" << endl;
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
				if (key == 'i')
				{
					cout << "PLS input new icp type(1,2):" << endl;
					scanf("  %d", &icp_type);
				}
				std::wcout << "icp type changed to " << icp_type << std::endl;
				if (show_vote_thresh <= 0)show_vote_thresh = 0.1;
				if (show_vote_thresh > 1)show_vote_thresh = 1;
				if (max_show_number <= 0)max_show_number = 1;
			}
		}
		
	}

	////
	////load a model for icp
	////
	//pcl::PointCloud<PointType>::Ptr model_for_icp (new pcl::PointCloud<PointType>);
	//pcl::io::loadPLYFile("../datafile/white_pipe_changed.ply",*model_for_icp);
	////
	////icp
	////
	//cout<<"begin icp"<<endl;
	//pcl::IterativeClosestPoint<PointType,PointType> icp;
	//icp.setTransformationEpsilon(1e-10);
	//icp.setEuclideanFitnessEpsilon(0.01);
	//icp.setMaximumIterations(300);
	//
	//for(int i=0;i<number;i++)
	//{
	//	
	//	pcl::PointCloud<PointType>::Ptr out (new pcl::PointCloud<PointType>);
	//	pcl::PointCloud<PointType>::Ptr out2 (new pcl::PointCloud<PointType>);
	//	pcl::copyPointCloud(*model_for_icp,*out);
	//	icp.setMaxCorrespondenceDistance(icp_dis_thresh);
	//	icp.setInputSource(out);
	//	icp.setInputTarget(scene);
	//	icp.align(*out2,pose_clusters[i].mean_transformation.matrix());	
	//	cout<<"icp converged? :"<<icp.hasConverged()<<endl;
	//	cout<<"icp score: "<<icp.getFitnessScore()<<endl;
	//	out->clear();
	//	pcl::copyPointCloud(*out2,*out);
	//	int cnt=30u;
	//	while(cnt--)
	//	{
	//		icp.setInputSource(out);
	//		icp.setInputTarget(scene);
	//		icp.align(*out2);
	//		static float last_score=10000000;
	//		float this_score=icp.getFitnessScore();
	//		
	//		cout<<"icp converged? :"<<icp.hasConverged()<<endl;
	//		cout<<"icp score: "<<this_score<<endl;
	//		out->clear();
	//		pcl::copyPointCloud(*out2,*out);

	//		out2->clear();
	//		//if(this_score>last_score)
	//			//break;
	//		if(abs(this_score-last_score)<1e-4)
	//		{
	//			//if(icp.getMaxCorrespondenceDistance()>21)
	//			//{
	//				//icp.setMaxCorrespondenceDistance(20);
	//				//last_score=10000000;
	//				//cout<<"change to 20"<<endl;
	//				//continue;
	//			//}
	//			//else if(icp.getMaxCorrespondenceDistance()>11)
	//			//{
	//				//icp.setMaxCorrespondenceDistance(10);
	//				//last_score=10000000;
	//				//cout<<"change to 10"<<endl;
	//				//continue;
	//			//}
	//			//else if(icp.getMaxCorrespondenceDistance()>2)
	//			//{
	//				//icp.setMaxCorrespondenceDistance(1);
	//				//last_score=10000000;
	//				//cout<<"change to 1"<<endl;
	//				//continue;
	//			//}
	//			//else if(icp.getMaxCorrespondenceDistance()>0.8)
	//			//{
	//				//icp.setMaxCorrespondenceDistance(0.6);
	//				//last_score=10000000;
	//				//cout<<"change to 0.6"<<endl;
	//				//continue;
	//			//}

	//			break;
	//		}
	//		last_score=this_score;
	//	
	//	}

	//	cout<<">>>>>>>>>>ICP Finish!<<<<<<<<<<<"<<endl;	
	//	pcl::visualization::PCLVisualizer viewer("icp view");
	//	pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
	//	viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");

	//	//pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	//	//pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	//	std::stringstream ss_cloud;
	//	ss_cloud << "instance" << i;
	//	pcl::visualization::PointCloudColorHandlerCustom<PointType> out_model_handle_custom (out, 255, 0, 0);
	//	viewer.addPointCloud (out, out_model_handle_custom, ss_cloud.str ());
	//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_cloud.str());
	//	viewer.spin();
	//}
	//for (int i = 0; i < pose_clusters.size(); i++)
	//	delete pose_clusters[i];
  return (0);
}
