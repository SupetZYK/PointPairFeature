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
bool use_cloud_resolution_ (false);
bool spread_ppf_switch_ (false);
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
  std::cout << "     -h:                    Show this help." << std::endl;
  std::cout << "     -k:                    Show used keypoints." << std::endl;
  std::cout << "     -c:                    Show used correspondences." << std::endl;
  std::cout << "     -r:                    Use the model cloud resolution and multiply to get scene sampling resolution" << std::endl;
  std::cout << "     --sc:                  Show cluster results" << std::endl;
  std::cout << "     --st:                  Show cluster results together, only when 'sc' is input" << std::endl;
  std::cout << "     --in:					Use existing normal files" << std::endl;
  std::cout << "     --sppf:				Spread discretized ppf" << std::endl;
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
  //if (pcl::console::find_switch (argc, argv, "--ply"))
  //{
    //use_ply_filetype_ = true;
  //}
	//if (pcl::console::find_switch(argc, argv, "--ppfs"))
	//{
		//use_ppfs_file_ = true;
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
	//}
  //if (pcl::console::find_switch(argc, argv, "--iss"))
  //{
	  //use_iss_ = true;
	  //pcl::console::parse_argument(argc, argv, "--iss_sr", iss_sr_);
	  //pcl::console::parse_argument(argc, argv, "--iss_nr", iss_nr_);
  //}

  //Model & scene filenames
	filenames.clear()	;
  //if(use_ply_filetype_==false)
	  //filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  //else
	 filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
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
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--angle_thresh", angle_thresh);
  pcl::console::parse_argument (argc, argv, "--dis_thresh", cluster_dis_thresh);
  pcl::console::parse_argument (argc, argv, "--para_1", relativeReferencePointsNumber);
  pcl::console::parse_argument (argc, argv, "--para_2", max_vote_thresh);
  pcl::console::parse_argument (argc, argv, "--para_3", max_vote_percentage);
  pcl::console::parse_argument(argc, argv, "--para_4", num_clusters_per_group);
  pcl::console::parse_argument(argc, argv, "--re_d", recopute_score_dis_thresh);
  pcl::console::parse_argument(argc, argv, "--re_a", recopute_score_ang_thresh);
  pcl::console::parse_argument(argc, argv, "--show_thresh", show_vote_thresh);
  pcl::console::parse_argument (argc, argv, "--icp_thresh", icp_dis_thresh);
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


	///std::string fileformat;
	//if (use_ply_filetype_)
	//{
		//fileformat = "ply";
	//}
	//else
	//{
		//fileformat = "pcd";
	//}
	if (use_existing_normal_data_)
	{
		if (!readPointCloud(scene_filename_, "ply", scene,scene_normals))
		{
			return(-1);
		}
	}
	else
	{
		if (!readPointCloud(scene_filename_, "ply", scene))
		{
			return(-1);
		}
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
	if (use_existing_normal_data_)
		sampled_index_ptr = uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints, scene_keyNormals);
	else
		sampled_index_ptr = uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;

	//
	//  Compute Normals
	//

	if (!use_existing_normal_data_)
	{
		if (!use_mls_)
		{
			pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
			norm_est.setIndices(sampled_index_ptr);
			norm_est.setKSearch(20);
			//norm_est.setRadiusSearch(scene_ss_);
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
			mls.setSearchRadius(0.5*scene_ss_);
			mls.process(*scene_keypoints);

			scene_keyNormals = mls.getNormals();
			for (size_t i = 0; i < scene_keyNormals->size(); ++i)
			{
				flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
			}
		}

		cout << "Normal compute complete£¡" << endl;
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
	model_feature_space.match(scene_keypoints, scene_keyNormals, spread_ppf_switch_, relativeReferencePointsNumber, max_vote_thresh, max_vote_percentage, angle_thresh, cluster_dis_thresh, recopute_score_dis_thresh, recopute_score_ang_thresh, num_clusters_per_group, pose_clusters);
	cout << "clusters size : " << pose_clusters.size() << endl;

	//
	//  Visualization
	//

	//pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
	//pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
	//viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");

	//pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
	//pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

	//if (show_correspondences_ || show_keypoints_)
	//{
	//  //  We are translating the model so that it doesn't end in the middle of the scene representation
	//  pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
	//  pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

	//  pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
	//  viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
	//}

	//if (show_keypoints_)
	//{
	//  pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
	//  viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
	//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

	//  pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
	//  viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
	//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	//}

	//test viewer
	//{
	//	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	//	pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
	//	viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");

	//	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	//	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	//	pcl::PointCloud<PointType>::Ptr rotated_model1(new pcl::PointCloud<PointType>());
	//	pcl::PointCloud<PointType>::Ptr rotated_model2(new pcl::PointCloud<PointType>());
	//	Eigen::Affine3f t1 = Eigen::Affine3f::Identity();
	//	t1.translate(Eigen::Vector3f(-74, 86, 1431));
	//	cout << t1.matrix() << endl;
	//	Eigen::Affine3f t2 = Eigen::Affine3f::Identity();
	//	t2.translate(Eigen::Vector3f(-152, 46, 1422));
	//	pcl::transformPointCloud(*model, *rotated_model1, t1);
	//	pcl::transformPointCloud(*model, *rotated_model2, t2);
	//	pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler1(rotated_model1, 255, 0, 0);
	//	pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler2(rotated_model2, 255, 0, 0);
	//	viewer.addPointCloud(rotated_model1, rotated_model_color_handler1, "r1");
	//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "r1");
	//	viewer.addPointCloud(rotated_model2, rotated_model_color_handler2, "r2");
	//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "r2");
	//	viewer.spin();
	//}


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
					if (angle>2 * M_PI)angle -= 2 * M_PI;
					if (angle < 0)angle += 2 * M_PI;
					Eigen::Vector3f tmp_angle_axis = angle*tmp_rot.axis();
					cout << "Rot:" << tmp_angle_axis.transpose() << endl;
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
	//for (int32_t i = 0; i < pose_clusters.size(); i++)
	//	delete pose_clusters[i];
  return (0);
}
