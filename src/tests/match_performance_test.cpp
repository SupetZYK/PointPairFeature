#include "common.h"
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"

#include <pcl/features/normal_3d_omp.h>
//#include <pcl/visualization/pcl_visualizer.h>
//std::string model_filename_;
std::string model_ppfs_filename_ = "../../../datafile/abcdefg.ppfs";
std::string scene_filename_ ="../../../datafile/whitePointCloud_smoothed.ply";
std::string scene_normal_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_cluster_result_ (false);
bool use_cloud_resolution_ (true);

bool use_existing_normal_data_ (true);

float scene_ss_ (6.5);
float angle_thresh=0.2;
float first_dis_thresh = 0.1;
float second_dis_thresh =0.5;
float relativeReferencePointsNumber=0.2;
int max_clusters_per_pose_can_be_in = 1;
float icp_dis_thresh=0.3;
float max_vote_thresh (0.5);
float max_vote_percentage (0.95);

int
main(int argc, char *argv[])
{
	//pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());

//	if (use_existing_normal_data_)
//	{
//		if (!readPointCloud(scene_filename_, "ply", scene,scene_normals))
//		{
//			return(-1);
//		}
//	}
//	else
//	{
//		if (!readPointCloud(scene_filename_, "ply", scene))
//		{
//			return(-1);
//		}
//	}
//	cout<<"Read finish"<<endl;
//	//
//	// some global variables
//	//
//	zyk::PPF_Space model_feature_space;
//	pcl::PointCloud<NormalType>::Ptr model_keyNormals(new pcl::PointCloud<NormalType>());
//	pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());
//
//	if (!model_feature_space.load(model_ppfs_filename_))
//	{
//		std::cout << "reading ppfs file fail!" << std::endl;
//		return -1;
//	}
//	cout<<"ppfs file load finish"<<endl;
//
//	model_feature_space.getPointCloud(model_keypoints);
//	model_feature_space.getPointNormalCloud(model_keyNormals);
//
//	float resolution = static_cast<float> (computeCloudResolution(scene));
//	cout<<"Scene resolution : "<<resolution<<endl;
//	if (use_cloud_resolution_)
//	{
//		if (resolution != 0.0f)
//		{
//			//model_ss_ *= resolution;
//			scene_ss_ *= resolution;
//		}
//
//		//std::cout << "Model resolution:       " << resolution << std::endl;
//		//std::cout << "Model sampling size:    " << model_ss_ << std::endl;
//		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
//	}
//
//	//
//	//  Compute Normals
//	//
//	
//	if (!use_existing_normal_data_)
//	{
//		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
//		norm_est.setKSearch(10);
//		norm_est.setInputCloud(scene);
//		norm_est.compute(*scene_normals);
//		//norm_est.setInputCloud(model);
//		//norm_est.setViewPoint(trans(0) + model_approximate_center(0), trans(1) + model_approximate_center(1), trans(2) + model_approximate_center(2));
//		//norm_est.compute(*model_normals);
//		cout << "Normal compute complete£¡" << endl;
//	}
//	else
//	{
//		cout<<"Using existing normals"<<endl;
//	}
//
//
//
//	//
//	//  Downsample Clouds to Extract keypoints
//	//
//
//	//if (!use_ppfs_file_)
//		//uniformDownSamplePointAndNormal(model, model_normals, model_ss_, model_keypoints, model_keyNormals);
//	//std::cout << "Model total points: " << model->size() << "; Selected downsample: " << model_keypoints->size() << std::endl;
//
//	uniformDownSamplePointAndNormal(scene, scene_normals, scene_ss_, scene_keypoints, scene_keyNormals);
//	std::cout << "Scene total points: " << scene->size() << "; Selected downsample: " << scene_keypoints->size() << std::endl;
//
//
//	//
//	//visualize keypoints
//	//
//
////	if (show_keypoints_)
////	{
////		pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
////		//keyPointVisual.addCoordinateSystem(0.2);
////		//scene
////		// keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(scene, 255.0, 0.0, 0.0), "scene");
////		keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
////		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
////		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
//////		keyPointVisual.addPointCloudNormals<PointType, NormalType>(model_keypoints, model_keyNormals, 1, 10, "model_normals");
////		keyPointVisual.addPointCloudNormals<PointType, NormalType>(scene_keypoints, scene_keyNormals, 1, 10, "scene_normals");
////		//model
////		// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
////		//keyPointVisual.addPointCloud(model_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model_keypoints, 0.0, 0.0, 255.0), "model_keypoints");
////		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
////		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
////		keyPointVisual.spin();
////	}
//
//	//
//	//  Compute Model Descriptors  PPF Space for model
//	//
//
//	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> pose_clusters;
//
//	cout<<">>Begin match!"<<endl;
//	cout<<">angle_thresh: "<<angle_thresh<<endl;
//	cout<<">first_dis_thresh: "<<first_dis_thresh<<endl;
//	cout<<">second_dis_thresh: "<<second_dis_thresh<<endl;
//	cout<<">max_clusters_per_pose_can_be_in: "<<max_clusters_per_pose_can_be_in<<endl;
//	cout<<">relativeReferencePointsNumber: "<<relativeReferencePointsNumber<<endl;
//	cout<<">max_vote_thresh: "<<max_vote_thresh<<endl;
//	cout<<">max_vote_percentage: "<<max_vote_percentage<<endl;
//
//	model_feature_space.match(scene_keypoints, scene_keyNormals, relativeReferencePointsNumber,max_vote_thresh,max_vote_percentage, angle_thresh,first_dis_thresh,second_dis_thresh,max_clusters_per_pose_can_be_in,pose_clusters);
//
//	cout << "clusters size : " << pose_clusters.size() << endl;




	//int number = pose_clusters.size();
	//if(show_cluster_result_)
	//{
	//	for (size_t i = 0; i < number; ++i)
	//  {
	//		pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	//		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
	//		viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");

	//		pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	//		pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	//		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
	//		pcl::transformPointCloud(*model_keypoints, *rotated_model, pose_clusters[i].mean_transformation);
	//		cout << "-----------------------" << endl;
	//		//vector<Eigen::Vector3f> tmp_rot_vec;
	//		//Eigen::Vector3f tmp_mean_rot_axis;
	//		//tmp_mean_rot_axis.setZero();
	//		for (int j = 0; j < pose_clusters[i].size(); j++)
	//		{
	//			cout<< pose_clusters[i].transformations[j].matrix() << endl;
	//			Eigen::AngleAxisf tmp_rot(pose_clusters[i].transformations[j].rotation());
	//			float angle=tmp_rot.angle();
	//			if(angle>2*M_PI)angle-=2*M_PI;
	//			if(angle<0)angle+=2*M_PI;
	//			Eigen::Vector3f tmp_angle_axis=angle*tmp_rot.axis();
	//			cout<<"Rot:"<<tmp_angle_axis.transpose()<<endl; 
	//			//tmp_mean_rot_axis+=tmp_angle_axis;
	//		}

	//		//Eigen::Vector3f tmp_mean_rot_axis;
	//		//for(int j=0;j<tmp_rot_vec.size();j++)
	//		//{
	//			//tmp_mean_rot_axis+=tmp_rot_vec[j];
	//		//}
	//		//tmp_mean_rot_axis/=pose_clusters[i].size();
	//		//cout<<"tmp_mean_rot_axis: "<<tmp_mean_rot_axis.transpose()<<endl;

	//	cout << ">>>>>>clusters " << i << "<<<<<<<<" << endl;
	//	cout << "mean " << "\r\n" << pose_clusters[i].mean_transformation.matrix() << endl << endl;
	//	cout << "rot: " << pose_clusters[i].mean_rot.transpose() << endl;
	//	cout << "trans: " << pose_clusters[i].mean_trans.transpose() << endl;
	//	cout << "Vote: " << pose_clusters[i].vote_count << endl << "Num Of Transformations: " << pose_clusters[i].size() << endl;
	//	std::stringstream ss_cloud;
	//	ss_cloud << "instance" << i;

	//	pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
	//	viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
	//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_cloud.str());
	//	//if (show_correspondences_)
	//	//{
	//	//  for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
	//	//  {
	//	//    std::stringstream ss_line;
	//	//    ss_line << "correspondence_line" << i << "_" << j;
	//	//    PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
	//	//    PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

	//	//    //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
	//	//    viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
	//	//  }
	//	//}
	//		//cout << "contain?: " << viewer.contains(ss_cloud.str()) << endl;
	//		viewer.spin();
	//		//viewer.removePointCloud(ss_cloud.str());
	//  }
	//}
	return (0);
}
