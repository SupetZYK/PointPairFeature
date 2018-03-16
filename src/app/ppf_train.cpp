#include "util_pcl.h"
#include <util.h>
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"
#include "SmartSampling.hpp"

#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
using namespace pcl;
std::string model_filename_;
std::string save_filename_;

//Algorithm params
//bool use_cloud_resolution_  (false);
//bool use_ply_filetype_  (false);
bool use_existing_normal_data_  (false);
//bool x_centrosymmetric_  (false);
//bool y_centrosymmetric_  (false);
//bool z_centrosymmetric_  (false);
bool save_sampled_cloud_ (false);
bool normal_reorient_switch_ (false);
bool smart_sample_border_ (false);
bool show_original_model_ (false);
bool change_center_switch_(false);
bool use_mls_ (false);
float ang_thresh (1);
float model_ds_ (0.05f);
float plane_ds_ (0.05f);
float curvature_radius_ (2.0f);
int angle_div_ (20);
int distance_div_ (25);
void showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: "  << " ppf_train [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:						Show this help." << std::endl;
  std::cout << "     --mod val:			Path of the model cloud." << std::endl;
  std::cout << "     --out val:			Path of the output .ppfs file(if not specified, same as model)" << std::endl;
  //std::cout << "     -r:					Compute the model cloud resolution and multiply" << std::endl;
	std::cout << "     -w:						write the sampled model" << std::endl;
  //std::cout << "     --ply:				Use .poly as input cloud. Default is .pcd" << std::endl;
	std::cout << "     --rn:					Reorient switch!" << std::endl;
	std::cout << "     --cc:					Change Center switch!" << std::endl;
	std::cout << "     --so:					show original model" << std::endl;
	std::cout << "     --in:					Use existing normal files" << std::endl;
	std::cout << "     --mls:					Use moving least squares" << std::endl;
	std::cout << "     --sp val:				smart sampling borders, set angle_degree thresh" << std::endl;
	std::cout << "     --model_ds val:			Model down sampling radtia (default 0.05)" << std::endl;
	std::cout << "     --plane_ds val:			Model plane feature down sampling ratia, if not set, default same as model" << std::endl;
	std::cout << "     --curv_r val:			curvature radius" << std::endl;
  std::cout << "     --a_div val:				angle division" << std::endl;
  std::cout << "     --d_div val:				distance division" << std::endl;

}

void parseCommandLine(int argc, char *argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Program behavior
	//if (pcl::console::find_switch(argc, argv, "-r"))
	//{
	//	use_cloud_resolution_ = true;
	//}
	if (pcl::console::find_switch(argc, argv, "-w"))
	{
		save_sampled_cloud_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "--rn"))
	{
		normal_reorient_switch_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "--sp"))
	{
		smart_sample_border_= true;
	}
	if (pcl::console::find_switch(argc, argv, "--so"))
	{
		show_original_model_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "--in"))
	{
		use_existing_normal_data_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "--mls"))
	{
		use_mls_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "--cc"))
	{
		change_center_switch_ = true;
	}
  //Model filename
  pcl::console::parse_argument(argc, argv, "--mod", model_filename_);
  pcl::console::parse_argument(argc, argv, "--out", save_filename_);
  if(save_filename_.empty()){
    int pos = model_filename_.find_last_of('.');
    save_filename_ = model_filename_.substr(0, pos);
    save_filename_ += ".ppfs";
  }
  else{
    if(save_filename_.find(".ppfs")==std::string::npos){
      pcl::console::print_error("invalid output file name, must be *.ppfs!");
      exit(-1);
    }
  }
	//General parameters
	pcl::console::parse_argument(argc, argv, "--model_ds", model_ds_);
	plane_ds_ = model_ds_;
	pcl::console::parse_argument(argc, argv, "--plane_ds", plane_ds_);
	pcl::console::parse_argument(argc, argv, "--curv_r", curvature_radius_);
	pcl::console::parse_argument(argc, argv, "--sp", ang_thresh);
	pcl::console::parse_argument(argc, argv, "--a_div", angle_div_);
	pcl::console::parse_argument(argc, argv, "--d_div", distance_div_);

}





int
main(int argc, char *argv[])
{
	parseCommandLine(argc, argv);

	showHelp(argv[0]);
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr keyNormals(new pcl::PointCloud<NormalType>());
	if (use_existing_normal_data_) {
		if (!zyk::readPointCloud(model_filename_, model, model_normals))
			return(-1);
	}
	else {
		if (!zyk::readPointCloud(model_filename_, model))
			return(-1);
	}

	//
	// show original model
	//
	if (show_original_model_)
	{
		pcl::visualization::PCLVisualizer key_visual("Original Viewr");
		key_visual.addCoordinateSystem(20);
		if (use_existing_normal_data_)
			key_visual.addPointCloudNormals<PointType, NormalType>(model, model_normals, 1, 10, "model_normal");
		key_visual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<PointType>(model, 0.0, 0.0, 255.0), "model");
		key_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "model_normal");
		key_visual.spin();
	}

	//
	//  Set up resolution invariance
	//

	double max_coord[3];
	double min_coord[3];
	float resolution = static_cast<float> (zyk::computeCloudResolution(model, max_coord, min_coord));

	double model_length = max_coord[0] - min_coord[0];
	double model_width = max_coord[1] - min_coord[1];
	double model_height = max_coord[2] - min_coord[2];
	Eigen::Vector3f model_approximate_center;
	model_approximate_center(0) = (max_coord[0] + min_coord[0]) / 2;
	model_approximate_center(1) = (max_coord[1] + min_coord[1]) / 2;
	model_approximate_center(2) = (max_coord[2] + min_coord[2]) / 2;


	double d_max = sqrt(model_length*model_length + model_width*model_width + model_height*model_height);
  double model_ss_ = model_ds_*d_max;
  //double model_ss_ = 0.05*d_max;
	std::cout << "Model resolution:       " << resolution << std::endl;
	std::cout << "Model sampling distance step:    " << model_ss_ << std::endl;
	

	std::cout << "Model length: " << model_length << std::endl;
	std::cout << "Model width: " << model_width << std::endl;
	std::cout << "Model height: " << model_height << std::endl;

	//
	//  Compute Normals
	//

	if (!use_existing_normal_data_)
	{
		if (!use_mls_)
		{
			pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
			//norm_est.setKSearch(20);
			norm_est.setRadiusSearch(model_ss_);
			norm_est.setInputCloud(model);
			norm_est.compute(*model_normals);
		}
		else
		{
			pcl::PointCloud<PointType>::Ptr pnts_tmp(new pcl::PointCloud<PointType>());
			pcl::MovingLeastSquares<PointType, PointType> mls;
			pcl::search::KdTree<PointType>::Ptr tree;
			// Set parameters
			mls.setInputCloud(model);
			mls.setComputeNormals(true);
			mls.setPolynomialFit(true);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(model_ss_);
			mls.process(*pnts_tmp);
			model = pnts_tmp;
			model_normals = mls.getNormals();
			//for (size_t i = 0; i < scene_keyNormals->size(); ++i)
			//{
			//	flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
			//}
		}

    cout << "Normal compute complete" << endl;
	}
	else
	{
		cout << "Using existing normals" << endl;
	}


	if (normal_reorient_switch_)
	{
		std::cout << "Reorint Normals" << std::endl;
		for (int i = 0; i < model->size(); ++i)
		{
			Eigen::Vector3f pnt_temp = model->points[i].getVector3fMap();
			Eigen::Vector3f normal_temp = model_normals->points[i].getNormalVector3fMap();
			if ((pnt_temp - model_approximate_center).dot(normal_temp) < 0)
			{
				model_normals->points[i].normal_x = -normal_temp(0);
				model_normals->points[i].normal_y = -normal_temp(1);
				model_normals->points[i].normal_z = -normal_temp(2);
			}
			//if (change_center_switch_)
			//{
			//	model->points[i].x -= model_approximate_center(0);
			//	model->points[i].y -= model_approximate_center(1);
			//	model->points[i].z -= model_approximate_center(2);
			//}

		}
	}

	//
	// Compute curvature
	//
	if (curvature_radius_ > 0) {
		pcl::PrincipalCurvaturesEstimation<PointType, NormalType> curv_est;
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv(new pcl::PointCloud<pcl::PrincipalCurvatures>());
		curv_est.setInputCloud(model);
		curv_est.setInputNormals(model_normals);
		curv_est.setRadiusSearch(curvature_radius_ * resolution);
		curv_est.compute(*curv);


		//
		// show those curvature 0 and none_zero separately
		//
		pcl::PointCloud<PointType>::Ptr pnts_zero(new pcl::PointCloud<PointType>());
		pcl::PointCloud<NormalType>::Ptr normals_zero(new pcl::PointCloud<NormalType>());
		pcl::PointCloud<PointType>::Ptr pnts_no_zero(new pcl::PointCloud<PointType>());
		pcl::PointCloud<NormalType>::Ptr normals_no_zero(new pcl::PointCloud<NormalType>());
		for (int i = 0; i < model->size(); ++i)
		{
			pcl::PrincipalCurvatures c = curv->at(i);
			if (abs(c.pc1) < 0.001&&abs(c.pc2) < 0.001)
			{
				pnts_zero->push_back(model->at(i));
				normals_zero->push_back(model_normals->at(i));
			}
			else
			{
				pnts_no_zero->push_back(model->at(i));
				normals_no_zero->push_back(model_normals->at(i));
			}
		}

		//
		//visualize zero
		//

		pcl::visualization::PCLVisualizer zeroVisual("zero detect");
		zeroVisual.addCoordinateSystem(20);
		zeroVisual.addPointCloud(pnts_zero, pcl::visualization::PointCloudColorHandlerCustom<PointType>(pnts_zero, 0.0, 0.0, 255.0), "zero_pnts");
		zeroVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "zero_pnts");
		zeroVisual.spin();

		//
		//visualize no-zero
		//

		pcl::visualization::PCLVisualizer nozeroVisual("no-zero detect");
		nozeroVisual.addCoordinateSystem(20);
		nozeroVisual.addPointCloud(pnts_no_zero, pcl::visualization::PointCloudColorHandlerCustom<PointType>(pnts_no_zero, 0.0, 0.0, 255.0), "pnts_no_zero");
		nozeroVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pnts_no_zero");
		nozeroVisual.spin();

		//
		//  Downsample Clouds to Extract keypoints
		//
		pcl::PointCloud<PointType>::Ptr model_zero_curvature_keypoints(new pcl::PointCloud<PointType>());
		pcl::PointCloud<NormalType>::Ptr model_zero_curvature_keyNormals(new pcl::PointCloud<NormalType>());
		pcl::PointCloud<PointType>::Ptr model_no_zero_curvatur_keypoints(new pcl::PointCloud<PointType>());
		pcl::PointCloud<NormalType>::Ptr model_no_zero_curvature_keyNormals(new pcl::PointCloud<NormalType>());
		//smat sample
		pcl::SmartSampling<PointType, NormalType> smart_samp;
		if (smart_sample_border_)
			zyk::SmartDownSamplePointAndNormal(pnts_no_zero, normals_no_zero, ang_thresh, model_ss_, model_no_zero_curvatur_keypoints, model_no_zero_curvature_keyNormals);
		else
			zyk::uniformDownSamplePointAndNormal(pnts_no_zero, normals_no_zero, model_ss_, model_no_zero_curvatur_keypoints, model_no_zero_curvature_keyNormals);
		if (plane_ds_ > 0)
			zyk::uniformDownSamplePointAndNormal(pnts_zero, normals_zero, plane_ds_*d_max, model_zero_curvature_keypoints, model_zero_curvature_keyNormals);
		std::cout << "Model total points: " << model->size() << std::endl;
		std::cout << "No zero total points: " << pnts_no_zero->size() << "; Selected downsample: " << model_no_zero_curvatur_keypoints->size() << std::endl;
		std::cout << "zero total points: " << pnts_zero->size() << "; Selected downsample: " << model_zero_curvature_keypoints->size() << std::endl;


		//
		// combine key pnts and key normals
		//
		keypoints = model_no_zero_curvatur_keypoints;
		keyNormals = model_no_zero_curvature_keyNormals;
		if (plane_ds_ > 0) {
			*keypoints += *model_zero_curvature_keypoints;
			*keyNormals += *model_zero_curvature_keyNormals;
		}
	}
	else {
    if(smart_sample_border_)
    {
      std::cout<<"Use smart sampling, angle thresh(deg): "<<ang_thresh<<std::endl;
      zyk::SmartDownSamplePointAndNormal(model, model_normals,ang_thresh,model_ss_,keypoints, keyNormals);
    }
    else
    {
      std::cout<<"Use uniform sampling"<<std::endl;
      zyk::uniformDownSamplePointAndNormal(model, model_normals, model_ss_, keypoints, keyNormals);
    }

	}
  std::cout << "Model total points: " << model->size() << std::endl;
  std::cout <<" Selected downsample: " << keypoints->size() << std::endl;

  if (save_sampled_cloud_)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*keypoints,*model_with_normals);
    pcl::copyPointCloud(*keyNormals,*model_with_normals);
    pcl::io::savePLYFile(model_filename_ + "_changed", *model_with_normals);
    std::cout<<"save sample changed!"<<std::endl;
  }

	//
	//visualize keypoints
	//

	pcl::visualization::PCLVisualizer key_visual("Key point Viewr");
	key_visual.addCoordinateSystem(20);
	key_visual.addPointCloud(keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(keypoints, 0.0, 0.0, 255.0), "keypoints");
	key_visual.addPointCloudNormals<PointType, NormalType>(keypoints, keyNormals, 1, 10, "keynormals");
	key_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
	key_visual.spin();


	//
	//  Compute Model Descriptors  PPF Space for model 
	//
	std::vector<double>model_size;
	model_size.push_back(model_width);
	model_size.push_back(model_length);
	model_size.push_back(model_height);
	std::sort(model_size.begin(), model_size.end());
	cout << "model_size after sort :";
	for (int i = 0; i < 3; i++)
		cout << model_size[i] << " \t";
	cout << endl;


	////
	//// if use smart sampling ,the resolution needs to be recalculated!
	////
	//if (smart_sample_border_)
	//{
	//	model_ss_ = static_cast<float> (computeCloudResolution(keypoints, max_coord, min_coord));
	//	cout << ">>Model smart sampled, then, recalculate resolution: " << model_ss_ << endl;
	//}

	//model ppf space
	char tmp[100];
	_splitpath(model_filename_.c_str(), NULL, NULL, tmp, NULL);
	std::string objName(tmp);
	std::cout << "Trained object Name: " << objName << std::endl;
	zyk::PPF_Space model_feature_space;
	cout << "trained using angle_div , distance_div: " << angle_div_ << ", " << distance_div_ << endl;
	model_feature_space.init(objName, keypoints, keyNormals, angle_div_ , distance_div_,true);
//	model_feature_space.model_size[0]=model_size[0];
//	model_feature_space.model_size[1]=model_size[1];
//	model_feature_space.model_size[2]=model_size[2];
  model_feature_space.model_res = model_ss_;
//  model_feature_space.model_res = 0.05*d_max;
	cout << "Calculated model max distance is " << model_feature_space.getMaxD() << endl;
	//
	// compute no empty ppf box nunber
	//
	int cnt = 0;
	for (int i = 0; i < model_feature_space.getBoxVector()->size(); ++i)
	{
		if (model_feature_space.getBoxVector()->at(i) != NULL)
			cnt++;
	}
	cout << "no empty box number is: " << cnt << endl;
	
	//CFile fileStore;
	//if (fileStore.Open(save_filename_.c_str(), CFile::modeWrite | CFile::modeCreate))
	//{
	//	CArchive arStore(&fileStore, CArchive::store);
	//	model_feature_space.Serialize(arStore);
	//	arStore.Close();
	//	fileStore.Close();
	//}
    std::cout<<"save file name: "<<save_filename_<<std::endl;
	getchar();
	model_feature_space.save(save_filename_);
	return 0;
}
