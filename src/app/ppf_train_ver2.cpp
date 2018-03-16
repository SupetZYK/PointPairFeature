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
//bool change_center_switch_(false);
bool use_mls_ (false);
float ang_degree_thresh (15);
float model_ds_ (0.05f);
float plane_ds_ (0.05f);
float curvature_radius_ (0.05f);
int angle_div_ (15);
int distance_div_ (20);
bool use_plane_flag_ (false);
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
  std::cout << "     -h:              Show this help." << std::endl;
  std::cout << "     --mod val:       Path of the model CAD(ply/obj)." << std::endl;
  std::cout << "     --out val:       Path of the output .ppfs file(if not specified, same as model)" << std::endl;
	//std::cout << "     -r:						Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "     -w:              write the sampled model" << std::endl;
	//std::cout << "     --ply:					Use .poly as input cloud. Default is .pcd" << std::endl;
  std::cout << "     --rn:            Reorient switch!" << std::endl;
  std::cout << "     --plf:           Plane feature flag." << std::endl;
  std::cout << "     --cc:            Change Center switch!" << std::endl;
  std::cout << "     --so:            show original model" << std::endl;
  std::cout << "     --in:            Use existing normal files" << std::endl;
  std::cout << "     --mls:           Use moving least squares" << std::endl;
  std::cout << "     --sp val:        smart sampling, set angle_degree thresh" << std::endl;
  std::cout << "     --model_ds val:  Model down sampling radtia (default 0.05)" << std::endl;
  std::cout << "     --plane_ds val:  Model plane feature down sampling ratia, if not set, default same as model" << std::endl;
  std::cout << "     --curv_r val:  	curvature radius" << std::endl;
  std::cout << "     --a_div val: 		angle division" << std::endl;
  std::cout << "     --d_div val:			distance division" << std::endl;

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
//	if (pcl::console::find_switch(argc, argv, "-w"))
//	{
//		save_sampled_cloud_ = true;
//	}
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
    if (pcl::console::find_switch(argc, argv, "--plf"))
    {
        use_plane_flag_ = true;
    }
//	if (pcl::console::find_switch(argc, argv, "--cc"))
//	{
//		change_center_switch_ = true;
//	}
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
  pcl::console::parse_argument(argc, argv, "--sp", ang_degree_thresh);
	pcl::console::parse_argument(argc, argv, "--a_div", angle_div_);
	pcl::console::parse_argument(argc, argv, "--d_div", distance_div_);

}



bool isOnPlane(float x, float y, float z, Eigen::Vector4f& plane_param);
int
main(int argc, char *argv[])
{
	parseCommandLine(argc, argv);

	showHelp(argv[0]);
    pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>());

    double min_coord[3],max_coord[3];
    if(!zyk::mesh_sampling(model_filename_,300000,*model,min_coord,max_coord)){
        PCL_ERROR("Samping mesh fail!");
     return -1;
  }


	//
	//  Set up resolution invariance
	//
	double model_length = max_coord[0] - min_coord[0];
	double model_width = max_coord[1] - min_coord[1];
	double model_height = max_coord[2] - min_coord[2];
	Eigen::Vector3f model_approximate_center;
	model_approximate_center(0) = (max_coord[0] + min_coord[0]) / 2;
	model_approximate_center(1) = (max_coord[1] + min_coord[1]) / 2;
	model_approximate_center(2) = (max_coord[2] + min_coord[2]) / 2;


	double d_max = sqrt(model_length*model_length + model_width*model_width + model_height*model_height);
	double model_ss_ = model_ds_*d_max;
	std::cout << "Model sampling distance step:    " << model_ss_ << std::endl;
	std::cout << "Model length: " << model_length << std::endl;
	std::cout << "Model width: " << model_width << std::endl;
	std::cout << "Model height: " << model_height << std::endl;

  //
  // show original model
  //
  if (show_original_model_)
  {

    pcl::visualization::PCLVisualizer org_visual("Original Viewr");
    org_visual.addCoordinateSystem(20);
    org_visual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(model, 0.0, 0.0, 255.0), "model");
    org_visual.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(model, model, 1, 10, "model_normal");
    org_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");
    org_visual.spin();
  }

  if(smart_sample_border_)
  {
    std::cout<<"Use smart sampling, angle thresh(deg): "<<ang_degree_thresh<<std::endl;
    zyk::SmartDownSamplePointAndNormal(model,ang_degree_thresh,model_ss_,keypoints);
  }
  else
  {
    std::cout<<"Use uniform sampling"<<std::endl;
    zyk::uniformDownSamplePointAndNormal(model, model_ss_, keypoints);
  }
  std::cout << "Model total points: " << model->size() << std::endl;
  std::cout <<" Selected downsample: " << keypoints->size() << std::endl;

  if (save_sampled_cloud_)
  {
    pcl::io::savePLYFile(model_filename_ + "_changed", *keypoints);
    std::cout<<"save sample changed!"<<std::endl;
  }

  pcl::PointCloud<PointType>::Ptr input_points (new pcl::PointCloud<PointType>());
  pcl::PointCloud<NormalType>::Ptr input_normals (new pcl::PointCloud<NormalType>());
  pcl::copyPointCloud(*keypoints,*input_points);
  pcl::copyPointCloud(*keypoints,*input_normals);

	//
	//visualize keypoints
	//

	pcl::visualization::PCLVisualizer key_visual("Key point Viewr");
	key_visual.addCoordinateSystem(20);
    key_visual.addPointCloud(keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(keypoints, 0.0, 0.0, 255.0), "keypoints");
    key_visual.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(keypoints, keypoints, 1, 10, "keynormals");
	key_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
	key_visual.spin();

    //
    //zyk 2018-3-16 plane feature set
    //
     vector<bool> plane_flag(input_points->size(),false);
    if(use_plane_flag_){
        vector<Eigen::Vector4f > plane_features;
        plane_features.push_back(Eigen::Vector4f(0,0,1,-25));
        plane_features.push_back(Eigen::Vector4f(0,0,-1,0));
        plane_features.push_back(Eigen::Vector4f(0,1,0,-70));
        plane_features.push_back(Eigen::Vector4f(0,-1,0,-55));
        plane_features.push_back(Eigen::Vector4f(1,0,0,-35));
        plane_features.push_back(Eigen::Vector4f(-1,0,0,-35));

        //loop through keypoints to get a plane bool vector
        int plane_numbers=0;
        for(int i=0;i<input_points->size();++i)
        {
            for(int j=0;j<plane_features.size();++j)
            {
                if(isOnPlane(input_points->at(i).x,input_points->at(i).y,input_points->at(i).z,plane_features[j]))
                {
                    plane_flag[i]=true;
                    plane_numbers++;
                    break;
                }
            }
        }
        std::cout<<"Number of points on plane: "<<plane_numbers<<std::endl;
    }
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

//  //
//  // if use smart sampling ,the resolution needs to be recalculated!
//  //
//  if (smart_sample_border_)
//  {
//    model_ss_ = static_cast<float> (zyk::computeCloudResolution(input_points, max_coord, min_coord));
//    cout << ">>Model smart sampled, then, recalculate resolution: " << model_ss_ << endl;
//  }

  //model ppf space
  char tmp[100];
  _splitpath(model_filename_.c_str(), NULL, NULL, tmp, NULL);
  std::string objName(tmp);
  std::cout << "Trained object Name: " << objName << std::endl;
    zyk::PPF_Space model_feature_space;
  cout << "trained using angle_div , distance_div: " << angle_div_ << ", " << distance_div_ << endl;

  model_feature_space.init(objName, input_points, input_normals, angle_div_ , distance_div_,true);
  if(use_plane_flag_)
      model_feature_space.setPlaneFlag(plane_flag);
  model_feature_space.model_size[0]=model_size[0];
  model_feature_space.model_size[1]=model_size[1];
  model_feature_space.model_size[2]=model_size[2];
  model_feature_space.model_res = model_ss_;
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
  std::cout<<"save file name: "<<save_filename_<<std::endl;
  getchar();
  model_feature_space.save(save_filename_);
	return 0;
}

bool isOnPlane(float x, float y, float z, Eigen::Vector4f& plane_param)
{
    Eigen::Vector4f p(x,y,z,1);
    return abs(p.dot(plane_param))<0.0001;
}
