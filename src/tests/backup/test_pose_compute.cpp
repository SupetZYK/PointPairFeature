//firstc
//from a 3d model, randomly give a rotation and translation.
//randomly choose a pair of points, and from the corresponding pair 
//calculate the rotation and translation. Choose another pair and do the same


//second
//from a 3d model, randomly give a rotation and translation
//choose a pair of points and calculate the rotation and translation
//make another rotaion and translation....

#include "common.h"
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"
std::string model_file_name_="../datafile/template_whitePart.ply";
int main(int argc, char**argv)
{
	
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_new(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr model_normals_new(new pcl::PointCloud<NormalType>());
	readPointCloud(model_file_name_,"ply",model,model_normals);

	//
	// Make a transformation to model ,zyk
	//

	double max_coord[3];
	double min_coord[3];
	float resolution = static_cast<float> (computeCloudResolution(model, max_coord, min_coord));


	Eigen::Vector3d model_approximate_center;
	model_approximate_center(0) = (max_coord[0] + min_coord[0]) / 2;
	model_approximate_center(1) = (max_coord[1] + min_coord[1]) / 2;
	model_approximate_center(2) = (max_coord[2] + min_coord[2]) / 2;

	double model_length = max_coord[0] - min_coord[0];
	double model_width = max_coord[1] - min_coord[1];
	double model_height = max_coord[2] - min_coord[2];
	//
	// Define a translation of 2.5 meters on the x axis.
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate(Eigen::AngleAxisf(0.25*M_PI, Eigen::Vector3f::UnitZ()));
	transform_2.rotate(Eigen::AngleAxisf(0.3*M_PI, Eigen::Vector3f::UnitX()));
	Eigen::Vector3d trans = model_approximate_center + Eigen::Vector3d(2 * model_length, 2 * model_width, 2 * model_height);
	transform_2.translation() << trans(0), trans(1), trans(2);
	pcl::transformPointCloud(*model, *model_new, transform_2);
	transformNormals(*model_normals, *model_normals_new, transform_2);

	


	//visualize two models
	
	pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
	keyPointVisual.addCoordinateSystem(0.2);
	//model
	keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<PointType>(model, 0.0, 255.0, 0.0), "model");
	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model");
	//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
	keyPointVisual.addPointCloudNormals<PointType, NormalType>(model, model_normals, 1, 10, "model_normals");

	//model_new
	keyPointVisual.addPointCloud(model_new, pcl::visualization::PointCloudColorHandlerCustom<PointType>(model_new, 255.0, 0.0, 0.0), "model_new");

	// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_new");
	keyPointVisual.addPointCloudNormals<PointType, NormalType>(model_new, model_normals_new, 1, 10, "model_normals_new");
	//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
	keyPointVisual.spin();
	Eigen::AngleAxisf truerot(transform_2.rotation());
	cout<<"rot is"<<transform_2.rotation()<<endl;
	cout<<"true rot: "<<(truerot.angle()*truerot.axis()).transpose()<<"true trans"<<transform_2.translation().transpose()<<endl;
	for(int32_t i=100;i<1100;i++)
	{
		zyk::PPF ppf1,ppf2;
		zyk::PPF_Space::computeSinglePPF(model,model_normals,i,i+1000,ppf1);
		zyk::PPF_Space::computeSinglePPF(model_new,model_normals_new,i,i+1000,ppf2);
		float alpha=ppf1.ppf.alpha_m-ppf2.ppf.alpha_m;
		
		if (alpha >= M_PI) alpha -= 2 * M_PI;
		if (alpha < -M_PI)alpha += 2 * M_PI;
		Eigen::Affine3f tmp;
		zyk::PPF_Space::getPoseFromPPFCorresspondence(model->at(i),model_normals->at(i),model_new->at(i),model_normals_new->at(i),alpha,tmp);
		Eigen::AngleAxisf rot(tmp.rotation());
		cout<<"alpha: "<<alpha<<"Rot: "<<rot.angle()*rot.axis().transpose()<<"Trans: "<<tmp.translation().transpose()<<endl;
	}	
		
		

	return 0;
}
