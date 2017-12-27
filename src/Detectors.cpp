#include "Detectors.h"
#include "PPFFeature.h"
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>


CDetectModel3D::CDetectModel3D()
	:mDetectOptions(detectOptions{ 0.05,0.3,1,10,1 })
	, mTrainOptions(trainOptions{ 0.05,1 })
	, ObjectName("")
	//, ShowColor("red")
	, p_PPF(NULL)
{
}

CDetectModel3D::~CDetectModel3D()
{
	clearSurModel();
}

bool CDetectModel3D::readSurfaceModel(string filePath)
{
	if (p_PPF != NULL) {
		delete p_PPF;
		p_PPF = NULL;
	}
	p_PPF = new zyk::PPF_Space;
	if (!p_PPF->load(filePath)) {
		delete p_PPF;
		p_PPF = NULL;
		return false;
	}
	ObjectName = p_PPF->getName();
	mTrainOptions.downSampleRatio = p_PPF->getSampleRatio();
	return true;
}
//
//bool CDetectModel3D::createSurModelFromCADFile(const string &filePath)
//{
//	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
//	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
//	// read cloud
//	if (!readPointCloud(filePath, model)) {
//		cout << "Error loading point cloud" << endl;
//		return false;
//	}
//	double max_coord[3];
//	double min_coord[3];
//	float resolution = static_cast<float> (computeCloudResolution(model, max_coord, min_coord));
//	double model_length = max_coord[0] - min_coord[0];
//	double model_width = max_coord[1] - min_coord[1];
//	double model_height = max_coord[2] - min_coord[2];
//	Eigen::Vector3f model_approximate_center;
//	model_approximate_center(0) = (max_coord[0] + min_coord[0]) / 2;
//	model_approximate_center(1) = (max_coord[1] + min_coord[1]) / 2;
//	model_approximate_center(2) = (max_coord[2] + min_coord[2]) / 2;
//
//	double d_max = sqrt(model_length*model_length + model_width*model_width + model_height*model_height);
//	double model_ss_ = mTrainOptions.downSampleRatio*d_max;
//	std::cout << "Model resolution:       " << resolution << std::endl;
//	std::cout << "Model sampling ratia:  " << mTrainOptions.downSampleRatio <<" Distance: "<<model_ss_<< std::endl;
//
//	/*@todo
//	// save the following to a struct
//	*/
//	std::cout << "Model length: " << model_length << std::endl;
//	std::cout << "Model width: " << model_width << std::endl;
//	std::cout << "Model height: " << model_height << std::endl;
//
//	// @todo not done
//	return true;
//}

void CDetectModel3D::clearSurModel()
{
	if (p_PPF != NULL) {
		delete p_PPF;
		p_PPF = NULL;
	}
}

void CDetectModel3D::getModelPointCloud(vector<Vec3d>& modelPoints)
{
	if (p_PPF != NULL) {
		pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
		p_PPF->getCenteredPointCloud(model);
		for (size_t i = 0; i < model->size(); ++i) {
			modelPoints.push_back(Vec3d{ model->at(i).x, model->at(i).y, model->at(i).z });
		}
	}
}

//global scene variable
pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
pcl::PointCloud<NormalType>::Ptr sceneNormals (new pcl::PointCloud<NormalType>());
pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());



bool CDetectors3D::readScene(const string filePath)
{
	scene->clear();
	sceneNormals->clear();
	if (!readPointCloud(filePath, scene)) {
		return false;
	}
	map<string, CDetectModel3D*>::iterator it;
	for (it = detectObjects.begin(); it != detectObjects.end(); ++it)
		it->second->result.matchComplete=false;
	return true;
}

bool CDetectors3D::findPart(const string objectName, double keyPointRatio)
{
	std::map<std::string, CDetectModel3D*>::iterator itr_modelDetector = detectObjects.find(objectName);
	if (itr_modelDetector==detectObjects.end())
		return false;
	if (scene->empty())
		return false;
	CDetectModel3D & modelDetector = *(itr_modelDetector->second);
	if (modelDetector.p_PPF == NULL)
		return false;
	//pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	//pcl::PointCloud<NormalType>::Ptr model_keyNormals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints;
	pcl::PointCloud<NormalType>::Ptr model_keyNormals;
	zyk::PPF_Space*pPPF = modelDetector.p_PPF;
	pPPF->getCenteredPointCloud(model_keypoints);
	pPPF->getPointNormalCloud(model_keyNormals);

	//downsample
	pcl::IndicesPtr sampled_index_ptr;
	float scene_ss_ = (modelDetector.mDetectOptions.downSampleRatio / modelDetector.mTrainOptions.downSampleRatio)*pPPF->model_res;
	sampled_index_ptr = uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);

	//normals
	pcl::MovingLeastSquaresOMP<PointType, PointType> mls;
	pcl::search::KdTree<PointType>::Ptr tree;
	mls.setInputCloud(scene);
	mls.setIndices(sampled_index_ptr);
	mls.setComputeNormals(true);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(modelDetector.mDetectOptions.mlsOrder);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(scene_ss_);
	mls.process(*scene_keypoints);
	scene_keyNormals = mls.getNormals();
	for (size_t i = 0; i < scene_keyNormals->size(); ++i)
	{
		pcl::flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
	}
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> pose_clusters;
	pPPF->match(scene_keypoints, scene_keyNormals, false, true, keyPointRatio, 3, 0.95, 0.2, 0.1, 0.1, 0.3, modelDetector.mDetectOptions.MaxOverlapDistRel,1, pose_clusters);
	//matchResult[objectIndex].resize(detectObjects[i].mDetectOptions.)
	int max_num = modelDetector.mDetectOptions.maxNumber;
	double minScore = modelDetector.mDetectOptions.minScore;
	matchResult& res = modelDetector.result;
	res.clear();
	if (pose_clusters.empty()) {
		return false;
	}
	for (int i = 0; i < max_num && pose_clusters[i].vote_count > minScore; ++i) {
		res.scores.push_back(pose_clusters[i].vote_count);
		res.rotatios.push_back(Vec3d{ pose_clusters[i].mean_rot[0], pose_clusters[i].mean_rot[1], pose_clusters[i].mean_rot[2] });
		res.translations.push_back(Vec3d{ pose_clusters[i].mean_trans[0], pose_clusters[i].mean_trans[1], pose_clusters[i].mean_trans[2] });
	}
	res.matchComplete = true;
	return true;
}

bool CDetectors3D::findParts(double keyPointRatio)
{
	//@todo repeated using find
	bool res = true;
	map<string, CDetectModel3D*>::iterator it;
	for (it=detectObjects.begin();it!=detectObjects.end();++it)
		if (!findPart(it->first, keyPointRatio))
			return false;
}

CDetectModel3D* CDetectors3D::readSurfaceModel(string filePath)
{
	CDetectModel3D* tmp_model3d = new CDetectModel3D();
	if (!tmp_model3d->readSurfaceModel(filePath)) {
		delete tmp_model3d;
		return NULL;
	}
	detectObjects.insert(make_pair(tmp_model3d->ObjectName, tmp_model3d));
	return tmp_model3d;
}

void CDetectors3D::showMatchResults()
{
	if (scene->empty()) {
		cout << "Empty scene!" << endl;
		return;
	}
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, 0, 255, 0);
	viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");
	map<string, CDetectModel3D*>::iterator it;
	for (it = detectObjects.begin(); it != detectObjects.end(); ++it) {
		pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
		//pcl::PointCloud<NormalType>::Ptr model_keyNormals(new pcl::PointCloud<NormalType>());
		if (it->second->p_PPF == NULL)
			continue;
		it->second->p_PPF->getModelPointCloud(model_keypoints);
		//detectObjects[i].p_PPF->getPointNormalCloud(model_keyNormals);
		matchResult& res = it->second->result;
		if (res.matchComplete)
			for (int j = 0; j < res.rotatios.size(); ++j) {
				pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
				float tx = res.translations[j].x;
				float ty = res.translations[j].y;
				float tz = res.translations[j].z;
				float rx = res.rotatios[j].x;
				float ry = res.rotatios[j].y;
				float rz = res.rotatios[j].z;
				float ang = sqrt(rx*rx + ry*ry + rz*rz);
				Eigen::Affine3f transformation = Eigen::Affine3f(Eigen::Translation3f(tx, ty, tz)*Eigen::AngleAxisf(ang, Eigen::Vector3f(rx, ry, rz).normalized()));
				pcl::transformPointCloud(*model_keypoints, *rotated_model, transformation);
				std::stringstream ss_cloud;
				ss_cloud << it->second->ObjectName << "Result" << j;
				pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
				viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_cloud.str());
			}

	}

	viewer.spin();
}

void CDetectors3D::clear()
{
	map<string, CDetectModel3D*>::iterator it;
	for (it = detectObjects.begin(); it != detectObjects.end(); ++it)
		if (it->second != NULL)
			delete it->second;
	detectObjects.clear();
}

