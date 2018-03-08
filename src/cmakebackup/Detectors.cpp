#include "Detectors.h"
#include "PPFFeature.h"
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>


CDetectModel3D::CDetectModel3D()
	: ObjectName("")
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

int CDetectModel3D::createSurfaceModel(string filePath, string savePath, string objName)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>());
  pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>());
  double min_coord[3],max_coord[3];
  if(!zyk::mesh_sampling(filePath,30000,*model,min_coord,max_coord)){
    PCL_ERROR("Samping mesh fail!Must be OBJ format!");
    return -1;
  }
  double model_length = max_coord[0] - min_coord[0];
  double model_width = max_coord[1] - min_coord[1];
  double model_height = max_coord[2] - min_coord[2];
  double d_max = sqrt(model_length*model_length + model_width*model_width + model_height*model_height);
  double model_ss_ = mTrainOptions.downSampleRatio*d_max;

  //zyk::SmartDownSamplePointAndNormal(model,30,model_ss_,keypoints);
  zyk::uniformDownSamplePointAndNormal(model, model_ss_, keypoints);
  std::cout << "Key points size: " << keypoints->size() << std::endl;
  pcl::PointCloud<PointType>::Ptr input_points (new pcl::PointCloud<PointType>());
  pcl::PointCloud<NormalType>::Ptr input_normals (new pcl::PointCloud<NormalType>());
  pcl::copyPointCloud(*keypoints,*input_points);
  pcl::copyPointCloud(*keypoints,*input_normals);


  if (objName.empty()) {
    char tmp[100];
    _splitpath(filePath.c_str(), NULL, NULL, tmp, NULL);
    objName = std::string(tmp);
  }
  this->ObjectName = objName;

  std::string save_filename_;
  if (savePath.empty()) {
    int pos = filePath.find_last_of('.');
    save_filename_ = filePath.substr(0, pos);
    save_filename_ += ".ppfs";
  }
  else
    save_filename_ = savePath;


  if (p_PPF != NULL) {
    delete p_PPF;
    p_PPF = NULL;
  }
  p_PPF = new(zyk::PPF_Space);
  if (!p_PPF->init(objName, input_points, input_normals, 20, 20, true))
	  return -2;

  p_PPF->model_res = model_ss_;

  if (!p_PPF->save(save_filename_))
	  return -3;

	return 1;
}

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
		p_PPF->getModelPointCloud(model);
		for (size_t i = 0; i < model->size(); ++i) {
			modelPoints.push_back(Vec3d{ model->at(i).x, model->at(i).y, model->at(i).z });
		}
	}
}

void CDetectModel3D::getModelPointNormals(vector<Vec3d>& modelNormals)
{
	if (p_PPF != NULL) {
		pcl::PointCloud<NormalType>::Ptr normals(new pcl::PointCloud<NormalType>());
		p_PPF->getPointNormalCloud(normals);
		for (size_t i = 0; i < normals->size(); ++i) {
			modelNormals.push_back(Vec3d{ normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z });
		}
	}
}

//global scene variable
pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
pcl::PointCloud<NormalType>::Ptr sceneNormals (new pcl::PointCloud<NormalType>());
pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());

double scene_resolution = -1.0;


bool CDetectors3D::readScene(const string filePath)
{
	scene->clear();
	sceneNormals->clear();
	if (!zyk::readPointCloud(filePath, scene)) {
		return false;
	}
	scene_resolution = zyk::computeCloudResolution(scene);
	map<string, CDetectModel3D*>::iterator it;
	for (it = detectObjects.begin(); it != detectObjects.end(); ++it)
		it->second->result.matchComplete=false;
	return true;
}

bool CDetectors3D::readScene(const vector<Vec3d>& pointCloud)
{
	scene->clear();
	sceneNormals->clear();
	if (pointCloud.empty())
		return false;
	for (size_t i = 0; i < pointCloud.size(); ++i) {
		PointType _tem;
		_tem.x = pointCloud[i].x;
		_tem.y = pointCloud[i].y;
		_tem.z = pointCloud[i].z;
		scene->push_back(_tem);
	}

	return true;
}

bool CDetectors3D::findPart(const string objectName, double keyPointRatio)
{
	int startTime = clock();
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
  //@TODO
	float scene_ss_ = (modelDetector.mDetectOptions.downSampleRatio / modelDetector.mTrainOptions.downSampleRatio)*pPPF->model_res;
	sampled_index_ptr = zyk::uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);

	//normals
	pcl::MovingLeastSquaresOMP<PointType, PointType> mls;
	pcl::search::KdTree<PointType>::Ptr tree;
	mls.setInputCloud(scene);
	mls.setIndices(sampled_index_ptr);
	mls.setComputeNormals(true);
	mls.setPolynomialOrder(modelDetector.mDetectOptions.mlsOrder);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(pPPF->model_res);
	mls.process(*scene_keypoints);
	scene_keyNormals = mls.getNormals();
	for (size_t i = 0; i < scene_keyNormals->size(); ++i)
	{
		pcl::flipNormalTowardsViewpoint(scene_keypoints->at(i), 0, 0, 0, scene_keyNormals->at(i).normal[0], scene_keyNormals->at(i).normal[1], scene_keyNormals->at(i).normal[2]);
	}
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> pose_clusters;
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> refined_pose_clusters;
	pPPF->match(
		scene_keypoints, 
		scene_keyNormals, 
		false, //spread ppf switch
		true, //two ball switch
		keyPointRatio, 
		3, //max vote thresh
		0.95, //max vote percentage
		0.15, //angle thresh
		0.1, //first distance thresh
		0.1, //recompute score distance thresh
		0.3, //recompute score angle thresh
		modelDetector.mDetectOptions.MaxOverlapDistRel,// max overlap ratio
		1, //
		pose_clusters
	);
	if (pose_clusters.empty()) {
		return false;
	}
	int icp_start_time = clock();
	//matchResult[objectIndex].resize(detectObjects[i].mDetectOptions.)
	int max_num = modelDetector.mDetectOptions.maxNumber;
	double minScore = modelDetector.mDetectOptions.minScore;
	//do icp
	int icp_number = std::min(max_num, int(pose_clusters.size()));

	if (modelDetector.mDetectOptions.param_1 > 0) {
		std::cout << "ICP VERSION 2..." << std::endl;
		pPPF->ICP_Refine2_0(scene, pose_clusters, refined_pose_clusters, icp_number, scene_resolution, modelDetector.mDetectOptions.param_1);
	}
	else {
		std::cout << "ICP VERSION 1..." << std::endl;
		pPPF->ICP_Refine(scene, pose_clusters, refined_pose_clusters, icp_number, scene_resolution);
	}
	std::sort(refined_pose_clusters.begin(), refined_pose_clusters.end(), zyk::pose_cluster_comp);
	matchResult& res = modelDetector.result;
	res.clear();
	for (int i = 0; i < max_num && refined_pose_clusters[i].vote_count > minScore; ++i) {
		res.scores.push_back(refined_pose_clusters[i].vote_count);
		res.rotatios.push_back(Vec3d{ refined_pose_clusters[i].mean_rot[0], refined_pose_clusters[i].mean_rot[1], refined_pose_clusters[i].mean_rot[2] });
		res.translations.push_back(Vec3d{ refined_pose_clusters[i].mean_trans[0], refined_pose_clusters[i].mean_trans[1], refined_pose_clusters [i].mean_trans[2] });
	}
	int end_time = clock();
	res.matchTime = (end_time - startTime) / 1000.0;
	res.icpTime = (end_time - icp_start_time) / 1000.0;
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
	return true;
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

				std::cout << ">>>>>>Pose " << j << "<<<<<<<" << std::endl;
				std::cout << "rot: " << rx << " " << ry << " " << rz << std::endl;
				std::cout << "tra: " << tx << " " << ty << " " << tz << std::endl;
				std::cout << "score: " << res.scores[j] << std::endl;
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


