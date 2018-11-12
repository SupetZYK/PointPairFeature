#include <util_pcl.h>
#include <util.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/features/ppf.h>

using namespace std;
using namespace pcl;

struct PPF
{
	int first_index;
	int second_index;
	pcl::PPFSignature ppf;
	float weight;
	PPF():first_index(-1),second_index(-1),weight(1.0){};
};

void computeSinglePPF(const PointType& first_pnt, const NormalType& first_normal, const PointType& second_pnt, const NormalType& second_normal, PPF& ppf)
{
	float d[3];
	d[0] = second_pnt.x - first_pnt.x;
	d[1] = second_pnt.y - first_pnt.y;
	d[2] = second_pnt.z - first_pnt.z;
	float d_norm = zyk::norm(d, 3);

	d[0] /= d_norm;
	d[1] /= d_norm;
	d[2] /= d_norm;

	float dot_res;
	dot_res = zyk::dot(first_normal.normal, d, 3);
	if (dot_res > 1)dot_res = 1;
	else if (dot_res < -1)dot_res = -1;
	// res is between 0-pi
	ppf.ppf.f1 = acosf(dot_res);

	dot_res = zyk::dot(second_normal.normal, d, 3);
	if (dot_res > 1)dot_res = 1;
	else if (dot_res < -1)dot_res = -1;
	//res is between 0-pi, use which angle is not important! pi-pf.ppf.f2 is equivalent
	ppf.ppf.f2 = acosf(dot_res);

	dot_res = zyk::dot(first_normal, second_normal);
	if (dot_res > 1)dot_res = 1;
	else if (dot_res < -1)dot_res = -1;
	ppf.ppf.f3 = acosf(dot_res);

	ppf.ppf.f4 = d_norm;
}
int main()
{
	string scene_file_name = "../../../datafile/ape_test/depth0.dpt.ply";
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints_reference(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr scene_keyNormals(new pcl::PointCloud<NormalType>());


	// read pointcloud first
	clock_t t1 = clock();
	zyk::readPointCloud(scene_file_name, scene);
	clock_t t2 = clock();
	cout << "t: " << t2 - t1 << endl;
	//downsample
	float scene_ss_ = 7.0;
	pcl::IndicesPtr sampled_index_ptr = zyk::uniformDownSamplePoint(scene, scene_ss_, scene_keypoints);
	clock_t t21 = clock();
	cout << "t: " << t21 - t2 << endl;
	//compute normal
	search::KdTree<PointType>::Ptr tree;
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setIndices(sampled_index_ptr);
	norm_est.setNumberOfThreads(8);
	norm_est.setSearchMethod(tree);
	norm_est.setRadiusSearch(scene_ss_);
	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_keyNormals);
	clock_t t3 = clock();
	cout << "t: " << t3 - t21 << endl;
	// show keypoints
	pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
	//scene
	keyPointVisual.addPointCloud(scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_keypoints, 0.0, 255.0, 0.0), "scene_keypoints");
	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
	keyPointVisual.addPointCloudNormals<PointType, NormalType>(scene_keypoints, scene_keyNormals, 1, 10, "scene_normals");
	keyPointVisual.spin();

	//sample again for reference point
	sampled_index_ptr = zyk::uniformDownSamplePoint(scene_keypoints, 2.5 * scene_ss_, scene_keypoints_reference);
	cout << "keypoints number: " << scene_keypoints->size() << " reference number: " << sampled_index_ptr->size() << endl;
//	pcl::PPFEstimation ppfe;
	clock_t t4 = clock();
	long long scene_size = scene_keypoints->size(), reference_size = sampled_index_ptr->size();
	search::KdTree<PointType> kd_tree;
	kd_tree.setInputCloud(scene_keypoints);
	vector<vector<PPF>> ppfs(reference_size);
#pragma omp parallel for shared(ppfs) num_threads(8)
	for (int i = 0; i < sampled_index_ptr->size(); ++i)
	{
		PointType& rp = scene_keypoints->at(sampled_index_ptr->at(i));
		NormalType& rn = scene_keyNormals->at(sampled_index_ptr->at(i));
		vector<int> idxes;
		vector<float> dists;
		kd_tree.radiusSearch(rp, 141.505, idxes, dists);
		ppfs[i] = vector<PPF>(idxes.size());
		for (int j=0;j<idxes.size();++j)
		{
			int idx = idxes[j];
			PointType& sp = scene_keypoints->at(idx);
			NormalType& sn = scene_keyNormals->at(idx);
			computeSinglePPF(rp, rn, sp, sn, ppfs[i][j]);
		}
	}
	clock_t t5 = clock();
	cout << "ppf time: " << t5 - t4 << endl;

	//check
	//for (auto& v : ppfs)
	//{
	//	for (auto&ppf : v)
	//		if (ppf.valid == false)
	//			cout << "error" << endl;
	//}
	system("pause");
	return 0;
}