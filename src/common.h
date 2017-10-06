#pragma once
#include <string>

//#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
//#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/shot_omp.h>
//#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> //iterative closet point
//#include <pcl/recognition/hv/hv_go.h> //hypothesis verification
//#include <pcl/keypoints/iss_3d.h> //iss method key point detect
//#include <pcl/features/rops_estimation.h> //rops feature
//#include <pcl/keypoints/harris_3d.h> //harris keypoint
//#include <pcl/surface/gp3.h> //triangularization point cloud
//#include <pcl/features/fpfh_omp.h>
#include <pcl/features/ppf.h>
#include <pcl/features/principal_curvatures.h>
#include "stdint.h"

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)&& defined zyk_ppf_EXPORTS
# define ZYK_EXPORTS  __declspec(dllexport)
#elif defined __GNUC__ && __GNUC__ >= 4
#  define ZYK_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define ZYK_EXPORTS
#endif

//#define use_eigen

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;

using namespace std;
using namespace pcl;
namespace zyk
{
	struct box
	{
		//存储盒子中的点的index
	public:
		void putin(int32_t idx) { this->pnts_index.push_back(idx); };
		int32_t size() { return this->pnts_index.size(); };
		//std::vector<int32_t>*getIndices(){ return &pnts_index; };
		const int32_t operator[](int32_t i) const { return pnts_index[i]; };
	private:
		std::vector<int32_t> pnts_index;
	};

	ZYK_EXPORTS void getNeiboringBoxIndex3D(int32_t currentIndex, const Eigen::Vector3i& grid_div, vector<int32_t>& out_vec);
	ZYK_EXPORTS void getNeiboringBoxIndex3D(const Eigen::Vector3i& currentCoord, const Eigen::Vector3i& grid_div, vector<int32_t>& out_vec);
}
//ZYK_EXPORTS bool downSamplePointCloud(const pcl::PointCloud<PointType>::Ptr &scene, const double relSamplingDistance,
//	pcl::PointCloud<PointType>::Ptr &outCloud, const int method = 1);
ZYK_EXPORTS pcl::IndicesPtr uniformDownSamplePoint(pcl::PointCloud<PointType>::Ptr pointcloud, double relSamplingDistance, pcl::PointCloud<PointType>::Ptr outCloud);
ZYK_EXPORTS pcl::IndicesPtr uniformDownSamplePointAndNormal(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, double relSamplingDistance,
	pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNormal);
ZYK_EXPORTS bool SmartDownSamplePointAndNormal(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, double relSamplingDistance,
	pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNormal);
ZYK_EXPORTS double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud, double max_coord[3] = NULL, double min_coord[3] = NULL);
ZYK_EXPORTS bool readPointCloud(std::string filename, std::string format, PointCloud<PointType>::Ptr outCloud, PointCloud<NormalType>::Ptr outNor = NULL);
//ZYK_EXPORTS void ISSmethod(const PointCloud<PointType>::Ptr &inCloud, double salientRatio, double NMPratio, PointCloud<PointType>::Ptr &outCloud);


void ZYK_EXPORTS transformNormals(const pcl::PointCloud<NormalType>&normals_in, pcl::PointCloud<NormalType>&normals_out, const Eigen::Affine3f& transform);


//replace eigen
inline double dot(const NormalType& n1, const NormalType&n2){ return n1.normal_x*n2.normal_x + n1.normal_y*n2.normal_y + n1.normal_z*n2.normal_z; };
double dot(const float* n1, const float* n2, const int dim);
double norm(const float* n, const int dim);
double dist(const float* n1, const float* n2, const int dim);