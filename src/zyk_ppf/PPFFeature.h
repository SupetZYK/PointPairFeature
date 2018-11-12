#pragma once
#include <vector>
#include <stdint.h>
#include <util.h>
#include <util_pcl.h>
#include "pose_cluster.h"
#include "Voxel_grid.h"

#include "boost/serialization/serialization.hpp"  
#include "boost/archive/binary_oarchive.hpp"  
#include "boost/archive/binary_iarchive.hpp"  
#include <boost/serialization/export.hpp> 

//some option for test
//#define use_eigen
#define use_neiboringIterator
#define vote_flag_use_map
//#define view_based
//#define plane_check
namespace zyk
{
	struct ZYK_EXPORTS PPF
	{
		int first_index;
		int second_index;
		pcl::PPFSignature ppf;
		float weight;
		PPF() :weight(1.0),first_index(-1),second_index(-1) {};
	};

	struct PPF_Accumulator
	{
        PPF_Accumulator(int pnt_number, int rot_angle_div){ acumulator = Eigen::MatrixXf(pnt_number, rot_angle_div); acumulator.setConstant(0); }
		Eigen::MatrixXf acumulator;
	};

	class ZYK_EXPORTS PPF_Space
	{
	public:
		PPF_Space();
		~PPF_Space();
	public:
		//construct
    bool init(std::string Name, pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, int angle_div, int distance_div, bool ignore_plane=false);
#ifdef view_based
    bool init(std::string Name, pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, std::vector<std::vector<int> >&view_based_indexes,int angle_div, int distance_div, bool ignore_plane=false);
#endif
		void clear();
		////////////////////
		////// property access
		////////////////////
        const string getName() const { return mName; }
        const double getMaxPPFD() const { return max_p[3]; }
		double getDiameter() const { return zyk::norm(model_size, 3); }
		const int getNumberPoints() const {return input_point_cloud->size();}
    const double getSampleRatio() const { return model_res / (zyk::norm(model_size, 3)); }
	public:
		////////////////////
		//////methods
		////////////////////

		static float computeAlpha(const PointType& first_pnt, const NormalType& first_normal, const PointType& second_pnt);
		static float computeAlpha(const Eigen::Vector3f& first_pnt, const Eigen::Vector3f& first_normal, const Eigen::Vector3f& second_pnt);
		static void computeSinglePPF(const PointType& first_pnt, const NormalType& first_normal, const PointType& second_pnt, const NormalType& second_normal, zyk::PPF& ppf);
		static void computeSinglePPF(const Eigen::Vector3f& first_pnt, const Eigen::Vector3f& first_normal, const Eigen::Vector3f& second_pnt, const Eigen::Vector3f& second_normal,zyk::PPF& ppf);
		// for train only, in match step, use the above two.
		static void computeSinglePPF(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormalType, int32_t index1, int32_t index2, PPF& ppf);
		static bool getPoseFromPPFCorresspondence(PointType& model_point, NormalType& model_normal, PointType& scene_point, NormalType&scene_normal, float alpha, Eigen::Affine3f& transformation);
		//test speed
		void getppfBoxCoord(PPF& ppf, Eigen::Vector4i& ijk);
		void getppfBoxCoord(PPF& ppf, int* ijk);
		int getppfBoxIndex(PPF& ppf);
        vector<box*>* getBoxVector() { return &ppf_box_vector; }
        vector<PPF>* getPPFVector() { return &ppf_vector; }

		//in order to spread discretized ppf, use this to get neighboring ppf box
		void getNeighboringPPFBoxIndex(int currentIndex,vector<int>&out_vec);
        void getModelPointCloud(pcl::PointCloud<PointType>::Ptr&pointcloud){ pointcloud = input_point_cloud; }
        void getCenteredPointCloud(pcl::PointCloud<PointType>::Ptr&pointcloud){ pointcloud = centered_point_cloud; }
        void getPointNormalCloud(pcl::PointCloud<NormalType>::Ptr&pointNormals){ pointNormals = input_point_normal; }

		friend class boost::serialization::access;
		
		//for test only
		float computeClusterScore(pcl::PointCloud<PointType>::Ptr& scene, pcl::PointCloud<NormalType>::Ptr& scene_normals, float dis_thresh, float ang_thresh, zyk::pose_cluster &pose_clusters);
		
		////////match
        void match(pcl::PointCloud<PointType>::Ptr scene,
                   pcl::PointCloud<NormalType>::Ptr scene_normals,
                   bool spread_ppf_switch,
                   bool two_ball_switch,
					bool use_weighted_vote,
                   float relativeReferencePointsNumber,
                   float max_vote_thresh,
                   float max_vote_percentage,
					int n_angles, //drost use 30
                   float angle_thresh,
                   float first_dis_thresh,
                   float recompute_score_dis_thresh,
                   float recompute_score_ang_thresh,
                   float max_overlap_ratio,
                   int num_clusters_per_group,
                   vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> >& pose_clusters);
		void match_v2(pcl::PointCloud<PointType>::Ptr scene,
						pcl::PointCloud<NormalType>::Ptr scene_normals,
						bool spread_ppf_switch,
						bool two_ball_switch,
						bool use_weighted_vote,
						float relativeReferencePointsNumber,
						float max_vote_thresh,
						float max_vote_percentage,
						int n_angles, //drost use 30
						float angle_thresh,
						float first_dis_thresh,
						float recompute_score_dis_thresh,
						float recompute_score_ang_thresh,
						float max_overlap_ratio,
						int num_clusters_per_group,
						vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> >& pose_clusters);
#ifdef plane_check
        //plane checker test 2018-3-14
        void setPlaneFlag(std::vector<bool >& flag){plane_flag=flag;ver_=3;}
        std::vector<bool >plane_flag;
        int plane_vote_thresh;
#endif
		///////USER IO
		bool save(std::string file_name);
		bool load(std::string fine_name);

		//bool model_x_centrosymmetric = false;
		//bool model_y_centrosymmetric = false;
		//bool model_z_centrosymmetric = false;

		//
		//model property
		//
		float model_size[3];
		float model_res;
		bool ignore_plane_switch;

		//////// ICP test version
    void ICP_Refine(pcl::PointCloud<PointType>::Ptr scene, const vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > &coarse_pose_clusters, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > &pose_clusters_out, int max_number, double scene_res=-1.0, double max_dis=-1.0);
    void ICP_Refine2_0(pcl::PointCloud<PointType>::Ptr scene, const vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > &coarse_pose_clusters, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > &pose_clusters_out, int max_number, double scene_res = -1.0, double max_dis = -1.0);

	protected:
		///////////////IO
		template<class Archive>
		void save(Archive& ar, const unsigned int version);
		template<class Archive>
		void load(Archive& ar, const unsigned int version);
		BOOST_SERIALIZATION_SPLIT_MEMBER()

		/////////////METHODS
		void recomputeClusterScore(zyk::CVoxel_grid& grid, pcl::PointCloud<NormalType>& scene_normals, float dis_thresh, float ang_thresh, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > &pose_clusters);
		void recomputeClusterScore_v2(zyk::CVoxel_grid& grid, pcl::PointCloud<NormalType>& scene_normals, float dis_thresh, float ang_thresh, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster> > &pose_clusters);
		bool computeAllPPF();
#ifdef view_based
    bool computeALL_Visible_PPF(std::vector<std::vector<int> >&view_based_indexes);
#endif
		bool findBoundingBox();
    bool constructGrid(int angle_div,int distance_div);
	protected:
		////////////////////
		//////basic property
		////////////////////
		std::string mName;
        int ver_;
		//for fast scene point access
		//CVoxel_grid scene_grid;
#ifdef view_based
        std::vector<float> occlusion_weights;
#endif
		//ptr to data
		pcl::PointCloud<PointType>::Ptr input_point_cloud;
		pcl::PointCloud<PointType>::Ptr centered_point_cloud;
		pcl::PointCloud<NormalType>::Ptr input_point_normal;
		//grid property
		int grid_f1_div;
		int grid_f2_div;
		int grid_f3_div;
		int grid_f4_div;
		//grids two limits
		Eigen::Array4f min_p;
		Eigen::Array4f max_p;
		//PPF container
		vector<PPF> ppf_vector;
		//ppf_box container
		vector<box*> ppf_box_vector;

		///////////////////////////////////////////
		////////////// other property
		///////////////////////////////////////////
		int32_t total_box_num;
		//size
		Eigen::Array4f leaf_size;
		Eigen::Array4f inv_leaf_size;
		//multiplier
		Eigen::Vector4i grid_div_mul;
		//Eigen::Vector3f point_cloud_center;
		//model cloud center
		double cx, cy, cz;
	};



}

template<class Archive>
void zyk::PPF_Space::save(Archive& ar, const unsigned int version)
{
    ar & ver_;
	ar & mName;
    ar & grid_f1_div;
    ar & grid_f4_div;
	ar & model_res;
	int ppf_vector_size=ppf_vector.size();
	ar & ppf_vector_size;
	for (int32_t i = 0; i < ppf_vector.size(); i++)
	{
		ar & ppf_vector[i].first_index;
		ar & ppf_vector[i].second_index;
		ar & ppf_vector[i].weight;
		ar & ppf_vector[i].ppf.alpha_m;
		ar & ppf_vector[i].ppf.f1;
		ar & ppf_vector[i].ppf.f2;
		ar & ppf_vector[i].ppf.f3;
		ar & ppf_vector[i].ppf.f4;
	}
	int point_num= input_point_cloud->size();
	ar & point_num;
	for (int32_t i = 0; i < input_point_cloud->size(); i++)
	{
		ar & input_point_cloud->at(i).x;
		ar & input_point_cloud->at(i).y;
		ar & input_point_cloud->at(i).z;
		ar & input_point_normal->at(i).normal_x;
		ar & input_point_normal->at(i).normal_y;
		ar & input_point_normal->at(i).normal_z;
#ifdef view_based
        if(ver_==2){
            ar & occlusion_weights[i];
        }
#endif
#ifdef plane_check
        if(ver_==3){
            int flag=plane_flag[i];
            ar & flag;
        }
#endif
	}

}

template<class Archive>
void zyk::PPF_Space::load(Archive& ar, const unsigned int version)
{
	int32_t ppf_vector_size = 0;
    ar & ver_;
	ar & mName;
    ar & grid_f1_div;
    ar & grid_f4_div;
	ar & model_res;
	ar & ppf_vector_size;
	ppf_vector.resize(ppf_vector_size);
	for (int32_t i = 0; i < ppf_vector_size; i++)
	{
		ar & ppf_vector[i].first_index;
		ar & ppf_vector[i].second_index;
		ar & ppf_vector[i].weight;
		ar & ppf_vector[i].ppf.alpha_m;
		ar & ppf_vector[i].ppf.f1;
		ar & ppf_vector[i].ppf.f2;
		ar & ppf_vector[i].ppf.f3;
		ar & ppf_vector[i].ppf.f4;
	}
	int32_t points_num = 0;
	ar & points_num;
	input_point_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	input_point_normal = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
	for (int32_t i = 0; i < points_num; i++)
	{
		PointType _tem;
		NormalType _nor;

		ar & _tem.x;
		ar & _tem.y;
		ar & _tem.z;
		ar & _nor.normal_x;
		ar & _nor.normal_y;
		ar & _nor.normal_z;
		input_point_cloud->push_back(_tem);
		input_point_normal->push_back(_nor);
#ifdef view_based
        if(ver_==2){//this version add view based weight
            float weight;
            ar & weight;
            occlusion_weights.push_back(weight);
        }
#endif
#ifdef plane_check
        if(ver_==3){//this version add plane check
            int isplane;
            ar&isplane;
            plane_flag.push_back(isplane);
        }
#endif
	}

  constructGrid(grid_f1_div,grid_f4_div);
}
