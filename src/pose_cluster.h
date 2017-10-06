#include "common.h"
#pragma once

namespace zyk
{

	class ZYK_EXPORTS pose_cluster
	{
		
	public:
		Eigen::Affine3f mean_transformation;
		//¸¨Öú
		Eigen::Vector3f mean_rot, mean_trans, first_trans, first_axis;
		float first_angle;
		

	public:
		//pose_cluster(){};
	  pose_cluster(const Eigen::Affine3f& transformation, float vote){ checkAndPutIn(transformation, vote, 100, 100); };
		~pose_cluster(){};
		bool checkAndPutIn(const Eigen::Affine3f& transformation, float vote, float distance_thresh, float angle_thresh);
		int32_t size(){ return transformations.size(); };
		bool empty() { return transformations.empty(); };
		float getVote() { return vote_count; };
		void getMeanTransformation(Eigen::Affine3f& transformation) { transformation = mean_transformation; };

		void checkDisAndRemove(int index,float thresh);
		vector < Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > transformations;
		vector < int> voteLists;
		float vote_count = 0.0;
	};


	inline bool pose_cluster_comp(const zyk::pose_cluster&a, const zyk::pose_cluster&b){ return a.vote_count > b.vote_count; };

}
