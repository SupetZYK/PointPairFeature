#pragma once
#include <vector>
#include "common.h"
using namespace std;


namespace zyk
{
	/*This is a struct used to store indexes
	**/
	struct box
	{
	public:
		void putin(int idx) { this->pnts_index.push_back(idx); };
		int size() { return this->pnts_index.size(); };
		//std::vector<int>*getIndices(){ return &pnts_index; };
		const int operator[](int i) const { return pnts_index[i]; };
	private:
		std::vector<int> pnts_index;
	};
	ZYK_EXPORTS void getNeiboringBoxIndex3D(int currentIndex, const Eigen::Vector3i& grid_div, std::vector<int>& out_vec);
	ZYK_EXPORTS void getNeiboringBoxIndex3D(const Eigen::Vector3i& currentCoord, const Eigen::Vector3i& grid_div, std::vector<int>& out_vec);
	/*Class used to separate a 3-dimmensional space into grids
	**/
	class ZYK_EXPORTS CVoxel_grid
	{
	public:
		//
		CVoxel_grid() {};
		void Init(int x_div, int y_div, int z_div, pcl::PointCloud<PointType>::Ptr pntcloud);
		void Init(float leaf_size_x, float leaf_size_y, float leaf_size_z, pcl::PointCloud<PointType>::Ptr pntcloud);
		~CVoxel_grid();

	public:
		////////method
		
		////�����������box��grid����
		void getPntBoxCoord(PointType& pnt, int* ijk);
		void getPntBoxCoord(PointType& pnt, Eigen::Vector3i& ijk);
		//����������ڵ�box��index
		int getPntBoxIndex(PointType& pnt);
		
		//resplit
		void resplit(float leaf_size_x, float leaf_size_y, float leaf_size_z);

		////////property get and set
		void getMinPoint(Eigen::Array3f& p);
		void getMaxPoint(Eigen::Array3f& p);
		void getLeafSize(Eigen::Array3f& s);
		void getGridDiv(Eigen::Array3i& d);
		void getGridDiv(Eigen::Vector3i& d);
		vector<box*>* getBox_vector();
		int getTotalBoxNum();

		pcl::PointCloud<PointType>::Ptr getInputPointCloud(){ return input_point_cloud; };
		void setInputPointCloud(pcl::PointCloud<PointType>::Ptr pc) { input_point_cloud = pc; };
	private:
		///////////////////////////////////////////
		//////////////��������/////////////////////
		///////////////////////////////////////////
		//grid����������Ļ�������
		int grid_x_div;
		int grid_y_div;
		int grid_z_div;
		//�洢box������,����x���ȱ仯��y��Σ�z������˳��洢��
		vector<box*> box_vector;
		//grid���������޵�����
		Eigen::Array3f min_p;
		Eigen::Array3f max_p;

		//ָ����Ƶ�ָ��
		pcl::PointCloud<PointType>::Ptr input_point_cloud;



		///////////////////////////////////////////
		//////////////�������ԣ����Լ�������ģ�
		///////////////////////////////////////////
		int total_box_num;
		//ÿһ��box��x��y��z�Ĵ�С
		Eigen::Array3f leaf_size;
		Eigen::Array3f inv_leaf_size;
		//Ϊ�˸���box��grid������vector�е�index�������һ������
		Eigen::Vector3i grid_div_mul;
		//////////////////////���صķ���
		bool constructGrid();
		//Ѱ��������Ƶ�bounding box���������е������������ֵ��
		bool findBoundingBox();
	};
	/*Class that used to iterate the neiboring of a specific voxel in N-dimmensional space
	**/
	class NeiboringIterator 
	{
	public:
		/*Default construct
		**/
		NeiboringIterator();
		/* construct function
		*@ Size, the size of the space
		*@ spaceDim, the dimmension of the space
		**/
		NeiboringIterator(int* Size, int spaceDim);
		/*Set the target voxel index
		*@ targetIndex, the position index of the target, for example, in 3D space, index of (x,y,z) is x+y*(size0)+z*(size0*size1);
		**/
		void setTarget(int targetIndex);
		void setTarget(int* pos);
		/*Default deconstruct
		**/
		~NeiboringIterator();
		/*Jump to next position
		**/
		NeiboringIterator& operator++ ();
		/*function used to signify whether the loop is done
		**/
		bool isDone() { return isdone; };
		/*Get the current index of pos
		**/
		int getIndex();
	private:
		int* spaceSize;
		int spaceDimmension;
		int* curPos;
		int* targetPos;
		int* div_mul;
		int maxIndex;
		bool isdone;
		bool selfIncludeFlag;
	};

}

