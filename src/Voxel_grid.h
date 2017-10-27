#pragma once
#include <vector>
#include "common.h"
using namespace std;
namespace zyk
{

	//�����3D�ռ������࣬��һ����ά��������ԱΪ���涨���box
	class ZYK_EXPORTS CVoxel_grid
	{
	public:
		//
		CVoxel_grid() {};
		void Init(int32_t x_div, int32_t y_div, int32_t z_div, pcl::PointCloud<PointType>::Ptr pntcloud);
		void Init(float leaf_size_x, float leaf_size_y, float leaf_size_z, pcl::PointCloud<PointType>::Ptr pntcloud);
		~CVoxel_grid();


	public:
		////////method
		
		////�����������box��grid����
		void getPntBoxCoord(PointType& pnt, int32_t* ijk);
		void getPntBoxCoord(PointType& pnt, Eigen::Vector3i& ijk);
		//����������ڵ�box��index
		int32_t getPntBoxIndex(PointType& pnt);
		
		//resplit
		void resplit(float leaf_size_x, float leaf_size_y, float leaf_size_z);

		////////property get and set
		void getMinPoint(Eigen::Array3f& p);
		void getMaxPoint(Eigen::Array3f& p);
		void getLeafSize(Eigen::Array3f& s);
		void getGridDiv(Eigen::Array3i& d);
		void getGridDiv(Eigen::Vector3i& d);
		vector<box*>* getBox_vector();
		int32_t getTotalBoxNum();

		pcl::PointCloud<PointType>::Ptr getInputPointCloud(){ return input_point_cloud; };
		void setInputPointCloud(pcl::PointCloud<PointType>::Ptr pc) { input_point_cloud = pc; };
	private:
		///////////////////////////////////////////
		//////////////��������/////////////////////
		///////////////////////////////////////////
		//grid����������Ļ�������
		int32_t grid_x_div;
		int32_t grid_y_div;
		int32_t grid_z_div;
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
		int32_t total_box_num;
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

}

