#pragma once
#include <vector>
#include "common.h"
using namespace std;
namespace zyk
{

	//整体的3D空间网格类，是一个三维容器，成员为上面定义的box
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
		
		////求输入点所在box的grid坐标
		void getPntBoxCoord(PointType& pnt, int32_t* ijk);
		void getPntBoxCoord(PointType& pnt, Eigen::Vector3i& ijk);
		//求输入点所在的box的index
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
		//////////////基本属性/////////////////////
		///////////////////////////////////////////
		//grid的三个方向的划分数量
		int32_t grid_x_div;
		int32_t grid_y_div;
		int32_t grid_z_div;
		//存储box的容器,按照x优先变化，y其次，z最外层的顺序存储。
		vector<box*> box_vector;
		//grid的两个极限点坐标
		Eigen::Array3f min_p;
		Eigen::Array3f max_p;

		//指向点云的指针
		pcl::PointCloud<PointType>::Ptr input_point_cloud;



		///////////////////////////////////////////
		//////////////辅助属性（可以计算出来的）
		///////////////////////////////////////////
		int32_t total_box_num;
		//每一个box的x，y，z的大小
		Eigen::Array3f leaf_size;
		Eigen::Array3f inv_leaf_size;
		//为了根据box的grid坐标求vector中的index而保存的一个乘子
		Eigen::Vector3i grid_div_mul;
		//////////////////////隐藏的方法
		bool constructGrid();
		//寻找输入点云的bounding box，并给类中的两个点变量赋值。
		bool findBoundingBox();
	};

}

