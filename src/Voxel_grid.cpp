#include "Voxel_grid.h"

zyk::CVoxel_grid::CVoxel_grid(int32_t x_div, int32_t y_div, int32_t z_div, pcl::PointCloud<PointType>::Ptr pntcloud)
{
	assert(x_div >= 1 && y_div >= 1 && z_div >= 1);
	grid_x_div = x_div;
	grid_y_div = y_div;
	grid_z_div = z_div;
	
	grid_div_mul(0) = 1;
	grid_div_mul(1) = x_div;
	grid_div_mul(2) = x_div*y_div;
	total_box_num = grid_div_mul(0)*z_div;
	assert(total_box_num > 0);
	input_point_cloud = pntcloud;
	box_vector.resize(total_box_num, NULL);
	//bonding
	findBoundingBox();
	//属性计算，输入的是div，故计算size
	Eigen::Array3f dim = max_p - min_p;
	leaf_size(0) = dim(0) / grid_x_div;
	leaf_size(1) = dim(1) / grid_y_div;
	leaf_size(2) = dim(2) / grid_z_div;
	inv_leaf_size(0) = 1 / leaf_size(0);
	inv_leaf_size(1) = 1 / leaf_size(1);
	inv_leaf_size(2) = 1 / leaf_size(2);
	constructGrid();
}

zyk::CVoxel_grid::CVoxel_grid(float leaf_size_x, float leaf_size_y, float leaf_size_z, pcl::PointCloud<PointType>::Ptr pntcloud)
{
	assert(leaf_size_x > 0 && leaf_size_y > 0 && leaf_size_z > 0);
	//设置点云
	input_point_cloud = pntcloud;
	//bounding
	findBoundingBox();
	//计算grid的大小
	Eigen::Array3f dim = max_p - min_p;
	grid_x_div = ceil(dim(0) / leaf_size_x);
	grid_y_div = ceil(dim(1) / leaf_size_y);
	grid_z_div = ceil(dim(2) / leaf_size_z);
	grid_div_mul(0) = 1;
	grid_div_mul(1) = grid_x_div;
	grid_div_mul(2) = grid_x_div*grid_y_div;
	total_box_num = grid_div_mul(2)*grid_z_div;
	//重新计算size大小
	leaf_size(0) = dim(0) / grid_x_div;
	leaf_size(1) = dim(1) / grid_y_div;
	leaf_size(2) = dim(2) / grid_z_div;
	inv_leaf_size(0) = 1 / leaf_size(0);
	inv_leaf_size(1) = 1 / leaf_size(1);
	inv_leaf_size(2) = 1 / leaf_size(2);
	//构造box
	assert(total_box_num > 0);
	box_vector.resize(total_box_num, NULL);

	if(!constructGrid());
}

void zyk::CVoxel_grid::resplit(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
	assert(leaf_size_x > 0 && leaf_size_y > 0 && leaf_size_z > 0);
	Eigen::Array3f dim = max_p - min_p;
	grid_x_div = ceil(dim(0) / leaf_size_x);
	grid_y_div = ceil(dim(1) / leaf_size_y);
	grid_z_div = ceil(dim(2) / leaf_size_z);
	grid_div_mul(0) = 1;
	grid_div_mul(1) = grid_x_div;
	grid_div_mul(2) = grid_x_div*grid_y_div;
	total_box_num = grid_div_mul(2)*grid_z_div;

	leaf_size(0) = dim(0) / grid_x_div;
	leaf_size(1) = dim(1) / grid_y_div;
	leaf_size(2) = dim(2) / grid_z_div;

	inv_leaf_size(0) = 1 / leaf_size(0);
	inv_leaf_size(1) = 1 / leaf_size(1);
	inv_leaf_size(2) = 1 / leaf_size(2);

	assert(total_box_num > 0);
	for (int32_t i = 0; i < box_vector.size(); i++)
		if (box_vector[i] != NULL)  delete box_vector[i];
	box_vector.clear();
	box_vector.resize(total_box_num, NULL);
	if(!constructGrid());
}

zyk::CVoxel_grid::~CVoxel_grid()
{
	for (int32_t i = 0; i < box_vector.size(); i++)
		if(box_vector[i]!=NULL)  delete box_vector[i];
}

bool zyk::CVoxel_grid::findBoundingBox()
{
	if (input_point_cloud==NULL)
		return false;
	if (input_point_cloud->empty())
		return false;
	min_p.setConstant(FLT_MAX);
	max_p.setConstant(-FLT_MAX);
	for (int32_t i = 0; i < input_point_cloud->size(); i++)
	{
		PointType p = input_point_cloud->at(i);
		if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y) || !pcl_isfinite(p.z))
		{
			continue;
		}
		min_p = min_p.min(p.getArray3fMap());
		max_p = max_p.max(p.getArray3fMap());
	}
	
	return true;
}
void zyk::CVoxel_grid::getPntBoxCoord(PointType& pnt, int32_t* ijk)
{
	if (!pcl_isfinite(pnt.x) || !pcl_isfinite(pnt.y) || !pcl_isfinite(pnt.z))
		ijk[0] = ijk[1] = ijk[2] = -1;
	float a1 = (pnt.x - min_p[0])*inv_leaf_size[0];
	float a2 = (pnt.y - min_p[1])*inv_leaf_size[1];
	float a3 = (pnt.z - min_p[2])*inv_leaf_size[2];

	ijk[0] = floor(a1);
	ijk[1] = floor(a2);
	ijk[2] = floor(a3);

	if (ijk[0] < 0 || ijk[1] < 0 || ijk[2] < 0 || ijk[0] >= grid_x_div || ijk[1] >= grid_y_div || ijk[2] >= grid_z_div)
		ijk[0] = ijk[1] = ijk[2] = -1;
}

void zyk::CVoxel_grid::getPntBoxCoord(PointType& pnt, Eigen::Vector3i& ijk)
{
	if (!pcl_isfinite(pnt.x) || !pcl_isfinite(pnt.y) || !pcl_isfinite(pnt.z))
		ijk = Eigen::Vector3i(-1, -1, -1);
	Eigen::Array3f temp = (Eigen::Array3f(pnt.x, pnt.y, pnt.z)-min_p)*inv_leaf_size;
	ijk = Eigen::Vector3i(floor(temp(0)), floor(temp(1)), floor(temp(2)));
	if (ijk(0) < 0 || ijk(1) < 0 || ijk(2) < 0||ijk(0)>= grid_x_div||ijk(1)>= grid_y_div||ijk(2)>= grid_z_div)
		ijk = Eigen::Vector3i(-1, -1, -1);
}

int32_t zyk::CVoxel_grid::getPntBoxIndex(PointType& pnt)
{
#ifdef use_eigen
	Eigen::Vector3i ijk;
#else
	int32_t ijk[3];
#endif
	getPntBoxCoord(pnt, ijk);
#ifdef use_eigen
	int32_t i = ijk.dot(grid_div_mul);
#else
	int32_t i = ijk[0] * grid_div_mul[0] + ijk[1] * grid_div_mul[1] + ijk[2] * grid_div_mul[2];
#endif
	if (i >= total_box_num || i < 0)
		i = -1;
	return i;
}



void zyk::CVoxel_grid::getGridDiv(Eigen::Array3i& d)
{
	d = Eigen::Array3i(grid_x_div, grid_y_div, grid_z_div);
}

void zyk::CVoxel_grid::getGridDiv(Eigen::Vector3i& d)
{
	d = Eigen::Vector3i(grid_x_div, grid_y_div, grid_z_div);
}

vector<zyk::box*>* zyk::CVoxel_grid::getBox_vector()
{
	return &box_vector;
}

bool zyk::CVoxel_grid::constructGrid()
{
	if (input_point_cloud == NULL)
		return false;
	if (input_point_cloud->empty())
		return false;
	for (int32_t i = 0; i < input_point_cloud->size(); ++i)
	{
		int32_t index = getPntBoxIndex(input_point_cloud->at(i));
		if (index != -1)
		{
			if (box_vector[index] == NULL)
				box_vector[index] = new box;
			box_vector[index]->putin(i);
		}
	}
	return true;
}
