#include "Voxel_grid.h"
#include <string.h>

void zyk::CVoxel_grid::Init(int x_div, int y_div, int z_div, pcl::PointCloud<PointType>::Ptr pntcloud)
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

void zyk::CVoxel_grid::Init(float leaf_size_x, float leaf_size_y, float leaf_size_z, pcl::PointCloud<PointType>::Ptr pntcloud)
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

	if (!constructGrid()) {
		std::cout << "ERROR DURING CONSTRUCT!" << std::endl;
		system("pause");
		exit(-1);
	}
}

void zyk::CVoxel_grid::resplit(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
	assert(leaf_size_x > 0 && leaf_size_y > 0 && leaf_size_z > 0);
	if (abs(leaf_size_x - leaf_size(0)) / leaf_size(0) < 0.05 && abs(leaf_size_y - leaf_size(1)) / leaf_size(1) < 0.05 && abs(leaf_size_z - leaf_size(2)) / leaf_size(2) < 0.05)
	{
		cout << "No need to resplit grid!" << endl;
		return;
	}
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
	for (int i = 0; i < box_vector.size(); i++)
		if (box_vector[i] != NULL)  delete box_vector[i];
	box_vector.clear();
	box_vector.resize(total_box_num, NULL);
	if (!constructGrid()) {
		std::cout << "ERROR DURING CONSTRUCT!" << std::endl;
		system("pause");
		exit(-1);
	}
}

zyk::CVoxel_grid::~CVoxel_grid()
{
	for (int i = 0; i < box_vector.size(); i++)
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
	for (int i = 0; i < input_point_cloud->size(); i++)
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
void zyk::CVoxel_grid::getPntBoxCoord(PointType& pnt, int* ijk)
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

int zyk::CVoxel_grid::getPntBoxIndex(PointType& pnt)
{
#ifdef use_eigen
	Eigen::Vector3i ijk;
#else
	int ijk[3];
#endif
	getPntBoxCoord(pnt, ijk);
#ifdef use_eigen
	int i = ijk.dot(grid_div_mul);
#else
	int i = ijk[0] * grid_div_mul[0] + ijk[1] * grid_div_mul[1] + ijk[2] * grid_div_mul[2];
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
	for (int i = 0; i < input_point_cloud->size(); ++i)
	{
		int index = getPntBoxIndex(input_point_cloud->at(i));
		if (index != -1)
		{
			if (box_vector[index] == NULL)
				box_vector[index] = new box;
			box_vector[index]->putin(i);
		}
	}
	return true;
}

zyk::NeiboringIterator::NeiboringIterator()
	: spaceDimmension(2)
	, maxIndex(1)
	, spaceSize(2, 1)
	, curPos(2, 0)
	, curIndex(0)
	, targetPos(2, 0)
	, targetIndex(0)
	, div_mul(2, 1)
	, stateFlag(0)
	, min_(2, 0)
	, max_(2, 1)
{
}


zyk::NeiboringIterator::NeiboringIterator(int* Size, int spaceDim)
	: spaceDimmension(spaceDim)
	, maxIndex(1)
	, spaceSize(spaceDim, 1)
	, curPos(spaceDim, 0)
	, curIndex(0)
	, targetPos(spaceDim, 0)
	, targetIndex(0)
	, div_mul(spaceDim, 1)
	, stateFlag(0)
	, min_(spaceDim, 0)
	, max_(spaceDim, 1)
{
	//error input check
	assert(Size != NULL && spaceDim>0);

	div_mul[0] = 1;
	for (int i = 0; i < spaceDimmension; ++i) {	
		spaceSize[i] = Size[i];
		if(i<spaceDimmension-1)
			div_mul[i + 1] = div_mul[i] * spaceSize[i];

		assert(div_mul[i] > 0);
		max_[i] = Size[i] - 1;
	}
	maxIndex *= div_mul[spaceDimmension-1]*spaceSize[spaceDimmension-1];
}

zyk::NeiboringIterator::~NeiboringIterator()
{
}

zyk::NeiboringIterator& zyk::NeiboringIterator::operator++()
{
	if (!isDone()) {
		if (firstOperation()) {
			for (int i = 0; i < spaceDimmension; ++i) {
				curPos[i] = min_[i];
			}
			int res = 0;
			for (int i = 0; i < spaceDimmension; ++i) {
				res += curPos[i] * div_mul[i];
			}
			curIndex = res;
			stateFlag |= 0x02;
		}
		else 
			for (int i = 0; i < spaceDimmension; ++i) {
				if (++curPos[i]>max_[i]) {
					if (i == spaceDimmension - 1) {
						stateFlag |= 0x01;
						break;
					}
					curPos[i] = min_[i];
					curIndex -= (max_[i] - min_[i])*div_mul[i];
				}
				else {
					curIndex += div_mul[i];
					break;
				}
					
			}
		//int res = 0;
		//for (int i = 0; i < spaceDimmension; ++i) {
		//	res += curPos[i] * div_mul[i];
		//}
		//curIndex = res;
		if (curIndex == targetIndex)
			return(++(*this));
	}
	return *this;
}

int zyk::NeiboringIterator::getIndex()
{
	//int res = 0;
	//for (int i = 0; i < spaceDimmension; ++i) {
	//	res += div_mul[i] * curPos[i];
	//}
	//if (!firstOperation() && res == targetIndex) {
	//	++(*this);
	//	res = getIndex();
	//}
	//return res;
	return curIndex;
}

void zyk::NeiboringIterator::setTarget(int targetIdx)
{
	assert(targetIdx < maxIndex && targetIdx >= 0);
	targetIndex = targetIdx;
	curIndex = targetIndex;
	for (int i = 0; i < spaceDimmension; ++i) {
		int idx = spaceDimmension - i - 1;
		targetPos[idx] = targetIdx / div_mul[idx];
		if (i < spaceDimmension - 1)
			targetIdx -= targetPos[idx] * div_mul[idx];
		curPos[idx] = targetPos[idx];
		min_[idx] = std::max(0, targetPos[idx] - 1);
		max_[idx] = std::min(spaceSize[idx] - 1, targetPos[idx] + 1);
	}
	stateFlag = 0;
}


void zyk::NeiboringIterator::setTarget(int* pos)
{
	int tmp = 0;
	for (int i = 0; i < spaceDimmension; ++i) {
		assert(pos[i] >= 0 && pos[i] < spaceSize[i]);
		targetPos[i] = pos[i];
		tmp += targetPos[i] * div_mul[i];

		min_[i] = std::max(0, targetPos[i] - 1);
		max_[i] = std::min(spaceSize[i] - 1, targetPos[i] + 1);
	}
	targetIndex = tmp;
	stateFlag = 0;
}

void zyk::getNeiboringBoxIndex3D(const Eigen::Vector3i& currentCoord, const Eigen::Vector3i& grid_div, vector<int>& out_vec)
{
	vector<int>box_index;
	Eigen::Vector3i grid_div_mul(1, grid_div(0), grid_div(0)*grid_div(1));
	for (int i = -1; i < 2; i++)
	{
		if (currentCoord(0) + i < 0 || currentCoord(0) + i >= grid_div(0))
			continue;
		for (int j = -1; j < 2; j++)
		{
			if (currentCoord(1) + j < 0 || currentCoord(1) + j >= grid_div(1))
				continue;
			for (int k = -1; k < 2; k++)
			{
				if (currentCoord(2) + k < 0 || currentCoord(2) + k >= grid_div(2))
					continue;
				if (i == 0 && j == 0 && k == 0)
					continue;
				out_vec.push_back((currentCoord + Eigen::Vector3i(i, j, k)).dot(grid_div_mul));
			}
		}
	}
}

void zyk::getNeiboringBoxIndex3D(int currentIndex, const Eigen::Vector3i& grid_div, vector<int>& out_vec)
{
	Eigen::Vector3i div_mul(1, grid_div(0), grid_div(0)*grid_div(1));
	int k = currentIndex / div_mul(2);
	currentIndex -= k*div_mul(2);
	int j = currentIndex / div_mul(1);
	currentIndex -= j*div_mul(1);
	int i = currentIndex;
	Eigen::Vector3i coord(i, j, k);
	getNeiboringBoxIndex3D(coord, grid_div, out_vec);
}