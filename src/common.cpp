#include "common.h"
#include <pcl/io/vtk_io.h>
#include "SmartSampling.hpp"
#include "cv.h"
double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud, double max_coord[3], double min_coord[3])
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);

	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);
	if (cloud->size() <= 0)
		return -1;
	if(max_coord!=NULL && min_coord!=NULL)
	{
		max_coord[0] = cloud->at(0).x;
		min_coord[0] = cloud->at(0).x;
		max_coord[1] = cloud->at(0).y;
		min_coord[1] = cloud->at(0).y;
		max_coord[2] = cloud->at(0).z;
		min_coord[2] = cloud->at(0).z;
	}
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
		//zyk max min
		if(max_coord==NULL || min_coord==NULL)
			continue;
		if (cloud->at(i).x > max_coord[0])
			max_coord[0] = cloud->at(i).x;
		else if (cloud->at(i).x<min_coord[0])
			min_coord[0] = cloud->at(i).x;
		if (cloud->at(i).y > max_coord[1])
			max_coord[1] = cloud->at(i).y;
		else if (cloud->at(i).y<min_coord[1])
			min_coord[1] = cloud->at(i).y;
		if (cloud->at(i).z > max_coord[2])
			max_coord[2] = cloud->at(i).z;
		else if (cloud->at(i).z < min_coord[2])
			min_coord[2] = cloud->at(i).z;
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}
//pcl::IndicesPtr test_return()
//{
//	pcl::IndicesPtr ptr (new pcl::IndicesPtr());
//	return ptr;
//}
pcl::IndicesPtr uniformDownSamplePoint(pcl::PointCloud<PointType>::Ptr pointcloud, double relSamplingDistance, pcl::PointCloud<PointType>::Ptr outCloud)
{
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(pointcloud);
	uniform_sampling.setRadiusSearch(relSamplingDistance);
	uniform_sampling.filter(*outCloud);
	return uniform_sampling.getSelectedIndex();

}

pcl::IndicesPtr uniformDownSamplePointAndNormal(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, double relSamplingDistance,
	pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNormal)
{
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(pointcloud);
	uniform_sampling.setRadiusSearch(relSamplingDistance);
	uniform_sampling.filter(*outCloud);
	pcl::IndicesPtr selectedIndex = uniform_sampling.getSelectedIndex();

	outNormal->height = 1;
	outNormal->is_dense = true;
	outNormal->points.resize(selectedIndex->size());
	for (int32_t i = 0; i < selectedIndex->size(); i++)
		outNormal->points[i]= pointNormal->points[selectedIndex->at(i)];
	return selectedIndex;
}

bool SmartDownSamplePointAndNormal(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, double relSamplingDistance,
	pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNormal)
{
	pcl::SmartSampling<PointType,NormalType> sampling;
	sampling.setInputCloud(pointcloud);
	sampling.setNormals(pointNormal);
	sampling.setRadiusSearch(relSamplingDistance);
	sampling.setAngleThresh(0.08);
	sampling.filter(*outCloud);
	vector<size_t>& selectedIndex = sampling.getSelectedIndex();

	outNormal->height = 1;
	outNormal->is_dense = true;
	outNormal->points.resize(selectedIndex.size());
	for (int32_t i = 0; i < selectedIndex.size(); i++)
		outNormal->points[i] = pointNormal->points[selectedIndex[i]];
	return true;
}

//bool downSamplePointCloud(const pcl::PointCloud<PointType>::Ptr &scene, const double relSamplingDistance,
//	pcl::PointCloud<PointType>::Ptr &outCloud, const int method)
//{
//	if (method == 1)
//	{
//		pcl::UniformSampling<PointType> uniform_sampling;
//		uniform_sampling.setInputCloud(scene);
//		uniform_sampling.setRadiusSearch(relSamplingDistance);
//		uniform_sampling.filter(*outCloud);
//		return true;
//	}
//	if (method == 2)
//	{
//		pcl::VoxelGrid<PointType> sor;
//		sor.setInputCloud(scene);
//		sor.setLeafSize(relSamplingDistance, relSamplingDistance, relSamplingDistance);
//		sor.filter(*outCloud);
//	}
//}

bool readPointCloud(std::string filename, std::string format, pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNor)
{
	if (format == "ply")
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr _Pcloud(new pcl::PointCloud<pcl::PointNormal>);
		if (pcl::io::loadPLYFile<pcl::PointNormal>(filename, *_Pcloud) < 0)
		{
			pcl::console::print_error("Error loading object/scene file!\n");
			return false;
		}
		outCloud->clear();
		if (outNor != NULL)outNor->clear();
		for (int pnum = 0; pnum < _Pcloud->points.size(); ++pnum)
		{
			PointType _tem;
			_tem.x = _Pcloud->points[pnum].x;
			_tem.y = _Pcloud->points[pnum].y;
			_tem.z = _Pcloud->points[pnum].z;
			outCloud->push_back(_tem);
			if (outNor != NULL)
			{
				NormalType _nor;
				_nor.normal_x = _Pcloud->points[pnum].normal_x;
				_nor.normal_y = _Pcloud->points[pnum].normal_y;
				_nor.normal_z = _Pcloud->points[pnum].normal_z;
				outNor->push_back(_nor);
			}
		}
	}
	else if (format == "pcd")
	{
		/*if (pcl::io::loadPCDFile(filename, *outCloud) < 0)
		{
		pcl::console::print_error("Error loading object/scene file!\n");
		return false;
		}*/
	}
	else
	{
		return false;
	}
	return true;
}

//void ISSmethod(const PointCloud<PointType>::Ptr &inCloud, double salientRatio, double NMPratio, PointCloud<PointType>::Ptr &outCloud)
//{
//	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
//	tree->setInputCloud(inCloud);
//	// Fill in the model cloud
//	//float model_resolution = resolution;
//	// Compute model_resolution
//	pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;
//	iss_detector.setSearchMethod(tree);
//	iss_detector.setSalientRadius(salientRatio );//ori---6 越小，关键点越多  the spherical neighborhood used to compute the scatter matrix
//	iss_detector.setNonMaxRadius(NMPratio);//ori---4 同上   The non maxima suppression radius
//	iss_detector.setThreshold21(0.975);//old---0.975
//	iss_detector.setThreshold32(0.975);//old---0.975
//	iss_detector.setMinNeighbors(5);//Minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
//	iss_detector.setNumberOfThreads(4);
//	iss_detector.setInputCloud(inCloud);
//	iss_detector.compute(*outCloud);
//
//}

void zyk::getNeiboringBoxIndex3D(const Eigen::Vector3i& currentCoord, const Eigen::Vector3i& grid_div,vector<int32_t>& out_vec)
{
		vector<int32_t>box_index;
		Eigen::Vector3i grid_div_mul(1, grid_div(0), grid_div(0)*grid_div(1));
		for (int32_t i = -1; i < 2; i++)
		{
			if (currentCoord(0) + i<0 || currentCoord(0) + i >= grid_div(0))
				continue;
			for (int32_t j = -1; j < 2; j++)
			{
				if (currentCoord(1) + j<0 || currentCoord(1) + j >= grid_div(1))
					continue;
				for (int32_t k = -1; k < 2; k++)
				{
					if (currentCoord(2) + k<0 || currentCoord(2) + k >= grid_div(2))
						continue;
					if (i == 0 && j == 0 && k == 0)
						continue;
					out_vec.push_back((currentCoord + Eigen::Vector3i(i, j, k)).dot(grid_div_mul));
				}
			}
		}
}

void zyk::getNeiboringBoxIndex3D(int32_t currentIndex, const Eigen::Vector3i& grid_div, vector<int32_t>& out_vec)
{
	Eigen::Vector3i div_mul(1, grid_div(0), grid_div(0)*grid_div(1));
	int32_t k = currentIndex / div_mul(2);
	currentIndex -= k*div_mul(2);
	int32_t j = currentIndex / div_mul(1);
	currentIndex -= j*div_mul(1);
	int32_t i = currentIndex;
	Eigen::Vector3i coord(i, j, k);
	getNeiboringBoxIndex3D(coord, grid_div, out_vec);
}

void transformNormals(const pcl::PointCloud<NormalType>&normals_in, pcl::PointCloud<NormalType>&normals_out, const Eigen::Affine3f& transform)
{
	size_t npts=normals_in.size();
	normals_out.resize(npts);
	for(size_t i=0;i<npts;++i)
	{
		Eigen::Vector3f nt (normals_in[i].normal_x, normals_in[i].normal_y, normals_in[i].normal_z);
		normals_out[i].normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
  	    normals_out[i].normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
        normals_out[i].normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));
	}
}


IplImage * loadDepth(std::string a_name)
{
	std::ifstream l_file(a_name.c_str(), std::ofstream::in | std::ofstream::binary);

	if (l_file.fail() == true)
	{
		printf("cv_load_depth: could not open file for writing!\n");
		return NULL;
	}
	int l_row;
	int l_col;

	l_file.read((char*)&l_row, sizeof(l_row));
	l_file.read((char*)&l_col, sizeof(l_col));

	IplImage * lp_image = cvCreateImage(cvSize(l_col, l_row), IPL_DEPTH_16U, 1);

	for (int l_r = 0; l_r < l_row; ++l_r)
	{
		for (int l_c = 0; l_c < l_col; ++l_c)
		{
			l_file.read((char*)&CV_IMAGE_ELEM(lp_image, unsigned short, l_r, l_c), sizeof(unsigned short));
		}
	}
	l_file.close();

	return lp_image;
}

double dot(const float* n1, const float* n2, const int dim)
{
	double res = 0;
	for (int i = 0; i < dim; ++i)
		res += n1[i] * n2[i];
	return res;
}

double norm(const float* n, const int dim)
{
	double res = 0;
	for (int i = 0; i < dim; ++i)
		res += n[i] * n[i];
	return sqrt(res);
}

double dist(const float* n1, const float* n2, const int dim)
{
	double res = 0;
	for (int i = 0; i < dim; ++i)
	{
		double tmp = n1[i]- n2[i];
		res += tmp*tmp;
	}
	return sqrt(res);
}
