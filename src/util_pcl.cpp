#include "util_pcl.h"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/vtk_io.h>
#include "SmartSampling.hpp"
#include <pcl/common/transforms.h>
#include <pcl/io/ply/ply_parser.h>
using namespace std;
using namespace pcl;



double zyk::computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud, double max_coord[3], double min_coord[3])
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
pcl::IndicesPtr zyk::uniformDownSamplePoint(pcl::PointCloud<PointType>::Ptr pointcloud, double relSamplingDistance, pcl::PointCloud<PointType>::Ptr outCloud)
{
	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(pointcloud);
	uniform_sampling.setRadiusSearch(relSamplingDistance);
	uniform_sampling.filter(*outCloud);
	return uniform_sampling.getSelectedIndex();

}

pcl::IndicesPtr zyk::uniformDownSamplePointAndNormal(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, double relSamplingDistance,
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
	for (int i = 0; i < selectedIndex->size(); i++)
		outNormal->points[i]= pointNormal->points[selectedIndex->at(i)];
	return selectedIndex;
}

bool zyk::SmartDownSamplePointAndNormal(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, double ang_degree_thresh, double relSamplingDistance,
	pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNormal)
{
	pcl::SmartSampling<PointType,NormalType> sampling;
	sampling.setInputCloud(pointcloud);
	sampling.setNormals(pointNormal);
	sampling.setRadiusSearch(relSamplingDistance);
	sampling.setAngleThresh(ang_degree_thresh);
	sampling.filter(*outCloud);
	vector<size_t>& selectedIndex = sampling.getSelectedIndex();

	outNormal->height = 1;
	outNormal->is_dense = true;
	outNormal->points.resize(selectedIndex.size());
	for (int i = 0; i < selectedIndex.size(); i++)
		outNormal->points[i] = pointNormal->points[selectedIndex[i]];
	return true;
}


bool zyk::readPointCloud(std::string filename, pcl::PointCloud<PointType>::Ptr outCloud, pcl::PointCloud<NormalType>::Ptr outNor)
{
	std::string fileformat;
	int pos = filename.find_last_of('.') + 1;
	fileformat = filename.substr(pos);

	if (fileformat == "ply")
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _Pcloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(filename, *_Pcloud) < 0)
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
	else if (fileformat == "txt")
	{
		//only x y z supported
		filebuf *pbuf;
		ifstream fileread;
		long size;
		char * buffer;
		// 要读入整个文件，必须采用二进制打开   
		fileread.open(filename, ios::binary);
		// 获取filestr对应buffer对象的指针（获得这个流对象的指针）
		pbuf = fileread.rdbuf();

		// 调用buffer对象方法获取文件大小  （当复位位置指针指向文件缓冲区的末尾时候pubseekoff返回的值就是整个文件流大小）
		size = pbuf->pubseekoff(0, ios::end, ios::in);
		pbuf->pubseekpos(0, ios::in);   //再让pbuf指向文件流开始位置

		// 分配内存空间  
		buffer = new char[size];

		// 获取文件内容  
		pbuf->sgetn(buffer, size);

		fileread.close();


		// 输出到标准输出  

		//最佳方法按字符遍历整个buffer
		string temp = "";
		PointType Pnts;

		float x = 0;
		float y = 0;
		float z = 0;
		bool isy = false;
		while (*buffer != '\0')
		{
			if (*buffer != '\n' && *buffer != '\r')
			{
				if (*buffer != ' ')
				{
					temp += *buffer;
				}
				else
				{
					if (!isy)  //如果是x的值
					{
						if (!temp.empty())
						{
							isy = !isy;
							sscanf(temp.data(), "%f", &x);
							Pnts.x = x;
							temp = "";
						}
					}
					else                  //如果是y的值
					{
						if (!temp.empty())
						{
							isy = !isy;
							sscanf(temp.data(), "%f", &y);
							Pnts.y = y;
							temp = "";
						}
					}
				}
			}
			else   //这里是z
			{
				if (!temp.empty())
				{
					sscanf(temp.data(), "%f", &z);
					Pnts.z = z;
					temp = "";
					outCloud->push_back(Pnts);
				}
			}
			buffer++;
		}
	}
	else if (fileformat == "pcd")
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


void zyk::transformNormals(const pcl::PointCloud<NormalType>&normals_in, pcl::PointCloud<NormalType>&normals_out, const Eigen::Affine3f& transform)
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
