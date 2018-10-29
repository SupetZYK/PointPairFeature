#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "SmartSampling.hpp"
//template class pcl::SmartSampling<pcl::PointXYZ, pcl::Normal>;
ZYK_INSTANTIATE_SMART_SAMPLING(pcl::PointXYZ,pcl::Normal);
//PCL_INSTANTIATE(SmartSampling, (pcl::PointXYZ, pcl::Normal));