#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "SmartSampling.hpp"

ZYK_INSTANTIATE_SMART_SAMPLING(PointType, NormalType);
//PCL_INSTANTIATE(SmartSampling, (pcl::PointXYZ, pcl::Normal));