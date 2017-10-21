#include "PPFFeature.h"
#include <pcl/features/ppf.h>
#include <pcl/common/transforms.h>
#include <fstream>
using namespace zyk;
//IMPLEMENT_SERIAL(PPF_Space,CObject,1);


zyk::PPF_Space::PPF_Space()
{

}

zyk::PPF_Space::~PPF_Space()
{
	for (int32_t i = 0; i < ppf_box_vector.size();i++)
		if (ppf_box_vector[i] != NULL) delete ppf_box_vector[i];
}
bool zyk::PPF_Space::init(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, float angle_step, float distance_step, bool ignore_plane)
{
	assert(angle_step>0 && distance_step>0);
	leaf_size(0) = angle_step;
	leaf_size(1) = angle_step;
	leaf_size(2) = angle_step;
	leaf_size(3) = distance_step;

	//ptr
	input_point_cloud = pointcloud;
	input_point_normal = pointNormal;
	//switch
	ignore_plane_switch = ignore_plane;
	//ppf
	if (!computeAllPPF())
		return false;
	//size
	if (!findBoundingBox())
		return false;
	Eigen::Array4f dim = max_p - min_p;
	grid_f1_div = floor(dim(0) / leaf_size(0) + 0.5);
	grid_f2_div = floor(dim(1) / leaf_size(1) + 0.5);
	grid_f3_div = floor(dim(2) / leaf_size(2) + 0.5);
	grid_f4_div = floor(dim(3) / leaf_size(3) + 0.5);

	grid_div_mul(0) = 1;
	grid_div_mul(1) = grid_div_mul(0)*grid_f1_div;
	grid_div_mul(2) = grid_div_mul(1)*grid_f2_div;
	grid_div_mul(3) = grid_div_mul(2)*grid_f3_div;
	total_box_num = grid_div_mul(3)*grid_f4_div;
	assert(total_box_num > 0);

	inv_leaf_size(0) = 1 / leaf_size(0);
	inv_leaf_size(1) = 1 / leaf_size(1);
	inv_leaf_size(2) = 1 / leaf_size(2);
	inv_leaf_size(3) = 1 / leaf_size(3);

	//grid
	ppf_box_vector.resize(total_box_num);
	if (!constructGrid())
		return false;
	return true;

}
bool zyk::PPF_Space::init(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, int32_t angle_div, int32_t distance_div, bool ignore_plane)
{
	////中心对称标志
	//model_x_centrosymmetric = x_s;
	//model_y_centrosymmetric = y_s;
	//model_z_centrosymmetric = z_s;
	////中心
	//point_cloud_center = pointCloudCenter;
	//ptr
	input_point_cloud = pointcloud;
	input_point_normal = pointNormal;
	//div
	assert(angle_div > 0 && distance_div > 0);
	grid_f1_div = angle_div;
	grid_f2_div = angle_div;
	grid_f3_div = angle_div;
	grid_f4_div = distance_div;
	grid_div_mul(0) = 1;
	grid_div_mul(1) = grid_div_mul(0)*grid_f1_div;
	grid_div_mul(2) = grid_div_mul(1)*grid_f2_div;
	grid_div_mul(3) = grid_div_mul(2)*grid_f3_div;
	total_box_num = grid_div_mul(3)*grid_f4_div;
	assert(total_box_num > 0);
	
	//switch
	ignore_plane_switch = ignore_plane;
	//ppf
	if (!computeAllPPF())
		return false;
	//size
	if (!findBoundingBox())
		return false;
	Eigen::Array4f dim = max_p - min_p;
	leaf_size(0) = dim(0) / grid_f1_div;
	leaf_size(1) = dim(1) / grid_f2_div;
	leaf_size(2) = dim(2) / grid_f3_div;
	leaf_size(3) = dim(3) / grid_f4_div;

	inv_leaf_size(0) = 1 / leaf_size(0);
	inv_leaf_size(1) = 1 / leaf_size(1);
	inv_leaf_size(2) = 1 / leaf_size(2);
	inv_leaf_size(3) = 1 / leaf_size(3);

	//grid
	ppf_box_vector.resize(total_box_num);
	if (!constructGrid())
		return false;
	return true;
}

void zyk::PPF_Space::clear()
{
	input_point_cloud = NULL;
	input_point_normal = NULL;
	for (int32_t i = 0; i < ppf_box_vector.size(); i++){
		if (ppf_box_vector[i] != NULL) delete ppf_box_vector[i];
	}
	ppf_box_vector.clear();
	ppf_vector.clear();
}

bool zyk::PPF_Space::findBoundingBox()
{
	if (ppf_vector.empty())
		return false;
	min_p.setConstant(FLT_MAX);
	max_p.setConstant(-FLT_MAX);
	for (int32_t i = 0; i < ppf_vector.size(); i++)
	{
		pcl::PPFSignature p = ppf_vector.at(i).ppf;
		if (!pcl_isfinite(p.f1) || !pcl_isfinite(p.f2) || !pcl_isfinite(p.f3) || !pcl_isfinite(p.f4) || !pcl_isfinite(p.alpha_m))
		{
			continue;
		}
		min_p = min_p.min(Eigen::Array4f(p.f1, p.f2, p.f3, p.f4));
		max_p = max_p.max(Eigen::Array4f(p.f1, p.f2, p.f3, p.f4));
	}
	return true;
}

bool zyk::PPF_Space::constructGrid()
{
	if (ppf_vector.empty())
		return false;
	for (int32_t i = 0; i < ppf_vector.size(); i++)
	{
		int32_t idx = getppfBoxIndex(ppf_vector[i]);
		if (idx != -1)
		{
			if (ppf_box_vector[idx] == NULL)
				ppf_box_vector[idx] = new box;
				ppf_box_vector[idx]->putin(i);
		}
	}
	return true;
}
float zyk::PPF_Space::computeAlpha(const PointType& first_pnt, const NormalType& first_normal, const PointType& second_pnt)
{
	float rotation_angle = acosf(first_normal.normal_x);
	bool parallel_to_x = (first_normal.normal_y > -FLT_EPSILON && first_normal.normal_y<FLT_EPSILON && first_normal.normal_z>-FLT_EPSILON && first_normal.normal_z < FLT_EPSILON);
	//TO DO, change to total array
	const Eigen::Vector3f& rotation_axis = (parallel_to_x) ? (Eigen::Vector3f::UnitY()) : (first_normal.getNormalVector3fMap().cross(Eigen::Vector3f::UnitX()).normalized());
	Eigen::AngleAxisf rotation_mg(rotation_angle, rotation_axis);

	const Eigen::Vector3f& second_point_transformed = rotation_mg * (Eigen::Vector3f(second_pnt.x-first_pnt.x,second_pnt.y-first_pnt.y,second_pnt.z-first_pnt.z));
	float angle = atan2f(-second_point_transformed(2), second_point_transformed(1));
	if (sin(angle) * second_point_transformed(2) > 0.0f)
		angle *= (-1);
	return angle;

}

float zyk::PPF_Space::computeAlpha(const Eigen::Vector3f& first_pnt, const Eigen::Vector3f& first_normal, const Eigen::Vector3f& second_pnt)
{
	float rotation_angle = acosf(first_normal(0));// float rotation_angle = acosf(model_reference_normal.dot(Eigen::Vector3f::UnitX()));
	bool parallel_to_x = (first_normal(1) > -FLT_EPSILON && first_normal(1)<FLT_EPSILON && first_normal(2)>-FLT_EPSILON && first_normal(2) < FLT_EPSILON);
	const Eigen::Vector3f& rotation_axis = (parallel_to_x) ? (Eigen::Vector3f::UnitY()) : (first_normal.cross(Eigen::Vector3f::UnitX()).normalized());
	Eigen::AngleAxisf rotation_mg(rotation_angle, rotation_axis);
	//Eigen::Affine3f transform_mg = Eigen::Affine3f::Identity();
	//transform_mg.rotate(rotation_mg);
	//transform_mg.translate(-rotation_mg.matrix()*model_reference_point);
	//Eigen::Affine3f transform_mg(Eigen::Translation3f(rotation_mg * ((-1) * model_reference_point)) * rotation_mg);

	const Eigen::Vector3f& second_point_transformed = rotation_mg * (second_pnt - first_pnt);
	float angle = atan2f(-second_point_transformed(2), second_point_transformed(1));
	if (sin(angle) * second_point_transformed(2) > 0.0f)
		angle *= (-1);
	return angle;
}
void zyk::PPF_Space::computeSinglePPF(const PointType& first_pnt, const NormalType& first_normal, const PointType& second_pnt, const NormalType& second_normal, zyk::PPF& ppf)
{
	float d[3];
	d[0] = second_pnt.x - first_pnt.x;
	d[1] = second_pnt.y - first_pnt.y;
	d[2] = second_pnt.z - first_pnt.z;
	float d_norm = norm(d, 3);

	d[0] /= d_norm;
	d[1] /= d_norm;
	d[2] /= d_norm;

	float dot_res;
	dot_res = dot(first_normal.normal, d, 3);
	if (dot_res > 1)dot_res = 1;
	else if (dot_res < -1)dot_res = -1;
	ppf.ppf.f1 = acosf(dot_res);

	dot_res = dot(second_normal.normal, d, 3);
	if (dot_res>1)dot_res = 1;
	else if (dot_res < -1)dot_res = -1;
	ppf.ppf.f2 = acosf(dot_res);

	dot_res = dot(first_normal, second_normal);
	if (dot_res>1)dot_res = 1;
	else if (dot_res < -1)dot_res = -1;
	ppf.ppf.f3 = acosf(dot_res);

	ppf.ppf.f4 = d_norm;

}
void zyk::PPF_Space::computeSinglePPF(const Eigen::Vector3f& first_pnt, const Eigen::Vector3f& first_normal, const Eigen::Vector3f& second_pnt, const Eigen::Vector3f& second_normal,zyk::PPF& ppf)
{
	Eigen::Vector3f d_normed = second_pnt - first_pnt;
	float d_norm = d_normed.norm();
	d_normed /= d_norm;
	float dot_res;
	dot_res = first_normal.dot(d_normed);
	if (dot_res>1)dot_res = 1;
	else if (dot_res<-1)dot_res = -1;
	ppf.ppf.f1 = acosf(dot_res);

	dot_res = second_normal.dot(d_normed);
	if (dot_res>1)dot_res = 1;
	else if (dot_res<-1)dot_res = -1;
	ppf.ppf.f2 = acosf(dot_res);

	dot_res = first_normal.dot(second_normal);
	if (dot_res>1)dot_res = 1;
	else if (dot_res<-1)dot_res = -1;
	ppf.ppf.f3 = acosf(dot_res);

	ppf.ppf.f4 = d_norm;

	/////// for speed reason, the following is separated!
	//float rotation_angle = acosf(first_normal(0));// float rotation_angle = acosf(model_reference_normal.dot(Eigen::Vector3f::UnitX()));
	//bool parallel_to_x = (first_normal(1) > -FLT_EPSILON && first_normal(1)<FLT_EPSILON && first_normal(2)>-FLT_EPSILON && first_normal(2) < FLT_EPSILON);
	//const Eigen::Vector3f& rotation_axis = (parallel_to_x) ? (Eigen::Vector3f::UnitY()) : (first_normal.cross(Eigen::Vector3f::UnitX()).normalized());
	//Eigen::AngleAxisf rotation_mg(rotation_angle, rotation_axis);
	////Eigen::Affine3f transform_mg = Eigen::Affine3f::Identity();
	////transform_mg.rotate(rotation_mg);
	////transform_mg.translate(-rotation_mg.matrix()*model_reference_point);
	////Eigen::Affine3f transform_mg(Eigen::Translation3f(rotation_mg * ((-1) * model_reference_point)) * rotation_mg);

	//const Eigen::Vector3f& second_point_transformed = rotation_mg * (second_pnt - first_pnt);
	//float angle = atan2f(-second_point_transformed(2), second_point_transformed(1));
	//if (sin(angle) * second_point_transformed(2) > 0.0f)
	//	angle *= (-1);
	//ppf.ppf.alpha_m = angle;
}

void zyk::PPF_Space::computeSinglePPF(pcl::PointCloud<PointType>::Ptr pointcloud, pcl::PointCloud<NormalType>::Ptr pointNormal, int32_t index1, int32_t index2, PPF& ppf)
{
	const Eigen::Vector3f& first_pnt = pointcloud->at(index1).getVector3fMap();
	const Eigen::Vector3f& first_normal = pointNormal->at(index1).getNormalVector3fMap();
	const Eigen::Vector3f& second_pnt = pointcloud->at(index2).getVector3fMap();
	const Eigen::Vector3f& second_normal = pointNormal->at(index2).getNormalVector3fMap();

	computeSinglePPF(first_pnt, first_normal, second_pnt, second_normal, ppf);
	ppf.first_index = index1;
	ppf.second_index = index2;
}

bool zyk::PPF_Space::getPoseFromPPFCorresspondence(PointType& model_point, NormalType& model_normal, PointType& scene_point, NormalType&scene_normal, float alpha, Eigen::Affine3f& transformation)
{

	Eigen::Vector3f model_p = model_point.getVector3fMap(),
		scene_p = scene_point.getVector3fMap(),
		model_n = model_normal.getNormalVector3fMap(),
		scene_n = scene_normal.getNormalVector3fMap();

	//model to global
	bool parallel_to_x_mg = (model_n.y() > -FLT_EPSILON && model_n.y()<FLT_EPSILON && model_n.z()>-FLT_EPSILON && model_n.z() < FLT_EPSILON);
	Eigen::Vector3f rotation_axis_mg = (parallel_to_x_mg) ? (Eigen::Vector3f::UnitY()) : (model_n.cross(Eigen::Vector3f::UnitX()).normalized());
	Eigen::AngleAxisf rotation_mg(acosf(model_n.dot(Eigen::Vector3f::UnitX())), rotation_axis_mg);
	Eigen::Affine3f transform_mg(Eigen::Translation3f(rotation_mg * ((-1) * model_p)) * rotation_mg);
	//scene to global
	bool parallel_to_x_sg = (scene_n.y() > -FLT_EPSILON && scene_n.y()<FLT_EPSILON && scene_n.z()>-FLT_EPSILON && scene_n.z() < FLT_EPSILON);
	Eigen::Vector3f rotation_axis_sg = (parallel_to_x_sg) ? (Eigen::Vector3f::UnitY()) : (scene_n.cross(Eigen::Vector3f::UnitX()).normalized());
	Eigen::AngleAxisf rotation_sg(acosf(scene_n.dot(Eigen::Vector3f::UnitX())), rotation_axis_sg);
	Eigen::Affine3f transform_sg(Eigen::Translation3f(rotation_sg * ((-1) * scene_p)) * rotation_sg);
	//total
	Eigen::Affine3f transform_ms(transform_sg.inverse()*Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX())*transform_mg);
	transformation = transform_ms;
	Eigen::AngleAxisf tmp1(transform_ms.rotation());
	Eigen::Vector3f rot = tmp1.angle()*tmp1.axis();
	Eigen::Vector3f trans = transform_ms.translation();
	if (!pcl_isfinite(trans(0) || !pcl_isfinite(trans(1)) || !pcl_isfinite(trans(2)) || !pcl_isfinite(rot(0)) || !pcl_isfinite(rot(1)) || !pcl_isfinite(rot(2))))
		return false;
	return true;
}

bool zyk::PPF_Space::computeAllPPF()
{
	//check
	if (input_point_cloud == NULL || input_point_normal == NULL)
		return false;
	if (input_point_cloud->empty() || input_point_normal->empty())
		return false;
	if (input_point_cloud->size() != input_point_normal->size())
		return false;

	//loop
	for (int32_t i = 0; i < input_point_cloud->size(); i++)
	{
		/*if (model_x_centrosymmetric)
		{
			if (input_point_cloud->at(i).z > point_cloud_center(2))
				continue;
		}
		if (model_y_centrosymmetric)
		{
			if (input_point_cloud->at(i).x > point_cloud_center(0))
				continue;
		}
		if (model_z_centrosymmetric)
		{
			if (input_point_cloud->at(i).y > point_cloud_center(1))
				continue;
		}*/
		for (int32_t j = 0; j < input_point_cloud->size(); j++)
		{
			if (i != j)
			{
				PPF ppf;
				computeSinglePPF(input_point_cloud, input_point_normal, i, j, ppf);
				if (ignore_plane_switch)
				{
					if (abs(ppf.ppf.f3) < 0.01 && abs(ppf.ppf.f1-M_PI_2)<0.01&&abs(ppf.ppf.f2-M_PI_2)<0.01)
						continue;
				}
				ppf.ppf.alpha_m = computeAlpha(input_point_cloud->at(i).getVector3fMap(), input_point_normal->at(i).getNormalVector3fMap(), input_point_cloud->at(j).getVector3fMap());
				ppf_vector.push_back(ppf);
			}
		}
	}

	return true;
}
void zyk::PPF_Space::getppfBoxCoord(PPF& ppf, Eigen::Vector4i& ijk)
{
	if (!pcl_isfinite(ppf.ppf.f1) || !pcl_isfinite(ppf.ppf.f2) || !pcl_isfinite(ppf.ppf.f3) || !pcl_isfinite(ppf.ppf.f4) || !pcl_isfinite(ppf.ppf.alpha_m))
	{
		ijk[0] = ijk[1] = ijk[2] = ijk[3] = -1;
	}
	//float a1 = (ppf.ppf.f1-min_p[0])*inv_leaf_size(0);
	//float a2 = (ppf.ppf.f2-min_p[1])*inv_leaf_size(1);
	//float a3 = (ppf.ppf.f3-min_p[2])*inv_leaf_size(2);
	//float a4 = (ppf.ppf.f4-min_p[3])*inv_leaf_size(3);
	Eigen::Array4f tmp = (Eigen::Array4f(ppf.ppf.f1, ppf.ppf.f2, ppf.ppf.f3, ppf.ppf.f4) - min_p)*inv_leaf_size;
	//ijk[0] = floor(a1);
	//ijk[1] = floor(a2);
	//ijk[2] = floor(a3);
	//ijk[3] = floor(a4);
	ijk = Eigen::Vector4i(floor(tmp(0)), floor(tmp(1)), floor(tmp(2)), floor(tmp(3)));
	if (ijk[0] < 0 || ijk[1] < 0 || ijk[2] < 0 || ijk[3] < 0 || ijk[0] >= grid_f1_div || ijk[1] >= grid_f2_div || ijk[2] >= grid_f3_div || ijk[3] >= grid_f4_div)
		ijk[0] = ijk[1] = ijk[2] = ijk[3] = -1;
}
void zyk::PPF_Space::getppfBoxCoord(PPF& ppf, int32_t* ijk)
{
	if (!pcl_isfinite(ppf.ppf.f1) || !pcl_isfinite(ppf.ppf.f2) || !pcl_isfinite(ppf.ppf.f3) || !pcl_isfinite(ppf.ppf.f4) || !pcl_isfinite(ppf.ppf.alpha_m))
	{
		ijk[0] = ijk[1] = ijk[2] = ijk[3] = -1;
	}
	float a1 = (ppf.ppf.f1-min_p[0])*inv_leaf_size(0);
	float a2 = (ppf.ppf.f2-min_p[1])*inv_leaf_size(1);
	float a3 = (ppf.ppf.f3-min_p[2])*inv_leaf_size(2);
	float a4 = (ppf.ppf.f4-min_p[3])*inv_leaf_size(3);
	//Eigen::Array4f tmp = (Eigen::Array4f(ppf.ppf.f1, ppf.ppf.f2, ppf.ppf.f3, ppf.ppf.f4) - min_p)*inv_leaf_size;
	ijk[0] = floor(a1);
	ijk[1] = floor(a2);
	ijk[2] = floor(a3);
	ijk[3] = floor(a4);
	//ijk = Eigen::Vector4i(floor(tmp(0)), floor(tmp(1)), floor(tmp(2)), floor(tmp(3)));
	if (ijk[0] < 0 || ijk[1] < 0 || ijk[2] < 0 || ijk[3]<0 || ijk[0] >= grid_f1_div || ijk[1] >= grid_f2_div || ijk[2] >= grid_f3_div||ijk[3]>=grid_f4_div)
		ijk[0] = ijk[1] = ijk[2] = ijk[3] = -1;
}

int32_t zyk::PPF_Space::getppfBoxIndex(PPF& ppf)
{
#ifdef use_eigen
	Eigen::Vector4i ijk;
#else
	int32_t ijk[4];
#endif
	getppfBoxCoord(ppf, ijk);
#ifdef use_eigen
	int32_t i = ijk.dot(grid_div_mul);
#else
	int32_t i = ijk[0] * grid_div_mul(0) + ijk[1] * grid_div_mul[1] + ijk[2] * grid_div_mul[2] + ijk[3] * grid_div_mul[3];
#endif
	if (i >= total_box_num || i < 0)
		i = -1;
	return i;
}

//void zyk::PPF_Space::Serialize(CArchive &ar)
//{
//	CObject::Serialize(ar);
//	if (ar.IsStoring())
//	{
//		ar << grid_f1_div << grid_f2_div << grid_f3_div << grid_f4_div << min_p(0) << min_p(1) << min_p(2) << min_p(3) << max_p(0) << max_p(1) << max_p(2) << max_p(3)<<ppf_vector.size();
//		for (int32_t i = 0; i < ppf_vector.size(); i++)
//		{
//			ar << ppf_vector[i].first_index << ppf_vector[i].second_index << ppf_vector[i].ppf.alpha_m << ppf_vector[i].ppf.f1 << ppf_vector[i].ppf.f2 << ppf_vector[i].ppf.f3 << ppf_vector[i].ppf.f4;
//		}
//		ar << input_point_cloud->size();
//		for (int32_t i = 0; i < input_point_cloud->size(); i++)
//		{
//			ar << input_point_cloud->at(i).x << input_point_cloud->at(i).y << input_point_cloud->at(i).z << input_point_normal->at(i).normal_x << input_point_normal->at(i).normal_y << input_point_normal->at(i).normal_z;
//		}
//	}
//	else
//	{
//		int32_t ppf_vector_size = 0;
//		ar >> grid_f1_div >> grid_f2_div >> grid_f3_div >> grid_f4_div >> min_p(0) >> min_p(1) >> min_p(2) >> min_p(3) >> max_p(0) >> max_p(1) >> max_p(2) >> max_p(3) >> ppf_vector_size;
//		ppf_vector.resize(ppf_vector_size);
//		for (int32_t i = 0; i < ppf_vector_size; i++)
//		{
//			ar >> ppf_vector[i].first_index >> ppf_vector[i].second_index >> ppf_vector[i].ppf.alpha_m >> ppf_vector[i].ppf.f1 >> ppf_vector[i].ppf.f2 >> ppf_vector[i].ppf.f3 >> ppf_vector[i].ppf.f4;
//		}
//		int32_t points_num = 0;
//		ar >> points_num;
//		input_point_cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
//		input_point_normal = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
//		for (int32_t i = 0; i < points_num; i++)
//		{
//			PointType _tem;
//			NormalType _nor;
//			ar >> _tem.x >> _tem.y >> _tem.z >> _nor.normal_x >> _nor.normal_y >> _nor.normal_z;
//			input_point_cloud->push_back(_tem);
//			input_point_normal->push_back(_nor);
//		}
//		grid_div_mul(0) = 1;
//		grid_div_mul(1) = grid_div_mul(0)*grid_f1_div;
//		grid_div_mul(2) = grid_div_mul(1)*grid_f2_div;
//		grid_div_mul(3) = grid_div_mul(2)*grid_f3_div;
//		total_box_num = grid_div_mul(3)*grid_f4_div;
//		assert(total_box_num > 0);
//
//		Eigen::Array4f dim = max_p - min_p;
//		leaf_size(0) = dim(0) / grid_f1_div;
//		leaf_size(1) = dim(1) / grid_f2_div;
//		leaf_size(2) = dim(2) / grid_f3_div;
//		leaf_size(3) = dim(3) / grid_f4_div;
//
//		inv_leaf_size(0) = 1 / leaf_size(0);
//		inv_leaf_size(1) = 1 / leaf_size(1);
//		inv_leaf_size(2) = 1 / leaf_size(2);
//		inv_leaf_size(3) = 1 / leaf_size(3);
//
//		//grid
//		ppf_box_vector.resize(total_box_num);
//		constructGrid();
//	}
//}





void zyk::PPF_Space::match(pcl::PointCloud<PointType>::Ptr scene, pcl::PointCloud<NormalType>::Ptr scene_normals,float relativeReferencePointsNumber,float max_vote_thresh, float max_vote_percentage, float angle_thresh, float first_dis_thresh, float recompute_score_dis_thresh, float recompute_score_ang_thresh, int num_clusters_per_group, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> &pose_clusters)
{
	int scene_steps = floor(1.0 / relativeReferencePointsNumber);
	if (scene_steps < 1)scene_steps = 1;

	float radius = max_p[3];
	float box_radius = sqrt(model_size[0] * model_size[0] + model_size[1] * model_size[1] + model_size[2] * model_size[2]);
	first_dis_thresh *= box_radius;
	float second_dis_thresh = 0.5 * box_radius;
	recompute_score_dis_thresh *= model_res;
	//build grid to accelerate matching process
	zyk::CVoxel_grid scene_grid(radius, radius, radius, scene);
	Eigen::Vector3i grid_div;
	scene_grid.getGridDiv(grid_div);
	vector<zyk::box*>*box_vector = scene_grid.getBox_vector();
	Eigen::Vector3i div_mul(1, grid_div(0), grid_div(0)*grid_div(1));


	int32_t num_poses_count = 0;
	//all raw poses
	vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> rawClusters;

	///////// info
	int32_t match_count = 0;
	int32_t match_percent = 0;
	int32_t info_step = 0.05*scene->size();

	int32_t tst_cnt1 = 0;
	int32_t tst_cnt2 = 0;
	/////////////////////begin iterate boxes
	for (int32_t box_index = 0; box_index < box_vector->size(); ++box_index)
	{
		/////////get box ptr
		zyk::box*p_current_point_box = box_vector->at(box_index);
		if (p_current_point_box == NULL)
			continue;
		/////////get neighboring
		vector<int32_t>neiboringBoxIndexVector;
		//for convenience, push back current index too!
		neiboringBoxIndexVector.push_back(box_index);
		zyk::getNeiboringBoxIndex3D(box_index, grid_div, neiboringBoxIndexVector);
		
		for (int32_t cnt1 = 0; cnt1 < box_vector->at(box_index)->size(); cnt1 = cnt1 + scene_steps)
		{
			if ((match_count+=scene_steps) > info_step)
			{
				match_count = 0;
				match_percent += 5;
				cout << "Match : " << match_percent << "%." << endl;
				//cout << "tst_cnt1: " << tst_cnt1 << endl;
				//cout << "tst_cunt2: " << tst_cnt2 << endl;
				tst_cnt1 = 0;
				tst_cnt2 = 0;
			}
			int32_t reference_pnt_index = (*p_current_point_box)[cnt1];
			/////////build an accumulator for this reference point
			// in matlab the row size is size+1, why?
			zyk::PPF_Accumulator acum(input_point_cloud->size(), 30);
#ifdef use_eigen
			const Eigen::Vector3f& rp = scene->at(reference_pnt_index).getVector3fMap();
			const Eigen::Vector3f& rn = scene_normals->at(reference_pnt_index).getNormalVector3fMap();
#else
			const PointType& rp = scene->at(reference_pnt_index);
			const NormalType& rn = scene_normals->at(reference_pnt_index);
#endif
			// loop through neighboring boxes to get scene pnt
			for (int32_t cnt2 = 0; cnt2 < neiboringBoxIndexVector.size(); ++cnt2)
			{
				int32_t neiboringBoxIndex = neiboringBoxIndexVector[cnt2];
				zyk::box*p_current_neiboring_point_box = box_vector->at(neiboringBoxIndex);
				if (p_current_neiboring_point_box == NULL)
					continue;
				for (int32_t cnt3 = 0; cnt3 < p_current_neiboring_point_box->size(); ++cnt3)
				{
					int32_t scene_pnt_index = (*p_current_neiboring_point_box)[cnt3];
					if (scene_pnt_index == reference_pnt_index)
						continue;
#ifdef use_eigen
					const Eigen::Vector3f& sp = scene->at(scene_pnt_index).getVector3fMap();
					const Eigen::Vector3f& sn = scene_normals->at(scene_pnt_index).getNormalVector3fMap();
#else
					const PointType& sp = scene->at(scene_pnt_index);
					const NormalType& sn = scene_normals->at(scene_pnt_index);
#endif
					if (neiboringBoxIndex != box_index)
					{
#ifdef use_eigen
						if ((sp - rp).norm() > radius)
							continue;
#else
						if (dist(sp.data, rp.data, 3)>radius)
							continue;
#endif
					}
					//calculate ppf
					zyk::PPF current_ppf;
					zyk::PPF_Space::computeSinglePPF(rp, rn, sp, sn, current_ppf);
					if (abs(current_ppf.ppf.f3) < 0.02 && abs(current_ppf.ppf.f1 - M_PI_2) < 0.02&&abs(current_ppf.ppf.f2 - M_PI_2) < 0.02)
						continue;
					//do hash
					int32_t ppf_box_index = getppfBoxIndex(current_ppf);
					if (ppf_box_index == -1)
						continue;
					zyk::box* current_ppf_box = ppf_box_vector.at(ppf_box_index);
					if (current_ppf_box == NULL)
						continue;
					//now calculate alpha of current_ppf
					double current_alpha= computeAlpha(rp, rn, sp);
					//loop hash list
					for (int32_t node = 0; node < current_ppf_box->size(); ++node)
					{
						zyk::PPF& correspond_model_ppf = ppf_vector.at((*current_ppf_box)[node]);
						float alpha = correspond_model_ppf.ppf.alpha_m - current_alpha;
						if (alpha >= M_PI) alpha -= 2 * M_PI;
						if (alpha < -M_PI)alpha += 2 * M_PI;
						float alpha_bin = (alpha + M_PI) / 2 / M_PI * 30;
						int32_t middle_bin = int(alpha_bin + 0.5f);
						if (middle_bin >= 30) middle_bin = 0;
						//calculate vote_val
						//float tmp = cos(correspond_model_ppf.ppf.f3);
						//if (1 - tmp < 0.003)
						//{
						//	continue;
						//}
						float vote_val = 1 - 0.98 * fabs(cos(correspond_model_ppf.ppf.f3));
						acum.acumulator(correspond_model_ppf.first_index, middle_bin) += vote_val;
						tst_cnt2++;
					}

				}

			}
			//check the accumulator and extract poses
			Eigen::MatrixXi::Index maxRow, maxCol;
			float maxVote = acum.acumulator.maxCoeff(&maxRow, &maxCol);
			//if (max_vote_thresh > 0){
			if (maxVote < max_vote_thresh)
				continue;
			//}
			float vote_thresh = max_vote_percentage*maxVote;

			//visit the accumulator and extract poses
			for (int32_t row = 0; row < acum.acumulator.rows(); ++row)
			{
				for (int32_t col = 0; col < acum.acumulator.cols(); ++col)
				{
					if (acum.acumulator(row, col) < vote_thresh)
						continue;
					num_poses_count++;
					tst_cnt1++;
					float alpha = col / 30.0 * 2 * M_PI - M_PI;
					Eigen::Affine3f transformation;
					if (!zyk::PPF_Space::getPoseFromPPFCorresspondence(input_point_cloud->at(row), input_point_normal->at(row), scene->at(reference_pnt_index), scene_normals->at(reference_pnt_index), alpha, transformation))
						continue;
					rawClusters.push_back(zyk::pose_cluster(transformation, acum.acumulator(row, col)));
				}
			}//end of visit accumulator

		}

	}
	std::sort(rawClusters.begin(), rawClusters.end(), zyk::pose_cluster_comp);
	cout << "all poses number : " << rawClusters.size() << endl;
	////////////Now do clustering
	// raw cluster first
	for (int32_t i = 0; i < rawClusters.size(); i++)
	{
		int result = 0;
		for (int32_t j = 0; j < pose_clusters.size(); j++)
		{
			if (pose_clusters[j].checkAndPutIn(rawClusters[i].transformations[0], rawClusters[i].vote_count, first_dis_thresh, angle_thresh))
			{
				//if(++result>=max_clusters_per_pose_can_be_in)
				result = 1;
				break;	
			}
		}
		if (!result)
			pose_clusters.push_back(rawClusters[i]);
	}
	std::sort(pose_clusters.begin(), pose_clusters.end(), zyk::pose_cluster_comp);

	//////recompute score
	if (recompute_score_dis_thresh > 0)
	{
		cout << "Now recompute scores, distance thresh is: " << recompute_score_dis_thresh << endl;
		recomputeClusterScore(scene_grid,*scene_normals, recompute_score_dis_thresh, recompute_score_ang_thresh, pose_clusters);
		std::sort(pose_clusters.begin(), pose_clusters.end(), zyk::pose_cluster_comp);
	}

	// second cluster by distance
	// group those clear clusters together and rank them
	if (num_clusters_per_group > 0){
		cout << "Now do grouping, number of clusters per group is : " << num_clusters_per_group << endl;
		vector<vector<pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>>> groups;
		for (int32_t i = 0; i < pose_clusters.size(); i++)
		{
			int result = 0;
			for (int32_t j = 0; j < groups.size(); j++)
			{
				if ((groups[j][0].mean_trans - pose_clusters[i].mean_trans).norm() < second_dis_thresh)
				{
					groups[j].push_back(pose_clusters[i]);
					result = 1;
					break;
				}
			}
			if (!result)
			{
				vector<pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> tmp_clusters;
				tmp_clusters.push_back(pose_clusters[i]);
				groups.push_back(tmp_clusters);
			}
		}

		cout << "group size is: " << groups.size() << endl;
		pose_clusters.clear();
		for (int i = 0; i < groups.size(); i++)
		{
			for (int j = 0; j < min(num_clusters_per_group, int(groups[i].size())); ++j)
			{
				pose_clusters.push_back(groups[i][j]);
			}
				
		}
	}

}

void zyk::PPF_Space::recomputeClusterScore(zyk::CVoxel_grid& grid, pcl::PointCloud<NormalType>& scene_normals, float dis_thresh, float ang_thresh, vector<zyk::pose_cluster, Eigen::aligned_allocator<zyk::pose_cluster>> &pose_clusters)
{
	
	pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr rotated_normal(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<PointType>::Ptr scene = grid.getInputPointCloud();

	grid.resplit(dis_thresh , dis_thresh , dis_thresh );
	Eigen::Vector3i grid_div;
	grid.getGridDiv(grid_div);

	vector<zyk::box*>* box_vector = grid.getBox_vector();
	if (ang_thresh < 0)ang_thresh = M_PI_2;
	float cos_ang_thresh = cos(ang_thresh);
	for (int32_t num = 0; num < pose_clusters.size(); ++num)
	{
		float score = 0;
		pcl::transformPointCloud(*input_point_cloud, *rotated_model, pose_clusters[num].mean_transformation);
		transformNormals(*input_point_normal, *rotated_normal, pose_clusters[num].mean_transformation);
		for (int32_t i = 0; i < rotated_model->size(); ++i)
		{
			int32_t pnt_box_index = grid.getPntBoxIndex(rotated_model->at(i));
			if (pnt_box_index == -1)continue;
			//zyk::box* p_box = box_vector->at(pnt_box_index);
			//if (p_box == NULL)continue;
			PointType mp = rotated_model->at(i);
			NormalType mn = rotated_normal->at(i);

			/////////get neighboring
			vector<int32_t>neiboringBoxIndexVector;
			neiboringBoxIndexVector.reserve(30);
			//for convenience, push back current index too!
			neiboringBoxIndexVector.push_back(pnt_box_index);
			zyk::getNeiboringBoxIndex3D(pnt_box_index, grid_div, neiboringBoxIndexVector);

			bool found = false;
			for (int j = 0; j < neiboringBoxIndexVector.size() && !found; ++j)
			{
				int neiboringBoxIndex = neiboringBoxIndexVector[j];
				zyk::box*p_current_neiboring_point_box = box_vector->at(neiboringBoxIndex);
				if (p_current_neiboring_point_box == NULL)
					continue;
				for (int k = 0; k < p_current_neiboring_point_box->size(); ++k)
				{
					PointType sp = scene->at((*p_current_neiboring_point_box)[k]);
					NormalType sn = scene_normals.at((*p_current_neiboring_point_box)[k]);
					if (fabs(mp.x - sp.x) + fabs(mp.y - sp.y) + fabs(mp.z - sp.z) < dis_thresh)
					{
						if (dot(mn.normal, sn.normal, 3) > cos_ang_thresh) {
							score = score + 1;
							found = true;
							break;
						}
					}
				}
			}

		}
		pose_clusters[num].vote_count = score / input_point_cloud->size();
	}
}

bool zyk::PPF_Space::save(std::string file_name)
{
	bool res = false;
	std::ofstream ofs(file_name, std::ios::binary);
	if (ofs.is_open())
	{
		boost::archive::binary_oarchive ar(ofs);
		this->save(ar, 1);
		res = true;
	}
	ofs.close();
	return res;
}

bool zyk::PPF_Space::load(std::string fine_name)
{
	bool res = false;
	std::ifstream ifs(fine_name, std::ios::binary);
	if (ifs.is_open())
	{
		boost::archive::binary_iarchive ar(ifs);
		this->load(ar, 1);
		res = true;
	}
	ifs.close();
	return res;

}

