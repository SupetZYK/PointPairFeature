#include "pose_cluster.h"

void zyk::pose_cluster::checkDisAndRemove(int index,float thresh)
{
	int cnt=size();
	assert(index>=0 && index<cnt);
	if((mean_trans-transformations[index].translation()).norm()>thresh)
	{
		vote_count-=voteLists[index];
		Eigen::Vector3f tmp=transformations[index].translation();
		
		mean_trans=(mean_trans*cnt-tmp)/(cnt-1);
	}
}	



bool zyk::pose_cluster::checkAndPutIn(const Eigen::Affine3f& transformation, float vote, float distance_thresh, float angle_thresh)
{
	if (transformations.empty())
	{
		transformations.push_back(transformation);
		vote_count += vote;
		voteLists.push_back(vote);
		mean_transformation = transformation;

		Eigen::AngleAxisf tmp(mean_transformation.rotation());
		//check tmp.angle()
		first_angle = tmp.angle();
		first_axis = tmp.axis();

		first_q = Eigen::Quaternionf(mean_transformation.rotation());
		mean_rot = first_angle*first_axis;
		mean_trans = mean_transformation.translation();

		return true;
	}
	
	if ((transformation.translation()-transformations[0].translation()).norm() > distance_thresh)
		return false;
	//Eigen::Affine3f dT = transformation.inverse()*transformations[0];
	//if (abs(Eigen::AngleAxisf(dT.rotation()).angle()) > angle_thresh)
	//	return false;

	Eigen::AngleAxisf trans_rot(transformation.rotation());
	Eigen::Vector3f trans_tmp_rot;

	float angle = trans_rot.angle();
	Eigen::Vector3f tst_axis = trans_rot.axis();

	float res1 = fabs(angle - first_angle);
	float res2 = fabs(2 * M_PI - angle - first_angle);
	float res3 = first_axis.dot(tst_axis);
	float res4 = cos(angle_thresh);
	//4 conditions
	if (res1<angle_thresh && res3>res4)
	{
		trans_tmp_rot = angle*tst_axis;
	}
	// this case angle is near 180 degree
	else if (res1<angle_thresh && -res3>res4) {
		trans_tmp_rot = (angle - 2 * M_PI)*tst_axis;
	}
	// this case angle is near 180 degree
	else if (res2<angle_thresh && res3>res4) {
		//trans_tmp_rot = (2 * M_PI - angle)*tst_axis;
		trans_tmp_rot = angle*tst_axis;
	}
	else if (res2 < angle_thresh && -res3 > res4)
	{
		trans_tmp_rot = (angle - 2 * M_PI)*tst_axis;
	}
	else
	{
		return false;
	}

	//Eigen::Vector3f tmp1_rot = angle*trans_rot.axis();
	//if (tmp1_rot.dot(mean_rot) < 0)
	//	tmp1_rot = -(2 * M_PI - angle)*trans_rot.axis();
	//float tmp3 = 1.0 / (transformations.size() + 1);
	//mean_rot = (transformations.size()*mean_rot + trans_tmp_rot)*tmp3;
	//mean_trans = (transformations.size()*mean_trans + transformation.translation())*tmp3;

	//float mean_angle = mean_rot.norm();
	//mean_transformation = Eigen::Affine3f(Eigen::Translation3f(mean_trans)*Eigen::AngleAxisf(mean_angle, mean_rot.normalized()));
	//cout << Eigen::AngleAxisf(mean_angle, mean_rot.normalized()).matrix() << endl;
	//cout << mean_transformation.matrix() << endl;
	transformations.push_back(transformation);
	voteLists.push_back(vote);
	vote_count += vote;
	return true;

}

bool zyk::pose_cluster::checkAndPutIn_V2(const Eigen::Affine3f & transformation, float vote, float distance_thresh, float angle_thresh)
{
	// use transformations[0], do not update mean
	if (transformations.empty())
	{
		transformations.push_back(transformation);
		vote_count += vote;
		voteLists.push_back(vote);
		return true;
	}
	if ((transformation.translation() - transformations[0].translation()).norm() > distance_thresh)
		return false;
	//Eigen::Matrix3f rot = transformation.rotation().transpose()*transformations[0].rotation();
	//float deta_angle = Eigen::AngleAxisf(rot).angle();
	//if (deta_angle > M_PI)
	//	deta_angle -= 2 * M_PI;
	//if (deta_angle < -M_PI)
	//	deta_angle += 2 * M_PI;
	//if (abs(deta_angle) > angle_thresh)
	//	return false;
	float d = Eigen::Quaternionf(transformation.rotation()).dot(Eigen::Quaternionf(transformations[0].rotation()));
	float d_rad = acos(2 * d*d - 1);
	if (fabs(d_rad) > angle_thresh)
		return false;
	transformations.push_back(transformation);
	vote_count += vote;
	voteLists.push_back(vote);
	return true;
}

void zyk::pose_cluster::averageCluster()
{
	Eigen::Vector3f tmp_mean_trans(0,0,0);
	float w=0, x=0, y=0, z=0;
	for (auto& t : transformations)
	{
		tmp_mean_trans += t.translation();
		Eigen::Quaternionf tmpq =  Eigen::Quaternionf(t.rotation()).normalized();
		float tmpx = tmpq.x(), tmpy = tmpq.y(), tmpz = tmpq.z(), tmpw = tmpq.w();
		if (tmpw > 0)
		{
			w += tmpw;
			x += tmpx;
			y += tmpy;
			z += tmpz;
		}
		else
		{
			w -= tmpw;
			x -= tmpx;
			y -= tmpy;
			z -= tmpz;
		}
	}
	w /= transformations.size();
	x /= transformations.size();
	y /= transformations.size();
	z /= transformations.size();
	Eigen::Quaternionf mean_q(w, x, y, z);
	mean_q.normalize();
	mean_trans = tmp_mean_trans / transformations.size();
	mean_transformation = Eigen::Affine3f(mean_q).pretranslate(mean_trans);
}