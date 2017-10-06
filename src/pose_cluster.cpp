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
		float angle = tmp.angle();
		//if(angle<0)
		//	cout<<"angle <0 detect!"<<endl;
		//if (angle > M_PI)
			//angle -= 2 * M_PI;
		//else if (angle < -M_PI)
			//angle += 2 * M_PI;
		first_angle = angle;
		first_axis = tmp.axis();
		mean_rot = angle*first_axis;
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
	float res1 = fabs(angle - first_angle);
	float res2 = fabs(2 * M_PI - angle - first_angle);
	float res3 = first_axis.dot(trans_rot.axis());
	float res4 = cos(angle_thresh);
	if (res1<angle_thresh&&res3>res4)
	{
		trans_tmp_rot = angle*trans_rot.axis();
	}
	else if (res2 < angle_thresh&&res3 < -res4)
	{
		trans_tmp_rot = (angle - 2 * M_PI)*trans_rot.axis();
	}
	else
	{
		return false;
	}
	//if (fabs(angle - first_angle) > angle_thresh)
	//	return false;
	//if (angle > M_PI)
		//angle -= 2 * M_PI;
	//else if (angle < -M_PI)
		//angle += 2 * M_PI;

	//Eigen::Vector3f tmp1_rot = angle*trans_rot.axis();
	//if (tmp1_rot.dot(mean_rot) < 0)
	//	tmp1_rot = -(2 * M_PI - angle)*trans_rot.axis();
	float tmp3 = 1.0 / (transformations.size() + 1);
	mean_rot = (transformations.size()*mean_rot + trans_tmp_rot)*tmp3;
	mean_trans = (transformations.size()*mean_trans + transformation.translation())*tmp3;

	float mean_angle = mean_rot.norm();
	mean_transformation = Eigen::Affine3f(Eigen::Translation3f(mean_trans)*Eigen::AngleAxisf(mean_angle, mean_rot.normalized()));
	//cout << Eigen::AngleAxisf(mean_angle, mean_rot.normalized()).matrix() << endl;
	//cout << mean_transformation.matrix() << endl;
	transformations.push_back(transformation);
	voteLists.push_back(vote);
	vote_count += vote;
	return true;

}
