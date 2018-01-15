#include <iostream>
#include <fstream>
#include <string.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/uniform_sampling.h> //һ���²���
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>//����
#include <pcl/kdtree/kdtree.h>//KD��
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h> 
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>


using namespace std;
using namespace pcl;

//some globals    ����3Dƥ������ߵģ�2  0.005   �����͵� 3 0.0001
std::string scene_filename;
std::string model_filename;
int max_iterations(100);
double max_dis(3);
double Epsilon(0.001);
//��ȡtxt�����ļ�
bool readfileTest(const string filename, PointCloud<PointXYZ>::Ptr &pnts)
{
	filebuf *pbuf;
	ifstream fileread;
	long size;
	char * buffer;
	// Ҫ���������ļ���������ö����ƴ�   
	fileread.open(filename, ios::binary);
	// ��ȡfilestr��Ӧbuffer�����ָ�루�������������ָ�룩
	pbuf = fileread.rdbuf();

	// ����buffer���󷽷���ȡ�ļ���С  ������λλ��ָ��ָ���ļ���������ĩβʱ��pubseekoff���ص�ֵ���������ļ�����С��
	size = pbuf->pubseekoff(0, ios::end, ios::in);
	pbuf->pubseekpos(0, ios::in);   //����pbufָ���ļ�����ʼλ��

	// �����ڴ�ռ�  
	buffer = new char[size];

	// ��ȡ�ļ�����  
	pbuf->sgetn(buffer, size);

	fileread.close();


	// �������׼���  

	//��ѷ������ַ���������buffer
	string temp = "";
	PointXYZ Pnts;

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
				if (!isy)  //�����x��ֵ
				{
					if (!temp.empty())
					{
						isy = !isy;
						sscanf(temp.data(), "%f", &x);
						Pnts.x = x;
						temp = "";
					}
				}
				else                  //�����y��ֵ
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
		else   //������z
		{
			if (!temp.empty())
			{
				sscanf(temp.data(), "%f", &z);
				Pnts.z = z;
				temp = "";
				pnts->push_back(Pnts);
			}
		}
		buffer++;
	}
	return true;
}
//��������ܶ�
float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k)
{
	double res = 0.0;
	int n_points = 0;

	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}

		std::vector<int> indices(k);
		std::vector<float> sqr_distances(k);

		if (tree.nearestKSearch(i, k, indices, sqr_distances) == k)
		{
			for (int i = 1; i < k; i++)
			{
				res += sqrt(sqr_distances[i]);
				++n_points;
			}
		}
	}

	if (n_points != 0)
	{
		res /= n_points;
	}

	return res;
}
//����̨����
void parseCommand(int argc, char**argv){
	vector<int>file_names;
	file_names = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
	if (file_names.size()<2){
		std::cout << "too few files input!" << std::endl;
		system("pause");
		exit(-1);
	}
	scene_filename = argv[file_names[0]];
	model_filename = argv[file_names[1]];

	pcl::console::parse_argument(argc, argv, "--mi", max_iterations);
	pcl::console::parse_argument(argc, argv, "--max_d", max_dis);
	pcl::console::parse_argument(argc, argv, "--ep", Epsilon);

}

int main(int argc, char** argv)
{
	parseCommand(argc, argv);
	std::cout << "scene: " << scene_filename << std::endl;
	std::cout << "model: " << model_filename << std::endl;
	std::cout << "max iteration: " << max_iterations << std::endl;
	std::cout << "max dis: " << max_dis << std::endl;
	std::cout << "Epsilon: " << Epsilon << std::endl;
	srand(time(0));  
	clock_t start, end,startall,endall;
	PointCloud<PointXYZ>::Ptr ModelCloud = boost::make_shared <PointCloud<PointXYZ>>();
	PointCloud<PointXYZ>::Ptr SceneCloud = boost::make_shared <PointCloud<PointXYZ>>();
	PointCloud<PointXYZ>::Ptr SampleCloud = boost::make_shared <PointCloud<PointXYZ>>();
	PointCloud<PointXYZ>::Ptr ModelCloud_icp = boost::make_shared <PointCloud<PointXYZ>>();
	PointCloud<PointXYZ>::Ptr orgModel = boost::make_shared <PointCloud<PointXYZ>>();
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;
	PointCloud<PointNormal>::Ptr ModelCloud_Normal(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr orgModel_Normal(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr SceneCloud_Normal(new PointCloud<PointNormal>);
	PointCloud<PointXYZ>::Ptr icpResultCloud = boost::make_shared <PointCloud<PointXYZ>>();


	std::cout << "*********************   ��ȡ����    *************************" << std::endl;
	//��ȡ��������
	if (pcl::io::loadPLYFile<PointXYZ>("../../../datafile/data/plat_scene.ply", *SceneCloud) == -1)//pipe_scene , plat_scene
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		std::cout << "��ȡPLY�ļ�����" << std::endl;
		return (-1);
	}
	//readfileTest(scene_filename, SceneCloud);

	//�²���
#if 0
	int method = 3;
	if (method < 3)
	{
		std::cout << "*********************   ��ʼ�²���    *************************" << std::endl;
		double relSamplingDistance = 4;
		if (method == 0)//����
		{
			std::cout << "�²���ǰ�ĵ��Ƹ�����" << SceneCloud->size() << endl;
			pcl::UniformSampling<PointXYZRGBNormal> uniform_sampling;
			uniform_sampling.setInputCloud(SceneCloud);
			uniform_sampling.setRadiusSearch(relSamplingDistance);
			uniform_sampling.filter(*SampleCloud);
			std::cout << "�²�����ĵ��Ƹ�����" << SampleCloud->size() << endl;
		}
		else if (method == 1)//�����������
		{
			std::cout << "�²���ǰ�ĵ��Ƹ�����" << SceneCloud->size() << endl;
			pcl::VoxelGrid<PointXYZRGBNormal> sor;
			sor.setInputCloud(SceneCloud);
			sor.setLeafSize(relSamplingDistance, relSamplingDistance, relSamplingDistance);  //��λcm
			sor.filter(*SampleCloud);
			std::cout << "�²�����ĵ��Ƹ�����" << SampleCloud->size() << endl;
		}
		else if (method == 2)//��������
		{
			std::cout << "�²���ǰ�ĵ��Ƹ�����" << SceneCloud->size() << endl;
			pcl::ApproximateVoxelGrid<PointXYZRGBNormal> sor;
			sor.setInputCloud(SceneCloud);
			sor.setLeafSize(relSamplingDistance, relSamplingDistance, relSamplingDistance);  //��λcm
			sor.filter(*SampleCloud);
			std::cout << "�²�����ĵ��Ƹ�����" << SampleCloud->size() << endl;
		}
	}
#endif
	//��ȡģ�����
	readfileTest(model_filename, orgModel);

	//ģ����ƴ�ƥ��λ��
	PointCloud<PointXYZ>::Ptr rot = boost::make_shared <PointCloud<PointXYZ>>();
	PointCloud<PointXYZ>::Ptr trans = boost::make_shared <PointCloud<PointXYZ>>();
	std::cout << "*********************   ��ȡλ��    *************************" << std::endl;
	readfileTest("../../../datafile/data/rot.txt", rot);
	readfileTest("../../../datafile/data/trans.txt", trans);

	if (rot->size() != trans->size())
	{
		std::cout << "��ת��ƽ�ƾ���������ͬ" << endl;
			return 0;
	}
	else
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ICP Viewer"));
		int maxIterations = 0;
		double allscore = 0;
		double icpscore = 0;
#if 0
		//�ȼ�����Ʒ���
		PointCloud<PointXYZ>::Ptr Norms = boost::make_shared <PointCloud<PointXYZ>>();
		readfileTest("./data/Norms.txt", Norms);
		//XYZ��Ϣ��ֵ
		pcl::copyPointCloud(*orgModel, *orgModel_Normal);
		//��Model������Ϣ����
		for (int i = 0; i < Norms->size(); i++)
		{
		orgModel_Normal->points.at(i).normal_x = Norms->at(i).x;
		orgModel_Normal->points.at(i).normal_y = Norms->at(i).y;
		orgModel_Normal->points.at(i).normal_z = Norms->at(i).z;
		}

		normal_estimator.setSearchMethod(tree);                        //���ù��ƶ�����õ���������
		normal_estimator.setKSearch(10);                                //���ù���ʱ���������õ�k��

		//normal_estimator.setInputCloud(orgModel);
		//normal_estimator.compute(*orgModel_Normal);           //����ֱ����Դ��Ŀ����Ʒ���
		//pcl::copyPointCloud(*orgModel, *orgModel_Normal);
		//for (int i = 0; i < orgModel_Normal->size(); i++)
		//{
		//	PointXYZ pnt;
		//	pnt.x = orgModel_Normal->points.at(i).x;
		//	pnt.y = orgModel_Normal->points.at(i).y;
		//	pnt.z = orgModel_Normal->points.at(i).z;
		//	Eigen::Matrix<float, 4, 1> pntNormal;
		//	pntNormal[0] = orgModel_Normal->points.at(i).normal_x;
		//	pntNormal[1] = orgModel_Normal->points.at(i).normal_y;
		//	pntNormal[2] = orgModel_Normal->points.at(i).normal_z;
		//	pntNormal[3] = 0;
		//	flipNormalTowardsViewpoint(pnt, 0, 0, 0, pntNormal);
		//	orgModel_Normal->points.at(i).normal_x = pntNormal[0];
		//	orgModel_Normal->points.at(i).normal_y = pntNormal[1];
		//	orgModel_Normal->points.at(i).normal_z = pntNormal[2];
		//}

		normal_estimator.setInputCloud(SceneCloud);
		normal_estimator.compute(*SceneCloud_Normal);
		pcl::copyPointCloud(*SceneCloud, *SceneCloud_Normal);
		for (int i = 0; i < SceneCloud_Normal->size(); i++)
		{
			PointXYZ pnt;
			pnt.x = SceneCloud_Normal->points.at(i).x;
			pnt.y = SceneCloud_Normal->points.at(i).y;
			pnt.z = SceneCloud_Normal->points.at(i).z;
			Eigen::Matrix<float, 4, 1> pntNormal;
			pntNormal[0] = SceneCloud_Normal->points.at(i).normal_x;
			pntNormal[1] = SceneCloud_Normal->points.at(i).normal_y;
			pntNormal[2] = SceneCloud_Normal->points.at(i).normal_z;
			pntNormal[3] = 0;
			flipNormalTowardsViewpoint(pnt, 0, 0, 0, pntNormal);
			SceneCloud_Normal->points.at(i).normal_x = pntNormal[0];
			SceneCloud_Normal->points.at(i).normal_y = pntNormal[1];
			SceneCloud_Normal->points.at(i).normal_z = pntNormal[2];
		}
#endif

		//��������ܶ�
		float ModelRes = computeCloudResolution(orgModel, 2);
		float SceneRes = computeCloudResolution(SceneCloud, 2);
		std::cout << "ģ������ܶ�" << ModelRes << endl;
		std::cout << "���������ܶ�" << SceneRes << endl;


		startall = clock();
		for (int i = 0; i < rot->size(); i++)
		{
			start = clock();
			std::cout << "*********************   ��ʼ�� "<<i+1<<"��λ��ICP    *************************" << std::endl;

			ModelCloud->clear();
			ModelCloud_icp->clear();
			icpResultCloud->clear();
			ModelCloud_Normal->clear();
			for (int j = 0; j < orgModel->size(); j++)
			{
				PointXYZ pnt;
				pnt.x = (*orgModel)[j].x;
				pnt.y = (*orgModel)[j].y;
				pnt.z = (*orgModel)[j].z;
				ModelCloud->push_back(pnt);
				ModelCloud_icp->push_back(pnt);
			}

			Eigen::Vector3f rot1, trans1;
			rot1.x() = (*rot)[i].x;
			rot1.y() = (*rot)[i].y;
			rot1.z() = (*rot)[i].z;
			trans1.x() = (*trans)[i].x;
			trans1.y() = (*trans)[i].y;
			trans1.z() = (*trans)[i].z;

			double ang = rot1.norm();
			rot1 /= ang;
			Eigen::Affine3f modelTransformation = Eigen::Affine3f(Eigen::Translation3f(trans1)*Eigen::AngleAxisf(ang, rot1));
			pcl::transformPointCloud<PointXYZ>(*ModelCloud, *ModelCloud, modelTransformation);
			pcl::transformPointCloud<PointXYZ>(*ModelCloud_icp, *ModelCloud_icp, modelTransformation);

			//����icp
			//1. setMaximumIterations�� ������������icp��һ�������ķ�������������Щ�Σ�����Ͽ��ӻ��������ʾ���ɽ���������Ϊ1����
			//2. setEuclideanFitnessEpsilon�� �������������Ǿ�������С����ֵ�� ֹͣ������
			//3. setTransformtionEpsilon, �������α仯����֮��Ĳ�ֵ��һ������Ϊ1e - 10���ɣ���
			//4. setMaxCorrespondenaceDistance, ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
#if 1
			pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
			icp.setMaximumIterations(max_iterations);//500
			icp.setInputSource(ModelCloud);
			icp.setInputTarget(SceneCloud);
			icp.setTransformationEpsilon(1e-10);//ǰ����������ֵ
			icp.setEuclideanFitnessEpsilon(Epsilon);//��������ֵ
			double t = double(SceneRes)*pow(2, 0.5) + 0.05;
			icp.setMaxCorrespondenceDistance(t);//max_dis//���ֵ������ù��󽫵��ºܶ�ǹ��������ƥ�䣬������׼���ȱ�0.7
			icp.align(*icpResultCloud);
			std::cout << "��Ϸ���" << icp.getFitnessScore() << endl;
			std::cout << "��������" << icp.nr_iterations_ << endl;
			allscore += icp.getFitnessScore();//sum of squared distances from the source to the target
			if (icp.nr_iterations_ == max_iterations)
			{
				maxIterations++;
			}
			Eigen::Matrix4d finalTransformation = icp.getFinalTransformation().cast<double>();
			pcl::transformPointCloud<PointXYZ>(*ModelCloud_icp, *ModelCloud_icp, finalTransformation);

			//���㾫ƥ�����:ͳ��ÿ�����ƥ����룬С�ڳ��������ܶ�Ϊ�ϸ�㣬���кϸ��ռģ����Ʊ���Ϊ����
			int okNumber = 0;
			for (int j = 0; j < icp.correspondences_->size(); j++)//�˴��õ���distanceΪ��Ծ����ƽ����Դ�룺d:\Program Files\PCL 1.8.0\include\pcl-1.8\pcl\registration\impl\correspondence_estimation.hpp
			{
				if (icp.correspondences_->at(j).distance <= SceneRes*SceneRes)
				{
					okNumber++;
				}
			}
			double icpMatchScore = (double)okNumber / orgModel->size();
			std::cout << "icp����׼����" << icpMatchScore*100 <<"%"<< endl;
			icpscore+=icpMatchScore*100;
#endif

			//����icp  plane to plane
			//pcl::GeneralizedIterativeClosestPoint< PointSource, PointTarget > Class
#if 0
			pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
			gicp.setInputCloud(ModelCloud);
			gicp.setInputTarget(SceneCloud);
			gicp.setMaximumIterations(max_iterations);
			gicp.setTransformationEpsilon(1e-10);
			gicp.setEuclideanFitnessEpsilon(Epsilon);
			gicp.setMaxCorrespondenceDistance(max_dis);
			gicp.align(*icpResultCloud);
			std::cout << "��Ϸ���" << gicp.getFitnessScore() << endl;
			std::cout << "��������" << gicp.nr_iterations_ << endl;
			allscore += gicp.getFitnessScore();//sum of squared distances from the source to the target
			if (gicp.nr_iterations_ == max_iterations)
			{
				maxIterations++;
			}
			Eigen::Matrix4d finalTransformation = gicp.getFinalTransformation().cast<double>();
			pcl::transformPointCloud<PointXYZRGB>(*ModelCloud_icp, *ModelCloud_icp, finalTransformation);

#endif

			//point to plane ���뷨����Ϣ,  ���ٵ�������³���Բ���
			//pcl::IterativeClosestPointWithNormals<PointXYZRGB, PointXYZRGB> normal;
#if 0
			pcl::copyPointCloud(*orgModel_Normal, *ModelCloud_Normal);
			pcl::transformPointCloudWithNormals(*ModelCloud_Normal, *ModelCloud_Normal, modelTransformation);

			pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> nicp;
			nicp.setInputSource(ModelCloud_Normal);
			nicp.setInputTarget(SceneCloud_Normal);
			nicp.setMaximumIterations(max_iterations);
			nicp.setTransformationEpsilon(1e-10);
			nicp.setEuclideanFitnessEpsilon(Epsilon);
			nicp.setMaxCorrespondenceDistance(max_dis);
			nicp.align(*icpResultCloud);
			std::cout << "��Ϸ���" << nicp.getFitnessScore() << endl;
			std::cout << "��������" << nicp.nr_iterations_ << endl;
			allscore += nicp.getFitnessScore();//sum of squared distances from the source to the target
			if (nicp.nr_iterations_ == max_iterations)
			{
				maxIterations++;
			}
			Eigen::Matrix4d finalTransformation = nicp.getFinalTransformation().cast<double>();
			pcl::transformPointCloud<PointXYZ>(*ModelCloud_icp, *ModelCloud_icp, finalTransformation);

			//���㾫ƥ�����:ͳ��ÿ�����ƥ����룬С�ڳ��������ܶ�Ϊ�ϸ�㣬���кϸ��ռģ����Ʊ���Ϊ����
			int okNumber = 0;
			for (int j = 0; j < nicp.correspondences_->size(); j++)//�˴��õ���distanceΪ��Ծ����ƽ����Դ�룺d:\Program Files\PCL 1.8.0\include\pcl-1.8\pcl\registration\impl\correspondence_estimation.hpp
			{
				if (nicp.correspondences_->at(j).distance <= SceneRes*SceneRes)
				{
					okNumber++;
				}
			}
			double icpMatchScore = (double)okNumber / orgModel->size();
			std::cout << "icp����׼����" << icpMatchScore*100 <<"%"<< endl;
			nicpscore+=icpMatchScore*100;
#endif

			//LM�Ż��㷨ICP����������С����
			//pcl::IterativeClosestPointNonLinear< PointSource, PointTarget, Scalar > Class
#if 0
			PointCloud<PointNormal>::Ptr ModelCloud_Normal(new PointCloud<PointNormal>);
			PointCloud<PointNormal>::Ptr SceneCloud_Normal(new PointCloud<PointNormal>);
			PointCloud<PointNormal>::Ptr _Normal(new PointCloud<PointNormal>);
			normal_estimator.setSearchMethod(tree);                        //���ù��ƶ�����õ���������
			normal_estimator.setKSearch(10);                                //���ù���ʱ���������õ�k��
			normal_estimator.setInputCloud(ModelCloud);
			normal_estimator.compute(*ModelCloud_Normal);           //����ֱ����Դ��Ŀ����Ʒ���
			pcl::copyPointCloud(*ModelCloud, *ModelCloud_Normal);
			normal_estimator.setInputCloud(SceneCloud);
			normal_estimator.compute(*SceneCloud_Normal);
			pcl::copyPointCloud(*SceneCloud, *SceneCloud_Normal);

			pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> LMicp;
			LMicp.setInputCloud(ModelCloud_Normal);
			LMicp.setInputTarget(SceneCloud_Normal);
			LMicp.setMaximumIterations(max_iterations);
			LMicp.setTransformationEpsilon(1e-10);
			LMicp.setEuclideanFitnessEpsilon(Epsilon);
			LMicp.setMaxCorrespondenceDistance(max_dis);
			LMicp.align(*_Normal);
			std::cout << "��Ϸ���" << LMicp.getFitnessScore() << endl;
			std::cout << "��������" << LMicp.nr_iterations_ << endl;
			allscore += LMicp.getFitnessScore();//sum of squared distances from the source to the target
			if (LMicp.nr_iterations_ == max_iterations)
			{
				maxIterations++;
			}
			Eigen::Matrix4d finalTransformation = LMicp.getFinalTransformation().cast<double>();
			pcl::transformPointCloud<PointXYZRGB>(*ModelCloud_icp, *ModelCloud_icp, finalTransformation);
#endif

			//��ʾICP���
			pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> SceneCloud_color(SceneCloud, 0, 200, 200);
			pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> ModelCloud_color(ModelCloud, 255, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> ModelCloud_icp_color(ModelCloud_icp, 0, 255, 0);
			string strSceneCloud = "SceneCloud";
			string strModelCloud = "ModelCloud";
			string strModelCloud_icp = "strModelCloud_icp";
			string strnormal = "strnormal";
			std::stringstream ss;
			std::string cloudNumstr;
			ss << i;
			ss >> cloudNumstr;
			strSceneCloud += cloudNumstr;
			strModelCloud += cloudNumstr;
			strModelCloud_icp += cloudNumstr;
			strnormal += cloudNumstr;
			viewer->addPointCloud(SceneCloud, SceneCloud_color, strSceneCloud);
			viewer->addPointCloud(ModelCloud, ModelCloud_color, strModelCloud);
			viewer->addPointCloud(ModelCloud_icp, ModelCloud_icp_color, strModelCloud_icp);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, strSceneCloud);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, strModelCloud);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, strModelCloud_icp);
			//��ʾģ����Ʒ���	
			//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(ModelCloud_icp, icpResultCloud, 1, 2, strnormal);
			
			end = clock();
			std::cout << (double)(end - start) / CLOCKS_PER_SEC << " s" << std::endl;
		}
		endall = clock();

		std::cout <<endl<< "����������icp����: " << maxIterations << endl;
		std::cout << "�ܷ�: " << allscore << endl;
		std::cout << "�ܹ���ʱ�� " << (double)(endall - startall) / CLOCKS_PER_SEC << " s" << std::endl;
		std::cout << endl << "icp��׼ƥ���ܷ�: " << icpscore << endl;
		//��ʾ�������Ʒ���
		//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(SceneCloud, SceneCloud_Normal, 1, 2, "SceneCloud_Normal");
		viewer->spin();
	}

	return 0;
}
	

