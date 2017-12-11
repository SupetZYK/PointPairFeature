#include "pcl/point_types.h"
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
std::string model_filename_;
std::string save_filename_;


float normal_r(2.0);
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
using namespace std;
void showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.txt" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                     Show this help." << std::endl;
}

void parseCommandLine(int argc, char *argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Program behavior
	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
	if (filenames.size() < 1)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}
	model_filename_ = argv[filenames[0]];
	int pos = model_filename_.find_last_of('.');
	save_filename_ = model_filename_.substr(0, pos);
	save_filename_ += ".ply";
}


using namespace std;  
  
typedef struct tagPOINT_3D  
{  
    double x;  //mm world coordinate x  
    double y;  //mm world coordinate y  
    double z;  //mm world coordinate z  
    double r;  
}POINT_WORLD;  
  
int main(int argc,char**argv)  
{  
	parseCommandLine(argc, argv);
	//
	// load txt
	//
    int number_Txt;  
    FILE *fp_txt;  
    tagPOINT_3D TxtPoint;  
    vector<tagPOINT_3D> m_vTxtPoints;  
    fp_txt = fopen(model_filename_.c_str(), "r");  
  
    if (fp_txt)  
    {  
        while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)  
        {  
            m_vTxtPoints.push_back(TxtPoint);  
        }  
    }  
    else  
        cout << "txt数据加载失败" << endl;  
    number_Txt = m_vTxtPoints.size();  
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  
  
    // Fill in the cloud data  
    cloud->width = number_Txt;  
    cloud->height = 1;     
    cloud->is_dense = false;  
    cloud->points.resize(cloud->width * cloud->height);  
  
  
    for (size_t i = 0; i < cloud->points.size(); ++i)  
    {  
        cloud->points[i].x = m_vTxtPoints[i].x;  
        cloud->points[i].y = m_vTxtPoints[i].y;  
        cloud->points[i].z = m_vTxtPoints[i].z;  
    }  
	
	//
	//visualize keypoints
	//

	pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
	keyPointVisual.addPointCloud(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointType>(cloud, 0.0, 0.0, 255.0), "model");
	keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
	keyPointVisual.spin();
	
	//
	// normal
	//
	pcl::PointCloud<NormalType>::Ptr compute_keyNormals(new pcl::PointCloud<NormalType>());
	if (1){
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		search::KdTree<PointType>::Ptr tree;
		//norm_est.setIndices(sampled_index_ptr);
		norm_est.setSearchMethod(tree);
		norm_est.setRadiusSearch(normal_r);
			
		//norm_est.setKSearch(10);
		norm_est.setInputCloud(cloud);
		norm_est.compute(*compute_keyNormals);
	}
	else{
		pcl::PointCloud<PointType>::Ptr pnts_tmp(new pcl::PointCloud<PointType>());
		MovingLeastSquares<PointType, PointType> mls;
		search::KdTree<PointType>::Ptr tree;
		// Set parameters
		mls.setInputCloud(cloud);
		mls.setComputeNormals(true);
		mls.setPolynomialFit(true);
		mls.setSearchMethod(tree);
		mls.setSearchRadius(normal_r);
		mls.process(*pnts_tmp);
		cloud = pnts_tmp;
		compute_keyNormals = mls.getNormals();
	}
	
	
	//
	// visualize point and normal
	//
	
	pcl::visualization::PCLVisualizer kp2("Compute normals");
	kp2.addPointCloud(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointType>(cloud, 0.0, 255.0, 0.0), "scene_keypoints");
	kp2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
	kp2.addPointCloudNormals<PointType, NormalType>(cloud, compute_keyNormals, 1, 10, "scene_normals");
	kp2.spin();
		
		
		
    system("pause");  
    return 0;  
}  