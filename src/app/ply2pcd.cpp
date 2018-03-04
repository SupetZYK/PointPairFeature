#include <util_pcl.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
std::string model_filename_;
std::string save_filename_;


void showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.ply save_filename_.pcd [Options]" << std::endl << std::endl;
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
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
	if (filenames.size() < 1)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}
	model_filename_ = argv[filenames[0]];
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() < 1)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}
	save_filename_=argv[filenames[0]];
}





int
main(int argc, char *argv[])
{
	parseCommandLine(argc, argv);

	showHelp(argv[0]);
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());

        if (!readPointCloud(model_filename_, model) )
		{
			return(-1);
		}

	//
	//  Set up resolution invariance
	//

	//
	//visualize keypoints
	//

		pcl::visualization::PCLVisualizer keyPointVisual("keyPoint detect");
		//scene
		// keyPointVisual.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(scene, 255.0, 0.0, 0.0), "scene");

		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");

		//model
		// keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(model, 255.0, 0.0, 0.0), "model");
		keyPointVisual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<PointType>(model, 0.0, 0.0, 255.0), "model_keypoints");
		keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");
		//keyPointVisual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
		keyPointVisual.spin();

		cout<<"save to file: "<<save_filename_<<endl;
		 pcl::io::savePCDFile(save_filename_,*model);
	

	return 0;
}
