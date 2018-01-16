#include <iostream>
#include <string.h>
#include <time.h>
#include "Detectors.h"
using namespace std;
//std::string surface_model_file = "../../../datafile/pipe.ppfs";
////std::string scene_file = "../../../datafile/pipe_scene.ply";
//std::string scene_file = "../../../datafile/pipes/5.ply";

std::string surface_model_file = "../../../datafile/cplat.ppfs";
std::string scene_file = "../../../datafile/plat_scene.ply";

//std::string surface_model_file = "../../../datafile/cylinder.ppfs";
//std::string scene_file = "../../../datafile/cylinder_scene/5.ply";
int main(int argc, char**argv) {
	//CDetectModel3D dm;
	//CDetectModel3D::trainOptions opt;
	//opt.downSampleRatio = 0.05;
	//opt.mlsOrder = 2;
	//dm.setTrainOptions(opt);
	//dm.createSurfaceModel("../../../datafile/pipe.ply");

	CDetectors3D detectors;
	//set detect parameters
	CDetectModel3D::detectOptions opt;
	opt.downSampleRatio = 0.05;
	opt.maxNumber = 20;
	opt.MaxOverlapDistRel = 0.5;
	opt.minScore = 0.4;
	opt.mlsOrder = 2;
	detectors.readScene(scene_file);
	CDetectModel3D* model = detectors.readSurfaceModel(surface_model_file);
	model->setDetectOptions(opt);
	long t1 = clock();
	detectors.findParts(0.2);
	long timespend = clock() - t1;
	
	detectors.showMatchResults();
	std::cout << "Time Used: " << timespend / 1000.0 << " s" << std::endl;
	system("pause");
	return 0;
}