#include <iostream>
#include <string.h>
#include "Detectors.h"
using namespace std;
//std::string surface_model_file = "../../../datafile/pipe.ppfs";
//std::string scene_file = "../../../datafile/pipe_scene.ply";

std::string surface_model_file = "../../../datafile/plat.ppfs";
std::string scene_file = "../../../datafile/plat_scene.ply";

//std::string surface_model_file = "../../../datafile/cylinder.ppfs";
//std::string scene_file = "../../../datafile/cylinder_scene/5.ply";
int main(int argc, char**argv) {

	CDetectors3D detectors;
	
	//set detect parameters
	CDetectModel3D::detectOptions opt;
	opt.downSampleRatio = 0.05;
	opt.maxNumber = 20;
	opt.MaxOverlapDistRel = 0.5;
	opt.minScore = 0.3;
	opt.mlsOrder = 1;

	detectors.readScene(scene_file);
	CDetectModel3D* model = detectors.readSurfaceModel(surface_model_file);
	model->setDetectOptions(opt);
	detectors.findParts(0.3);

	//detectors.showMatchResults();
	//system("pause");
	return 0;
}