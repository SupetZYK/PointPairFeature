#include <iostream>
#include <string.h>
#include <time.h>
#include "Detectors.h"
using namespace std;
std::string model_file = "../../../datafile/plat.obj";

int main(int argc, char**argv) {


	CDetectModel3D detectorModel;
	//set detect parameters
	CDetectModel3D::trainOptions opt;
	opt.downSampleRatio = 0.06;
	opt.mlsOrder = 1;

	detectorModel.setTrainOptions(opt);
	detectorModel.createSurfaceModel(model_file, "../../../datafile/plat_d.ppfs");
	system("pause");
	return 0;
}