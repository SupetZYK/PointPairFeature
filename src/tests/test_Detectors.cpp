
#include <string.h>
#include "Detectors.h"
using namespace std;
//std::string surface_model_file = "../../../datafile/pipe.ppfs";
//std::string scene_file = "../../../datafile/pipe_scene.ply";
std::string surface_model_file = "../../../datafile/plat.ppfs";
std::string scene_file = "../../../datafile/plat_scene.ply";
int main(int argc, char**argv) {

	CDetectors3D detectors;
	detectors.readSurfaceModel(surface_model_file);
	detectors.readScene(scene_file);
	detectors.findParts(0.2);
	detectors.showMatchResults();
	system("pause");
	return 0;
}