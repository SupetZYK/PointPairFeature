#include "common.h"
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"

using namespace std;
int
main(int argc, char *argv[])
{
	Eigen::AngleAxisf a(1, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf b(1, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf c(1, Eigen::Vector3f::UnitX());

	zyk::pose_cluster p(Eigen::Affine3f(a),1);
	p.checkAndPutIn(Eigen::Affine3f(b), 1, 100, 100);
	p.checkAndPutIn(Eigen::Affine3f(c), 1, 100, 100);

	Eigen::Affine3f tmp;
	p.getMeanTransformation(tmp);
	cout << tmp.rotation() << endl;

	Eigen::Vector3f a_rot = a.angle()*a.axis();
	Eigen::Vector3f b_rot = b.angle()*b.axis();
	Eigen::Vector3f c_rot = c.angle()*c.axis();

	cout << "rot vector" << endl;
	cout << a_rot.transpose() << endl;
	cout << b_rot.transpose() << endl;
	cout << c_rot.transpose() << endl;

	Eigen::AngleAxisf tmp2(tmp.rotation());
	cout << "mean rot" << endl;
	cout << tmp2.angle()*tmp2.axis().transpose() << endl;
	getchar();
	return 0;
}