#include "util.h"
#include "Voxel_grid.h"
int main(int argc, char**argv) {

	Eigen::Vector3i grid_div(27, 39, 19);
	std::vector<int> res1;
	zyk::NeiboringIterator ni(grid_div.data(), 3);
	for (int i = 0; i <= 26; i += 26) {
		for (int j = 0; j <= 38; j += 38) {
			for (int k = 0; k <= 18; k += 18) {
				Eigen::Vector3i p(i, j, k);
				res1.clear();
				zyk::getNeiboringBoxIndex3D(p, grid_div, res1);
				std::sort(res1.begin(), res1.end());
				std::cout << "----Res1" << std::endl;
				for (int c = 0; c < res1.size(); ++c)
					std::cout << res1[c] << " ";
				std::cout << std::endl;
				ni.setTarget(p.data());
				for (++ni; !ni.isDone(); ++ni) {
					std::cout << ni.getIndex() << " ";
				}
				std::cout << std::endl;
			}
		}
			
	}
	//for (int num = 100; num < 900; num += 20) {

	//	res1.clear();
	//	zyk::getNeiboringBoxIndex3D(num, grid_div, res1);
	//	std::sort(res1.begin(), res1.end());
	//	std::cout << "----Res1" << std::endl;
	//	for (int i = 0; i < res1.size(); ++i)
	//		std::cout << res1[i] << " ";
	//	std::cout << std::endl;
	//	
	//	ni.setTarget(num);
	//	for (++ni; !ni.isDone(); ++ni) {
	//		std::cout << ni.getIndex() << " ";
	//	}
	//	std::cout << std::endl;
	//}
	system("pause");
	return 0;
}
