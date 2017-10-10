#include "common.h"
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"


extern "C" {
	zyk::PPF_Space ppf_detector;
	
	ZYK_EXPORTS int python_test_func(char* p, int len)
	{
		string s(p,len);
		cout << "file name is: " << s << endl;
		if (ppf_detector.load(s))
		{
			printf("load success!");
		}
		else
		{
			printf("load filed");
			return 0;
		}
		printf("model_res: %f", ppf_detector.model_res);
		return 1;
	}

	ZYK_EXPORTS int foo(int a, int b)
	{
		printf("you input %d and %d\n", a, b);
		return a + b;
	}
}