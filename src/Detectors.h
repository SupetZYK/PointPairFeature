#pragma once
#include <string>
#include <map>
#include <vector>
#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)&& defined zyk_ppf_EXPORTS
# define ZYK_EXPORTS  __declspec(dllexport)
#elif defined __GNUC__ && __GNUC__ >= 4
#  define ZYK_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define ZYK_EXPORTS
#endif

using namespace std;
namespace zyk{
	class PPF_Space;
};
//��ʶ��3Dģ����
class ZYK_EXPORTS CDetectModel3D
{
	friend class CDetectors3D;
	typedef struct {
		double downSampleRatio;
		double minScore;
		double MaxOverlapDistRel;
		int maxNumber;
		int mlsOrder;
	} detectOptions;
	typedef struct {
		double downSampleRatio;
		int mlsOrder;
	}trainOptions;

	typedef struct {
		double x;
		double y;
		double z;
	} Vec3d;
	typedef struct {
		bool matchComplete;
		vector<Vec3d> rotatios;
		vector<Vec3d> translations;
		vector<double> scores;
		void clear() { matchComplete = false; rotatios.clear(); translations.clear(); scores.clear(); }
	} matchResult;

public:
	detectOptions mDetectOptions;
	trainOptions mTrainOptions;
	std::string ObjectName; 
	std::string ShowColor; 
public:
	CDetectModel3D();
	~CDetectModel3D();
	// Short Description: ��ȡsurfaceģ������
	bool readSurfaceModel(string filePath);
	// Short Description: ��CAD�ļ��д���surfaceģ��
	// bool createSurModelFromCADFile(const string &filePath);
	// Short Description: ����surfaceģ������
	void clearSurModel();
	// ��ȡ�������ģ�͵���,������Ŀ
	// int getSampledModel(double*x, double*y, double*z, double*nx = NULL, double*ny = NULL, double*nz = NULL);
	matchResult result;
private:
	zyk::PPF_Space* p_PPF;
};


class ZYK_EXPORTS CDetectors3D
{
	typedef CDetectModel3D::Vec3d Vec3d;
	typedef CDetectModel3D::matchResult matchResult;

public:
	~CDetectors3D() { clear(); };
	bool readScene(const string filePath, const string &unit = "m");
	bool findPart(const string objectName, double keyPointRatio=0.2,const bool smoothFlag = true);
	bool findParts(double keyPointRatio = 0.2,const bool smoothflag = true);
	bool readSurfaceModel(string filePath);
	void showMatchResults();
	void clear();
public:
	std::map <std::string, CDetectModel3D*> detectObjects;//ʰȡ���󼯺�
};

