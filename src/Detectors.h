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
/** Class that implement the core algorithm, this is unaccessible!
*/
namespace zyk{
	class PPF_Space;
};
/** 3D surface model class
*/
class ZYK_EXPORTS CDetectModel3D
{
	friend class CDetectors3D;
	

public:
	/** struct for detect options
	*@ downSampleRatio, 0.05 by default, relative to the diameter of the model bounding box
	*@ minScore, 0.3 by default, minimum score of the detected poses to show
	*@ MaxOverlapDistRel, 0.5 by default, minimum distance between any two poses, relative to the diameter of the model bounding box
	*@ maxNumber, 10 by default, max number of poses to store
	*@ mlsOrder, 1 by default, smooth order
	*/
	typedef struct {
		double downSampleRatio;
		double minScore;
		double MaxOverlapDistRel;
		int maxNumber;
		int mlsOrder;
	} detectOptions;
	/** struct for train options
	*@ downSampleRatio, 0.05 by default, relative to the diameter of the model bounding box
	*@ mlsOrder, 1 by default, smooth order
	*/
	typedef struct {
		double downSampleRatio;
		int mlsOrder;
	}trainOptions;
	/** struct for 3D vector
	*/
	typedef struct {
		double x;
		double y;
		double z;
	} Vec3d;
	/** struct for match results
	*@ matchComplete, false by default, a flag to stand for whether the match is completed
	*@ rotatios, vector to store detected rotation vectors, sorted by scores (large to small)
	*@ translations, vector to store detected translation vecors, sorted by scores (large to small)
	*@ scores, vectors to store scores.
	*/
	typedef struct {
		bool matchComplete;
		vector<Vec3d> rotatios;
		vector<Vec3d> translations;
		vector<double> scores;
		/** function to clear the struct
		*/
		void clear() { matchComplete = false; rotatios.clear(); translations.clear(); scores.clear(); }
	} matchResult;



public:
	CDetectModel3D();
	~CDetectModel3D();
	/** read a pre-created surface model
	*@ filepath, path of the model file(.ppfs format only)
	*/
	bool readSurfaceModel(string filePath);
	/** create a surface model from a model point cloud
	*@ filePath, path of the model point cloud, .ply format only
	*@ savePath, path of the surface model to be created 
	*/
	bool createSurfaceModel(string filePath, string savePath="");
	/** clear the surface model
	*/
	void clearSurModel();
	/** get detect options
	*/
	const detectOptions& getDetectOptions() const { return mDetectOptions; };
	/** set detect options
	*@ opt, options to set
	*/
	void setDetectOptions(detectOptions& opt) { mDetectOptions = opt; };
	/** get train options
	*/
	const trainOptions& getTrainOptions() const { return mTrainOptions; };
	/** set train options
	*@ opt, options to set
	*/
	void setTrainOptions(trainOptions& opt) { mTrainOptions = opt; };
	/** get the name of the model
	*/
	const string& getModelName() const { return ObjectName; };
	/** get match result
	*/
	const matchResult& getMatchResult() const { return result; };
	/** get model point cloud
	*/
	void getModelPointCloud(vector<Vec3d>& modelPoints);
	/** get model point normals
	*/
	void getModelPointNormals(vector<Vec3d>& modelNormals);
protected:
	detectOptions mDetectOptions;
	trainOptions mTrainOptions;
	std::string ObjectName;
	matchResult result;
private:
	zyk::PPF_Space* p_PPF;
};



/** 3D detector class
*/
class ZYK_EXPORTS CDetectors3D
{
	typedef CDetectModel3D::Vec3d Vec3d;
	typedef CDetectModel3D::matchResult matchResult;

public:
	~CDetectors3D() { clear(); };
	/** read in the scene point cloud
	*@ filePath, path of the scene point cloud file, (.ply format only)
	*/
	bool readScene(const string filePath);
	/** read in the scene point cloud
	*@ pointCloud, pointClouds
	*/
	bool readScene(const vector<Vec3d>& pointCloud);
	/** detect the specified model, the detected results is stored the model class
	*@ objectName, name of the model to be detected
	*@ keyPointRatio, key point to be used. 1/5 is recommended, large value may not improve the accuracy but slow down the process greatly
	*/
	bool findPart(const string objectName, double keyPointRatio=0.2);
	/** detect all models in the detectors
	*@ keyPointRatio, key point to be used. 0.2 is recommended, large value may not improve the accuracy but slow down the process greatly
	*/
	bool findParts(double keyPointRatio = 0.2);
	/** read in a pre-created surface model and return its pointer
	*@ filePath, path of the surface model file
	*/
	CDetectModel3D* readSurfaceModel(string filePath);
	/** only for test use, show the detected results
	*/
	void showMatchResults();
	/** clear all surface models
	*/
	void clear();
protected:
	std::map <std::string, CDetectModel3D*> detectObjects;
};

