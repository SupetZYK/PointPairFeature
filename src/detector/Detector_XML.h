#pragma once
#include "tinyxml.h"
#include "tinystr.h"

#include "Detectors_2.h"

class Detector_XML
{
public:
	Detector_XML(){  }
	static bool Read3DdetectSetingParams(std::string filename, CDetectors3D::detectorParams &DetectorParams, vector <CDetectModel3D::detectObjParams> &DetectObjsParams);
	static bool parseDetectorParams(TiXmlHandle hDoc, CDetectors3D::detectorParams &DetectorParams);
	static bool parseDetectObjParams(TiXmlHandle hDoc, vector <CDetectModel3D::detectObjParams> &DetectObjsParams);
	
	static void Write3DdetectSetingParams(const std::string &filename, const CDetectors3D::detectorParams &DetectorParams, const vector <CDetectModel3D::detectObjParams> &DetectObjsParams);
	
};
