#ifndef __OPENCV_UTILS_H__
#define __OPENCV_UTILS_H__
#include <opencv2/core.hpp>
#include <limits>

namespace cv
{
CV_EXPORTS_W
void
depthTo3d(InputArray depth, InputArray K, OutputArray points3d, InputArray mask = noArray());

/** If the input image is of type CV_16UC1 (like the Kinect one), the image is converted to floats, divided
* by 1000 to get a depth in meters, and the values 0 are converted to std::numeric_limits<float>::quiet_NaN()
* Otherwise, the image is simply converted to floats
* @param in the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
*              (as done with the Microsoft Kinect), it is assumed in meters)
* @param depth the desired output depth (floats or double)
* @param out The rescaled float depth image
*/
CV_EXPORTS_W
void
rescaleDepth(InputArray in, int depth, OutputArray out);
}

#endif
