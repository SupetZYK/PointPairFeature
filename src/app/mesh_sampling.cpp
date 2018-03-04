/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/surface/mls.h>

#include <util.h>
#include <util_pcl.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

const float default_sample_ratio = 0.05;
//const int default_number_samples = 100000;
//const float default_leaf_size = 0.01f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.{ply,obj} output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     --ds X      = down sample ratio (default: ");
  print_value ("%d", default_sample_ratio);
  print_info (")\n");
  print_info ("                     --wn = flag to write normals to the output pcd\n");
  print_info (
              "                     --nv = flag to stop visualizing the generated pcd\n");
}

/* ---[ */
int
main (int argc, char **argv)
{
  print_info ("Convert a CAD model to a point cloud using uniform sampling. For more information, use: %s -h\n",
              argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
//  int SAMPLE_POINTS_ = default_number_samples;
//  parse_argument (argc, argv, "-n", SAMPLE_POINTS_);
//  float leaf_size = default_leaf_size;
//  parse_argument (argc, argv, "--ls", leaf_size);
  float down_sample_ratio = default_sample_ratio;
  parse_argument(argc,argv,"--ds",down_sample_ratio);
  std::string file_name;
  parse_argument(argc,argv,"--tar", file_name);
  bool vis_result = ! find_switch (argc, argv, "--nv");
  const bool write_normals = find_switch (argc, argv, "--wn");

  // Parse the command line arguments for .ply and PCD files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need a single output PCD file to continue.\n");
    return (-1);
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointNormal>);
  double min_coord[3],max_coord[3];
  if(!zyk::mesh_sampling(file_name,50000,*cloud_1,min_coord,max_coord))
  {
    print_error("Need a PLY/OBJ file input!\n");
    return (-1);
  }
  double diameter = zyk::dist(min_coord,max_coord,3);
  std::cout<<"model diameter is: "<<diameter<<std::endl;

//  //test mls
//  pcl::PointCloud<pcl::PointNormal>::Ptr pnts_tmp(new pcl::PointCloud<pcl::PointNormal>());
//  pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
//  pcl::search::KdTree<pcl::PointNormal>::Ptr tree;
//  // Set parameters
//  mls.setInputCloud(cloud_1);
//  mls.setComputeNormals(true);
//  mls.setPolynomialFit(true);
//  mls.setSearchMethod(tree);
//  mls.setSearchRadius(0.02*diameter);
//  mls.process(*pnts_tmp);

//  //reoint normal
//  for(size_t i=0;i<pnts_tmp->size();++i){
//    if(zyk::dot(pnts_tmp->at(i).normal,cloud_1->at(i).normal,3)<0)
//    {
//      pnts_tmp->at(i).normal_x*=-1;
//      pnts_tmp->at(i).normal_y*=-1;
//      pnts_tmp->at(i).normal_z*=-1;
//    }
//  }
//  cloud_1 = pnts_tmp;

  //
  //test use zyk method to resampling
  //
  pcl::PointCloud<PointType>::Ptr cloud_2 (new pcl::PointCloud<PointType>());
  pcl::PointCloud<NormalType>::Ptr Normal_2 (new pcl::PointCloud<NormalType>());
  pcl::PointCloud<PointType>::Ptr cloud_3 (new pcl::PointCloud<PointType>());
  pcl::PointCloud<NormalType>::Ptr Normal_3 (new pcl::PointCloud<NormalType>());

  pcl::copyPointCloud(*cloud_1,*cloud_2);
  pcl::copyPointCloud(*cloud_1,*Normal_2);
  zyk::uniformDownSamplePointAndNormal(cloud_2,Normal_2, down_sample_ratio*diameter,cloud_3,Normal_3);
  //test vis
  visualization::PCLVisualizer test_vis ("test_vis");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> test_color_handler(cloud_3, 255, 0, 0);
  test_vis.addPointCloud(cloud_3, test_color_handler, "model");
  test_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
  test_vis.addPointCloudNormals<PointType, NormalType>(cloud_3, Normal_3, 1, 10, "scene_normals");
  test_vis.spin();

  if (vis_result)
  {
    visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
    //vis3.addPointCloud<pcl::PointNormal> (cloud_1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> model_color_handler(cloud_1, 255, 0, 0);
    vis3.addPointCloud(cloud_1, model_color_handler, "model");
    vis3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
    if (write_normals){
      vis3.addPointCloudNormals<pcl::PointNormal> (cloud_1, 1, 0.05*diameter, "cloud_normals");
      std::cout<<"normals required!"<<std::endl;
    }
    vis3.spin ();
  }

  if (!write_normals)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    // Strip uninitialized normals from cloud:
    pcl::copyPointCloud (*cloud_1, *cloud_xyz);
    savePCDFileASCII (argv[pcd_file_indices[0]], *cloud_xyz);
  }
  else
  {
    savePCDFileASCII (argv[pcd_file_indices[0]], *cloud_1);
  }
}
