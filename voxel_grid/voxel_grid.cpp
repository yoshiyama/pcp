//
// Software License Agreement (BSD License)

// Point Cloud Library (PCL) - www.pointclouds.org
// Copyright (c) 2009-2012, Willow Garage, Inc.
// Copyright (c) 2012-, Open Perception, Inc.
// Copyright (c) XXX, respective authors.

// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted providethat the following conditions
// are met: 

//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder(s) nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////
//
//
// voxel_grid inputf outputf voxel_leaf_size_x[m] voxel_leaf_y voxel_leaf_z
//
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  	if (argc != 6)
    {
        std::cout << "Error!\n **.exe input.pcd output.pcd leaf_size_x[m] leaf_size_y[m] leaf_size_z[m] \n";
            return 0;
    }

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  // reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
  reader.read (argv[1], *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//voxelgridのデータ型．実体．
  sor.setInputCloud (cloud);

  //Set the voxel grid leaf size. 
  float l_x=atof(argv[3]);
  float l_y=atof(argv[4]);
  float l_z=atof(argv[5]);
  
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.setLeafSize (l_x, l_y, l_z);
  //Calls the filtering method and returns the filtered dataset in output. 
  sor.filter (*cloud_filtered);

//Get the available point cloud fields as a space separated string. 
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

//   pcl::io::savePCDFileASCII (argv[1], cloud);
//   std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

//   pcl::PCDWriter writer;
//   writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
//          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  pcl::PCDWriter writer;
  writer.write (argv[2], *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
