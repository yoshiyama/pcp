//
// Software License Agreement (BSD License)

// Point Cloud Library (PCL) - www.pointclouds.org
// Copyright (c) 2009-2012, Willow Garage, Inc.
// Copyright (c) 2012-, Open Perception, Inc.
// Copyright (c) XXX, respective authors.

// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
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
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{

    	if (argc != 6)
    {
        std::cout << "Error!\n **.exe input.pcd output.pcd leaf_size_x[m] leaf_size_y[m] leaf_size_z[m] \n";
            return 0;
    }
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
  pcl::PCLPointCloud2::Ptr cloud_org (new pcl::PCLPointCloud2 ());// Original point cloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud_move (new pcl::PCLPointCloud2 ());// Transformed point cloud
  pcl::PCLPointCloud2::Ptr cloud_icp (new pcl::PCLPointCloud2 ());// ICP output point cloud

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  // reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
  reader.read (argv[1], *cloud_org); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud_org->width * cloud_org->height 
       << " data points (" << pcl::getFieldsList (*cloud_org) << ")." << std::endl;

  // // Fill in the CloudIn data
  // for (auto& point : *cloud_in)
  // {
  //   point.x = 1024 * rand() / (RAND_MAX + 1.0f);
  //   point.y = 1024 * rand() / (RAND_MAX + 1.0f);
  //   point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  // }
  
  std::cout << "Saved " << cloud_org->size () << " data points to input:" << std::endl;
      
  // for (auto& point : *cloud_in)
  //   std::cout << point << std::endl;
      
  // *cloud_out = *cloud_in;
  
  // std::cout << "size:" << cloud_out->size() << std::endl;
  // for (auto& point : *cloud_out)
  //   point.x += 0.7f;

  // std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
  // for (auto& point : *cloud_out)
  //   std::cout << point << std::endl;

  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;//icpの実体
  icp.setInputSource(cloud_org);//original
  icp.setInputTarget(cloud_move);//transformed
  
  // pcl::PointCloud<pcl::PointXYZ> Final;->for output
  pcl::PointCloud<pcl::PointXYZRGB> Final;
  // Perform the alignment
  
// Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
// Parameters
//     [out]	output	the resultant input transformed point cloud dataset
// Definition at line 155 of file registration.hpp.
  icp.align(Final);
  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  Eigen::Matrix4f transformation = icp.getFinalTransformation ();

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  //show transformation_matrix
  std::cout << icp.getFinalTransformation() << std::endl;

 return (0);
}
