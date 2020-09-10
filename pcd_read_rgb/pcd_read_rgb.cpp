// Software License Agreement (BSD License)

// Point Cloud Library (PCL) - www.pointclouds.org
// Copyright (c) 2009-2012, Willow Garage, Inc.
// Copyright (c) 2012-, Open Perception, Inc.
// Copyright (c) XXX, respective authors.

// All rights reserved.

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>

int
main (int argc, char** argv)
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
  //fromPCLPointCloud2 
  std::cout << "1" << std::endl;

  pcl::PCDReader reader;
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  reader.read (argv[1], *cloud2); // Remember to download the file first!
  //fromPCLPointCloud2 
  std::cout << "2" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  //fromPCLPointCloud2 
  std::cout << "3" << std::endl;
 
  pcl::fromPCLPointCloud2(*cloud2,*cloud);
  // if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  // {
  //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
  //   return (-1);
  // }
  std::cout << "4" << std::endl;
 
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    // uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
    uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8)  & 0x0000ff;
    uint8_t b = (rgb)       & 0x0000ff;
    // uint8_t r = (rgb >> 16) & 0x0000ff;
    // uint8_t g = (rgb >> 8)  & 0x0000ff;
    // uint8_t b = (rgb)       & 0x0000ff;
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z 
              << std::endl;
  }

  return (0);
}