#include <iostream>
#include <memory>
#include <thread>

#include "open3d/Open3D.h"
//include windows open3d
#include "open3d/utility/Console.h"


// A simplified version of examples/cpp/Visualizer.cpp to demonstrate linking
// an external project to Open3D.
int main() {
  //create a ptr variable to hold the point cloud
  open3d::geometry::PointCloud pcd;
  //read the point cloud fragment.pcd
  open3d::io::ReadPointCloud("fragment.pcd", pcd);



  //show the size of the point cloud
  std::cout << "Point cloud contains " << pcd.points_.size() << " points."
            << std::endl;

  //create a ptr to the pcd
  std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr =
      std::make_shared<open3d::geometry::PointCloud>(pcd);

  //draw the point cloud
  open3d::visualization::DrawGeometries({pcd_ptr});

  /*
  //draw the point cloud
  auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
  sphere->ComputeVertexNormals();
  sphere->PaintUniformColor({0.0, 1.0, 0.0});
  open3d::visualization::DrawGeometries({sphere});
  */

  return 0;
}