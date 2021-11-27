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

  //create a voxel down sampling to pcd
  auto Vox_pcd = pcd.VoxelDownSample(0.05);
  //draw original point cloud
  //new window
  open3d::visualization::DrawGeometries({Vox_pcd}, "Voxel Down Sample");

  //estimate vertex normals
  //downpcd.estimate_normals(
  //    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
  //o3d.visualization.draw_geometries([downpcd],
  //                                  zoom=0.3412,
  //                                  front=[0.4257, -0.2125, -0.8795],
  //                                  lookat=[2.6172, 2.0475, 1.532],
  //                                  up=[-0.0694, -0.9768, 0.2024],
  //                                  point_show_normal=True)
  Vox_pcd->EstimateNormals(
      open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
  //draw the point cloud with normals
  open3d::visualization::DrawGeometries({Vox_pcd}, "Voxel Down Sample with Normals",
                                        640,640,50,50, true);




  /*
  //draw the point cloud
  auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
  sphere->ComputeVertexNormals();
  sphere->PaintUniformColor({0.0, 1.0, 0.0});
  open3d::visualization::DrawGeometries({sphere});
  */

  return 0;
}