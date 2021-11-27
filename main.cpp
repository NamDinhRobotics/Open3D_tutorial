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
  Vox_pcd->PaintUniformColor(Eigen::Vector3d(1, 0.706, 0));
  //cluser the point cloud
  auto cluster_pcd = Vox_pcd->ClusterDBSCAN(0.05, 2, true);

  //plane segmentation, return a tuple of two point clouds
  auto model = Vox_pcd->SegmentPlane(0.01, 3, 1000);
  //get the first element of the tuple
  auto model_pcd = std::get<0>(model);
  //get the second element of the tuple
  auto inlier_pcd = std::get<1>(model);
  //print model_pcd coffecients
  std::cout << "Model coefficients: " << model_pcd.transpose()<< std::endl;


  //draw the share of cluster_pcd




  //draw original point cloud
  //new window
  open3d::visualization::DrawGeometries({Vox_pcd}, "Voxel Down Sample");

  //print the size of the voxel down sampled point cloud
  std::cout << "Voxel Down Sample contains " << Vox_pcd->points_.size()
            << " points." << std::endl;

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

  //print the number of estimated normals
  std::cout << "Point cloud contains " << Vox_pcd->normals_.size()
            << " normals." << std::endl;

  //print the first 10 normals vectors
  for (int i = 0; i < 10; i++) {
    std::cout << Vox_pcd->normals_[i].transpose() << std::endl;
  }

  /*
  //draw the point cloud
  auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
  sphere->ComputeVertexNormals();
  sphere->PaintUniformColor({0.0, 1.0, 0.0});
  open3d::visualization::DrawGeometries({sphere});
  */

  return 0;
}