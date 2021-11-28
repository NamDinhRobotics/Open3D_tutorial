//
// Created by dinhnambkhn on 21. 11. 28..
//
#include <iostream>
#include <open3d/Open3D.h>

int main(){

    //point cloud path
    std::string path = "/home/dinhnambkhn/Open3D/examples/test_data/fragment.ply";

    //read point cloud
    open3d::geometry::PointCloud pcd;
    open3d::io::ReadPointCloud(path, pcd);

    //makeshared prt to the pcd
    std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(pcd);

    //print number of points
    std::cout << "Number of points: " << pcd.points_.size() << std::endl;
    //draw point cloud
    open3d::visualization::DrawGeometries({pcd_ptr}, "Point Cloud");

}

