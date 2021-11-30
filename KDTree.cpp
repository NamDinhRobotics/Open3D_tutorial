//
// Created by dinhnambkhn on 21. 11. 30..
//
#include <iostream>
#include <open3d/Open3D.h>

int main(){

    //Testing kdtree in Open3D
    std::cout << "Testing kdtree in Open3D" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "Load a point cloud and paint it gray" << std::endl;
    //load point cloud
    open3d::geometry::PointCloud cloud;
    open3d::io::ReadPointCloud("/home/dinhnambkhn/Open3D/examples/test_data/Feature/cloud_bin_0.pcd", cloud);
    //check cloud is empty or not
    if(cloud.points_.empty()){
        std::cout << "Point cloud is empty" << std::endl;
    }else
        std::cout << "Point cloud is not empty" << std::endl;

    cloud.PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    //create KDTreeFlann
    open3d::geometry::KDTreeFlann pcd_tree(cloud);
    //open3d::geometry::KDTreeFlann pcd_tree;
    //pcd_tree.SetGeometry(cloud);
    //We pick the 1500th point as the anchor point and paint it red.
    std::cout << "Paint the anchor point red" << std::endl;
    cloud.colors_[1500] = Eigen::Vector3d(1, 0, 0);

    //Using search_knn_vector_3d
    //The function search_knn_vector_3d returns a list of indices of the k nearest neighbors of the anchor point.
    // These neighboring points are painted with blue color.
    std::cout << "Find its 200 nearest neighbors, and paint them blue" << std::endl;
    //template <typename T>
    //    int SearchKNN(const T &query,
    //                  int knn,
    //                  std::vector<int> &indices,
    //                  std::vector<double> &distance2) const;
    std::vector<int> indices;
    std::vector<double> distance2;

    pcd_tree.SearchKNN(cloud.points_[1500], 200, indices, distance2);
    for(int idx : indices){
        cloud.colors_[idx] = Eigen::Vector3d(0, 0, 1);
    }
    //Using search_radius_vector_3d
    //Similarly, we can use search_radius_vector_3d to query all points with distances to the anchor point less than a given radius.
    // We paint these points with a green color.

    return 0;
}
