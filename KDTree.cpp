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

    //make shared point cloud
    std::shared_ptr<open3d::geometry::PointCloud> cloud_ptr(new open3d::geometry::PointCloud(cloud));

    cloud_ptr->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    //create KDTreeFlann
    open3d::geometry::KDTreeFlann pcd_tree(*cloud_ptr);
    //open3d::geometry::KDTreeFlann pcd_tree;
    //pcd_tree.SetGeometry(cloud);
    const int red_idx = 1000;

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

    pcd_tree.SearchKNN(cloud_ptr->points_[red_idx], 200, indices, distance2);
    for(int idx : indices){
        cloud_ptr->colors_[idx] = Eigen::Vector3d(0, 0, 1);
    }
    //Using search_radius_vector_3d
    //Similarly, we can use search_radius_vector_3d to query all points with distances
    // to the anchor point less than a given radius.
    // We paint these points with a green color.
    std::cout << "Find its neighbors with distance less than 0.2, and paint them green" << std::endl;

    std::vector<int> indicesR;
    std::vector<double> distanceR;
    pcd_tree.SearchRadius(cloud_ptr->points_[red_idx], 0.2, indicesR, distanceR);
    //show the number of neighbors
    std::cout << "Number of neighbors: " << indicesR.size() << std::endl;

    //paint them green.
    for(int idx : indicesR){
        cloud_ptr->colors_[idx] = Eigen::Vector3d(0, 1, 0);
    }

    //We pick the 1500th point as the anchor point and paint it red.
    std::cout << "Paint the anchor point red" << std::endl;
    cloud_ptr->colors_[red_idx] = Eigen::Vector3d(1, 0, 0);


    //now, let visualize the point cloud
    //draw line_set
    double zoom = 0.5599;
    auto lookat = Eigen::Vector3d (2.1126, 1.0163, -1.8543);
    auto front = Eigen::Vector3d (-0.4958, 0.8229, 0.2773);
    auto up = Eigen::Vector3d (0.1007, -0.2626, 0.9596);

    open3d::visualization::DrawGeometries({cloud_ptr}, "LineSet",
                                          640, 480, 50, 50,
                                          false, false, false,
                                          &lookat,
                                          &up,
                                          &front,
                                          &zoom);


    return 0;
}
