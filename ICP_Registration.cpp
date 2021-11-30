//
// Created by dinhnambkhn on 21. 11. 30..
// http://www.open3d.org/docs/latest/tutorial/Basic/icp_registration.html
// This tutorial demonstrates the ICP (Iterative Closest Point) registration algorithm.
// It has been a mainstay of geometric registration in both research and industry for many years.
// The input are two point clouds and an initial transformation that roughly aligns the source point cloud to the target point cloud.
// The output is a refined transformation that tightly aligns the two point clouds.
// A helper function draw_registration_result visualizes the alignment during the registration process.
// In this tutorial, we show two ICP variants, the point-to-point ICP and the point-to-plane ICP [Rusinkiewicz2001]_.
// https://ieeexplore.ieee.org/document/924423

#include <iostream>
#include <open3d/Open3D.h>

//Helper visualization function
void draw_registration_result(
        const open3d::geometry::PointCloud &source, //shared_ptr<PointCloud>
        const open3d::geometry::PointCloud &target,
        const Eigen::Matrix4d &transformation) {
    open3d::geometry::PointCloud source_down, target_down;

    //std::shared_ptr<open3d::geometry::PointCloud> cloud_ptr(new open3d::geometry::PointCloud(cloud));
    //to make copies and protect the original point clouds.
    source_down = source;
    target_down = target;
    std::shared_ptr<open3d::geometry::PointCloud> source_temp(new open3d::geometry::PointCloud(source_down));
    std::shared_ptr<open3d::geometry::PointCloud> target_temp(new open3d::geometry::PointCloud(target_down));


    source_temp->PaintUniformColor(Eigen::Vector3d(1, 0.706, 0));
    target_temp->PaintUniformColor(Eigen::Vector3d(0, 0.651, 0.929));
    source_temp->Transform(transformation);

    double zoom = 0.4459;
    auto look_at = Eigen::Vector3d(1.6784, 2.0612, 1.4451);
    auto front = Eigen::Vector3d(0.9288, -0.2951, -0.2242);
    auto up = Eigen::Vector3d(-0.3402, -0.9189, -0.1996);

    open3d::visualization::DrawGeometries({source_temp, target_temp}, "ICP-PCL",
                                          640, 480, 50, 50,
                                          false, false, false,
                                          &look_at,
                                          &up,
                                          &front,
                                          &zoom);

}

int main() {

    //The code below reads a source point cloud and a target point cloud from two files. A rough transformation is given.
    //The initial alignment is usually obtained by a global registration algorithm. See Global registration for examples.
    //read the source point cloud
    open3d::geometry::PointCloud source;
    open3d::io::ReadPointCloud("/home/dinhnambkhn/Open3D/examples/test_data/ICP/cloud_bin_0.pcd", source);

    //read the target point cloud
    open3d::geometry::PointCloud target;
    open3d::io::ReadPointCloud("/home/dinhnambkhn/Open3D/examples/test_data/ICP/cloud_bin_1.pcd", target);
    //check the point cloud is loaded correctly
    std::cout << "Loaded source: " << source.points_.size() << "points and target data "
              << target.points_.size() << " points with " << std::endl;

    auto threshold = 0.02;
    //transformation init to [0.862, 0.011, -0.507, 0.5],
    //                         [-0.139, 0.967, -0.215, 0.7],
    //                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]
    auto trans_init = Eigen::Matrix4d({{0.862,  0.011, -0.507, 0.5},
                                       {-0.139, 0.967, -0.215, 0.7},
                                       {0.487,  0.255, 0.835,  -1.4},
                                       {0.0,    0.0,   0.0,    1.0}});
    //show trans_init matrix
    std::cout << "init matrix " << trans_init.matrix() << std::endl;
    //draw

    draw_registration_result(source, target, trans_init);


    //The function evaluate_registration calculates two main metrics:
    //fitness, which measures the overlapping area (# of inlier correspondences / # of points in target). The higher the better.
    //inlier_rmse, which measures the RMSE of all inlier correspondences. The lower the better.
    //Initial alignment
    std::cout << "Initial alignment: " << std::endl;
    //evaluate_registration
    auto evaluation = open3d::pipelines::registration::EvaluateRegistration(source, target, threshold, trans_init);

    //print evaluation
    std::cout << "RegistrationResult with fitness " << evaluation.fitness_ << std::endl;
    std::cout << "inline_rmse_ " << evaluation.inlier_rmse_ << std::endl;
    std::cout << "correspondence_set_ " << evaluation.correspondence_set_.size() << std::endl;

    //Point-to-point ICP
    //In general, the ICP algorithm iterates over two steps:
    //step 1: Find correspondence set K={(p,q)} from target point cloud P,
    // and source point cloud Q transformed with current transformation matrix T.
    //Step 2: Update the transformation T by minimizing an objective function E(T) defined over the correspondence set K.
    std::cout << "Apply point-to-point ICP" << std::endl;
    //point_to_point_registration
    //begin counting time
    auto start = std::chrono::system_clock::now();

    auto reg_p2p = open3d::pipelines::registration::RegistrationICP(source, target, threshold, trans_init,
                                                                    open3d::pipelines::registration::TransformationEstimationPointToPoint());

    //stop counting time
    auto stop = std::chrono::system_clock::now();
    //print time
    std::cout << "time for Transformation from color ms: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << std::endl;

    std::cout << "RegistrationResult with fitness " << reg_p2p.fitness_ << std::endl;
    std::cout << "inline_rmse_ " << reg_p2p.inlier_rmse_ << std::endl;
    std::cout << "correspondence_set_ " << reg_p2p.correspondence_set_.size() << std::endl;

    std::cout << "Transformation is: \n" << reg_p2p.transformation_ << std::endl;
    draw_registration_result(source, target, reg_p2p.transformation_);
    //The fitness score increases from 0.174723 to 0.372450.
    // The inlier_rmse reduces from 0.011771 to 0.007760.
    // By default, registration_icp runs until convergence or reaches a maximum number of iterations (30 by default).
    // It can be changed to allow more computation time and to improve the results further.
    //let change trans_init
    //trans_init = reg_p2p.transformation_;

    start = std::chrono::system_clock::now();
    reg_p2p = open3d::pipelines::registration::RegistrationICP(source, target, threshold, trans_init,
                                                               open3d::pipelines::registration::TransformationEstimationPointToPoint(),
                                                               open3d::pipelines::registration::ICPConvergenceCriteria(
                                                                       1e-6, 1e-6, 1000));

    stop = std::chrono::system_clock::now();
    //print time
    std::cout << "with 1000 integrations, time for Transformation from color ms: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << std::endl;

    std::cout << "RegistrationResult with fitness " << reg_p2p.fitness_ << std::endl;
    std::cout << "inline_rmse_ " << reg_p2p.inlier_rmse_ << std::endl;
    std::cout << "correspondence_set_ " << reg_p2p.correspondence_set_.size() << std::endl;
    //show the interation number

    //The final alignment is tight. The fitness score improves to 0.621123. The inlier_rmse reduces to 0.006583.

    std::cout << "Transformation is: \n" << reg_p2p.transformation_ << std::endl;
    draw_registration_result(source, target, reg_p2p.transformation_);


    //Point-to-plane ICP
    //The point-to-plane ICP algorithm iterates using normal of point p
    // Apply point-to-plane ICP
    std::cout << "Apply point-to-plane ICP" << std::endl;
    //begin counting time
    start = std::chrono::system_clock::now();
    auto reg_p2l = open3d::pipelines::registration::RegistrationICP(source, target, threshold, trans_init,
                                                                    open3d::pipelines::registration::TransformationEstimationPointToPlane());
    stop = std::chrono::system_clock::now();
    //print time
    std::cout << "PointToPlane time for Transformation from color ms: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << std::endl;

    std::cout << "RegistrationResult with fitness " << reg_p2l.fitness_ << std::endl;
    std::cout << "inline_rmse_ " << reg_p2l.inlier_rmse_ << std::endl;
    std::cout << "correspondence_set_ " << reg_p2l.correspondence_set_.size() << std::endl;
    //The final alignment is tight. The fitness score improves to 0.621123. The inlier_rmse reduces to 0.006583.
    //The point-to-plane ICP reaches tight alignment within 30 iterations
    // (a fitness score of 0.620972 and an inlier_rmse score of 0.006581).

    std::cout << "Transformation is: \n" << reg_p2l.transformation_ << std::endl;
    draw_registration_result(source, target, reg_p2l.transformation_);


    return 0;
}

