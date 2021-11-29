//
// Created by dinhnambkhn on 21. 11. 29..
//
#include <iostream>
#include <open3d/Open3D.h>

int main() {
    //camera intrinsic path
    std::string camera_intrinsic_path = "/home/dinhnambkhn/Open3D/examples/test_data/camera_primesense.json";
    //read_pinhole_camera_intrinsic
    open3d::camera::PinholeCameraIntrinsic pinhole_camera_intrinsic;
    open3d::io::ReadIJsonConvertibleFromJSON(camera_intrinsic_path, pinhole_camera_intrinsic);
    //check the intrinsic parameters
    std::cout << "Intrinsic: " << pinhole_camera_intrinsic.intrinsic_matrix_ << std::endl;

    //read source color image path
    std::string source_color_image_path = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/color/00000.jpg";
    //read source depth image path
    std::string source_depth_image_path = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/depth/00000.png";
    //read image from path
    open3d::geometry::Image color_image, depth_image;
    open3d::io::ReadImage(source_color_image_path, color_image);
    open3d::io::ReadImage(source_depth_image_path, depth_image);
    //check if the image is read or not
    std::cout << "Color image: " << color_image.width_ << " " << color_image.height_ << std::endl;
    std::cout << "Depth image: " << depth_image.width_ << " " << depth_image.height_ << std::endl;

    //create rgbd image from color and depth image
    auto source_rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(color_image, depth_image);

    //read target color image path
    std::string target_color_image_path = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/color/00001.jpg";
    //read target depth image path
    std::string target_depth_image_path = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/depth/00001.png";
    //read image from path
    open3d::geometry::Image target_color_image, target_depth_image;
    open3d::io::ReadImage(target_color_image_path, target_color_image);
    open3d::io::ReadImage(target_depth_image_path, target_depth_image);
    //check if the image is read or not
    std::cout << "Color image: " << target_color_image.width_ << " " << target_color_image.height_ << std::endl;
    std::cout << "Depth image: " << target_depth_image.width_ << " " << target_depth_image.height_ << std::endl;

    auto target_rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(target_color_image, target_depth_image);

    //create target point cloud from target rgbd image
    auto target_point_cloud = open3d::geometry::PointCloud::CreateFromRGBDImage(*target_rgbd, pinhole_camera_intrinsic);
    //check if the point cloud is empty or not
    std::cout << "Point cloud: " << target_point_cloud->points_.size() << std::endl;
    //create Odometry option
    //Compute odometry from two RGBD image pairs
    auto option = open3d::pipelines::odometry::OdometryOption();
    auto odom_init = Eigen::Matrix4d::Identity();
    //print option
    std::cout << "max_depth_: " << option.max_depth_ << std::endl;
    std::cout << "min_depth_: " << option.min_depth_ << std::endl;
    std::cout << "max_depth_diff_: " << option.max_depth_diff_ << std::endl;

    //begin counting time
    auto start = std::chrono::system_clock::now();

    //print option//create odometry from two RGBD image pairs
    auto trans_color_term_info = open3d::pipelines::odometry::ComputeRGBDOdometry(*source_rgbd,
                                                                                  *target_rgbd,
                                                                                  pinhole_camera_intrinsic,
                                                                                  odom_init,
                                                                                  open3d::pipelines::odometry::RGBDOdometryJacobianFromColorTerm(),
                                                                                  option);
    //stop counting time
    auto stop = std::chrono::system_clock::now();
    //print time
    std::cout << "time for Transformation from color ms: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() <<std::endl;

    //print the first element of the tuple
    std::cout << "Transformation from color term: ok? " << std::get<0>(trans_color_term_info) << std::endl;
    std::cout << "Transformation from color term:" << std::endl << std::get<1>(trans_color_term_info) << std::endl;
    std::cout << "INFORM from color term:" << std::endl << std::get<2>(trans_color_term_info) << std::endl;


    //begin counting time
    auto start1 = std::chrono::system_clock::now();



    auto trans_hybrid_term_info = open3d::pipelines::odometry::ComputeRGBDOdometry(*source_rgbd,
                                                                                   *target_rgbd,
                                                                                   pinhole_camera_intrinsic,
                                                                                   odom_init,
                                                                                   open3d::pipelines::odometry::RGBDOdometryJacobianFromColorTerm(),
                                                                                   option);

    //stop time counting
    auto stop1 = std::chrono::system_clock::now();
    //print time
    std::cout << "time for Transformation from color ms: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1).count() <<std::endl;

    //print the first element of the tuple
    std::cout << "Transformation from HYBRID term: ok? " << std::get<0>(trans_hybrid_term_info) << std::endl;
    std::cout << "Transformation from HYBRID term:" << std::endl<< std::get<1>(trans_hybrid_term_info) << std::endl;
    std::cout << "INFORM from HYBRID term:" << std::endl << std::get<2>(trans_hybrid_term_info) << std::endl;



    return 0;

}
