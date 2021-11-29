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
              std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << std::endl;

    //print the first element of the tuple
    //std::cout << "Transformation from color term: ok? " << std::get<0>(trans_color_term_info) << std::endl;
    //std::cout << "Transformation from color term:" << std::endl << std::get<1>(trans_color_term_info) << std::endl;
    //std::cout << "INFORM from color term:" << std::endl << std::get<2>(trans_color_term_info) << std::endl;

    if(std::get<0>(trans_color_term_info))
    {
        //print Using RGB-D Odometry
        std::cout <<"Using RGB-D Odometry " <<std::endl;
        std::cout <<"Transformation: " << std::get<1>(trans_color_term_info)<< std::endl;
        //create point cloud from rgbd_image
        auto source_pcd_color_term = open3d::geometry::PointCloud::CreateFromRGBDImage(*source_rgbd, pinhole_camera_intrinsic);
        source_pcd_color_term->Transform(std::get<1>(trans_color_term_info));
        //draw pcl
        Eigen::Vector3d lookat;
        lookat << 0.0345, -0.0937, 1.8033;
        Eigen::Vector3d up;
        up << -0.0067, -0.9838, 0.1790;
        Eigen::Vector3d front;
        front << 0.0999, -0.1787, -0.9788;

        open3d::visualization::DrawGeometries({target_point_cloud, source_pcd_color_term},
                                              "Point Cloud", 640, 480, 50, 50, true, false, false,
                                              &lookat, &up, &front);
    }
    else
    {
        std::cout <<"Using RGB-D Odometry failed" <<std::endl;
    }


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
              std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1).count() << std::endl;

    //print the first element of the tuple
    std::cout << "Transformation from HYBRID term: ok? " << std::get<0>(trans_hybrid_term_info) << std::endl;
    std::cout << "Transformation from HYBRID term:" << std::endl << std::get<1>(trans_hybrid_term_info) << std::endl;
    std::cout << "INFORM from HYBRID term:" << std::endl << std::get<2>(trans_hybrid_term_info) << std::endl;

    if(std::get<0>(trans_hybrid_term_info))
    {
        //print Using RGB-D Odometry
        std::cout <<"Using RGB-D Hybrid_term " <<std::endl;
        std::cout <<"Transformation: " <<std::get<1>(trans_hybrid_term_info)<< std::endl;
        //create point cloud from rgbd_image
        auto source_pcd_hybrid_term = open3d::geometry::PointCloud::CreateFromRGBDImage(*source_rgbd, pinhole_camera_intrinsic);
        source_pcd_hybrid_term->Transform(std::get<1>(trans_hybrid_term_info));
        //draw pcl
        Eigen::Vector3d lookat;
        lookat << 0.0345, -0.0937, 1.8033;
        Eigen::Vector3d up;
        up << -0.0067, -0.9838, 0.1790;
        Eigen::Vector3d front;
        front << 0.0999, -0.1787, -0.9788;
        //colorize the pcl
        target_point_cloud->PaintUniformColor(Eigen::Vector3d(1.0, 0.506, 0.0));

        //source_pcd_hybrid_term->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.706));

        open3d::visualization::DrawGeometries({target_point_cloud, source_pcd_hybrid_term},
                                              "Point Cloud", 640, 480, 50, 50, false, false, false,
                                              &lookat, &up, &front);
    }
    else
    {
        std::cout <<"Using RGB-D Hybrid_term failed" <<std::endl;
    }


    // This code block calls two different RGBD odometry methods. The first one is from [Steinbrucker2011].
    // It minimizes photo consistency of aligned images. The second one is from [Park2017].
    // In addition to photo consistency, it implements constraint for geometry. Both functions run in similar speed,
    // but [Park2017] is more accurate in our test on benchmark datasets and is thus the recommended method.
    // Park, Q.-Y. Zhou, and V. Koltun, Colored Point Cloud Registration Revisited, ICCV, 2017.
    // Steinbrucker, J. Sturm, and D. Cremers, Real-time visual odometry from dense RGB-D images, In ICCV Workshops, 2011.




    return 0;

}
