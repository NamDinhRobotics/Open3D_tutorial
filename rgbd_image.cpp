//
// Created by dinhnambkhn on 21. 11. 28..
//
#include <iostream>
#include <open3d/Open3D.h>

int main(){
    //read color image
    std::string color_path = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/color/00000.jpg";
    std::string depth_path = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/depth/00000.png";
    //create image variable open3d
    open3d::geometry::Image color_image;
    open3d::geometry::Image depth_image;

    open3d::io::ReadImage(color_path, color_image);
    open3d::io::ReadImage(depth_path, depth_image);

    //check image empty or not
    if(color_image.IsEmpty() || depth_image.IsEmpty()){
        std::cout << "Image is empty" << std::endl;
        return -1;
    } else
        std::cout << "Image is not empty" << std::endl;


    //create rgbd image from color and depth image
    auto rgbd_img = open3d::geometry::RGBDImage::CreateFromColorAndDepth(color_image, depth_image);
    //check rgbd image empty or not
    if(rgbd_img->IsEmpty()){
        std::cout << "RGBD image is empty" << std::endl;
        return -1;
    } else
        std::cout << "RGBD image is not empty" << std::endl;

    //print rgbd image
    std::cout << rgbd_img << std::endl;
    //draw rgbd image
    open3d::visualization::DrawGeometries({rgbd_img});
    // create point cloud from rgbd image with default parameters
    auto point_cloud=open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_img,open3d::camera::PinholeCameraIntrinsic(open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault));
    //check point cloud empty or not
    if(point_cloud->IsEmpty()){
        std::cout << "Point cloud is empty" << std::endl;
        return -1;
    } else
        std::cout << "Point cloud is not empty" << std::endl;

    //

    //create a eigen matrix 4x4
    Eigen::Matrix4d transformation_matrix;
    //set transformation matrix
    transformation_matrix << 1, 0, 0, 0
                           , 0, -1, 0, 0
                           , 0, 0, -1, 0
                           , 0, 0, 0, 1;

    point_cloud->Transform(transformation_matrix);

    //draw point cloud with zoom property 0.5
    open3d::visualization::DrawGeometries({point_cloud});

    //read color image
    std::string color_path2 = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/other_formats/SUN_color.jpg";
    std::string depth_path2 = "/home/dinhnambkhn/Open3D/examples/test_data/RGBD/other_formats/SUN_depth.png";
    //create image variable open3d
    open3d::geometry::Image color_image2;
    open3d::geometry::Image depth_image2;
    //read image
    open3d::io::ReadImage(color_path2, color_image2);
    open3d::io::ReadImage(depth_path2, depth_image2);

    //shown the color image
    //create rgbd image from color and depth image
    auto rgbd_img2 = open3d::geometry::RGBDImage::CreateFromSUNFormat(color_image2, depth_image2);
    //check rgbd image empty or not
    if(rgbd_img2->IsEmpty()){
        std::cout << "RGBD SUN image is empty" << std::endl;
        return -1;
    } else
        std::cout << "RGBD SUN image is not empty" << std::endl;

    //draw the rgbd image
    open3d::visualization::DrawGeometries({rgbd_img2});

    //create point cloud from rgbd image with default parameters
    auto point_cloud2=open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_img2,open3d::camera::PinholeCameraIntrinsic(open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault));
    //check point cloud empty or not
    if(point_cloud2->IsEmpty()){
        std::cout << "Point cloud SUN is empty" << std::endl;
        return -1;
    } else
        std::cout << "Point cloud SUN is not empty" << std::endl;


    Eigen::Matrix4d transformation_matrix2;
    transformation_matrix2 << 1, 0, 0, 0
            , 0, -1, 0, 0
            , 0, 0, -1, 0
            , 0, 0, 0, 1;

    point_cloud2->Transform(transformation_matrix2);

    //draw point cloud with zoom property 0.5
    open3d::visualization::DrawGeometries({point_cloud2});

    //filter the point cloud with voxel grid filter
    auto voxel_grid_filter = point_cloud2->VoxelDownSample(0.1);
    //draw the point cloud
    open3d::visualization::DrawGeometries({voxel_grid_filter});

    return 0;
}
