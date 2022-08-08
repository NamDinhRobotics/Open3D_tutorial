//
// Created by dinhnambkhn on 21. 11. 28..
//
#include <iostream>
#include "open3d/Open3D.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>


int main() {
    //read image file
    open3d::geometry::Image image;
    open3d::io::ReadImage("/home/dinhnambkhn/Open3D/examples/test_data/lena_color.jpg", image);
    //check image is empty or not
    if (image.IsEmpty()) {
        std::cout << "Image is empty" << std::endl;
    } else {
        std::cout << "Image is not empty" << std::endl;
    }

    //create a share d pointer of image
    std::shared_ptr<open3d::geometry::Image> image_ptr = std::make_shared<open3d::geometry::Image>(image);
    //draw the image_ptr
    open3d::visualization::DrawGeometries({image_ptr});

    return 0;

}



