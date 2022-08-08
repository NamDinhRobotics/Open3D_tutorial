//
// Created by dinhnambkhn on 21. 11. 28..
//
#include <iostream>
#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>

int main() {
    //create a coordinate frame
    auto mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame();

    //open3d::geometry::MeshBase mesh_tx = mesh->Translate(Eigen::Vector3d(1.3, 0, 0), false);
    //open3d::geometry::MeshBase mesh_ty = mesh->Translate(Eigen::Vector3d(0, 1.3, 0), false);
    //auto mesh_tx = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1, Eigen::Vector3d (1.3, 0, 0));
    //auto mesh_ty = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1, Eigen::Vector3d (0, 1.3, 0));

    //print center of mesh
    std::cout << "center of mesh: " << mesh->GetCenter() << std::endl;

    //create shared pointer
    //move mesh coordinate frame
    //create a transformation eigen matrix
    Eigen::Matrix4d transformation;
    //set identity matrix
    transformation.setIdentity();
    //rotate around x-axis
    transformation(0, 0) = 1;
    transformation(1, 1) = cos(M_PI / 4);
    transformation(1, 2) = -sin(M_PI / 4);
    transformation(2, 1) = sin(M_PI / 4);
    transformation(2, 2) = cos(M_PI / 4);
    //translate
    transformation(0, 3) = 1.3;
    transformation(1, 3) = 1.3;
    //apply transformation
    auto mesh_tx = mesh->Transform(transformation);
    //create pointer to mesh

    //rotation matrix from rpy
    //make share d pointer to mesh_tx
    auto mesh_tx_ptr = std::make_shared<open3d::geometry::TriangleMesh>(mesh_tx);
    //make share mesh_scale
    //print center of mesh after translation
    std::cout << "center of mesh_tx after translation: " << mesh_tx.GetCenter() << std::endl;
    //draw mesh
    open3d::visualization::DrawGeometries({mesh, mesh_tx_ptr});

    return 0;
}

