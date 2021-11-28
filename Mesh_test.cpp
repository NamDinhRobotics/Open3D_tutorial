//
// Created by dinhnambkhn on 21. 11. 28..
//
#include <iostream>

#include "open3d/Open3D.h"

int main(){
    //create a shape mesh
    auto mesh = open3d::geometry::TriangleMesh::CreateSphere(1.0, 32, 32);
    mesh->ComputeVertexNormals();
    mesh->ComputeTriangleNormals();
    // create a line mesh
    //auto line = open3d::geometry::LineSet::CreateFromTriangleMesh(*mesh);
    // create a point cloud
    //draw the mesh
    open3d::visualization::DrawGeometries({mesh});

    //create a box mesh
    auto box = open3d::geometry::TriangleMesh::CreateBox(0.5, 0.2, 0.8);
    box->ComputeVertexNormals();
    box->ComputeTriangleNormals();
    // draw the box
    open3d::visualization::DrawGeometries({box});

    return 0;

}
