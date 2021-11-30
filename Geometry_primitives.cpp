//
// Created by dinhnambkhn on 21. 11. 30..
// http://www.open3d.org/docs/latest/tutorial/Basic/visualization.html
//        Visualization
//        Function draw_geometries
//
#include <iostream>
#include <open3d/Open3D.h>
#include <memory>

int main() {
    //print let's define some primitives
    std::cout << "Let's define some primitives" << std::endl;
    //create a TriangleMesh create a box
    auto mesh_box = open3d::geometry::TriangleMesh::CreateBox(1.0, 1.0, 1.0);
    //create a weak pointer to mesh_box
    const std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_box_ptr = mesh_box;

    mesh_box->ComputeVertexNormals();
    mesh_box->PaintUniformColor(Eigen::Vector3d(0.9, 0.1, 0.1));
    //create a sphere
    auto mesh_sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    mesh_sphere->ComputeVertexNormals();
    mesh_sphere->PaintUniformColor(Eigen::Vector3d(0.1, 0.1, 0.7));
    //create a cylinder
    auto mesh_cylinder = open3d::geometry::TriangleMesh::CreateCylinder(0.3, 4.0);
    mesh_cylinder->ComputeVertexNormals();
    mesh_cylinder->PaintUniformColor(Eigen::Vector3d(0.1, 0.9, 0.7));
    //create a mesh frame coordinate system
    auto mesh_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.6, Eigen::Vector3d(-2, -2, -2));

    //draw_geometries takes a list of geometries and renders them all together.
    //Alternatively, TriangleMesh supports a + operator to combine multiple meshes into one.
    //We recommend the first approach since it supports a combination of different geometries
    // (e.g., a mesh can be rendered in tandem with a point cloud).
    //add all mesh using + operator

    //show "We draw a few primitives using collection."
    std::cout << "We draw a few primitives using collection." << std::endl;
    open3d::visualization::DrawGeometries({mesh_box_ptr, mesh_sphere, mesh_cylinder, mesh_frame});

    //show "We draw a few primitives using + operator of mesh." using + operator
    std::cout << "We draw a few primitives using + operator of mesh." << std::endl;
    auto box_total = mesh_box->operator+(*mesh_sphere).operator+(*mesh_cylinder).operator+(*mesh_frame);

    //need to create make_shared to avoid memory leak
    std::shared_ptr<open3d::geometry::TriangleMesh> box_total_ptr = std::make_shared<open3d::geometry::TriangleMesh>(
            box_total);
    open3d::visualization::DrawGeometries({box_total_ptr});

    //let draw a line set
    std::cout << "Let's draw a box using o3d.geometry.LineSet. " << std::endl;
    //create a vector of eigen 3d
    std::vector<Eigen::Vector3d> pointsets;
    //add points to pointsets in aline code
    pointsets.emplace_back(0, 0, 0);
    pointsets.emplace_back(1, 0, 0);
    pointsets.emplace_back(0, 1, 0);
    pointsets.emplace_back(1, 1, 0);
    pointsets.emplace_back(0, 0, 1);
    pointsets.emplace_back(1, 0, 1);
    pointsets.emplace_back(0, 1, 1);
    pointsets.emplace_back(1, 1, 1);

    //create a vector of eigen2i
    std::vector<Eigen::Vector2i> linesets;
    //add lines to linesets in aline code
    linesets.emplace_back(0, 1);
    linesets.emplace_back(0, 2);
    linesets.emplace_back(1, 3);
    linesets.emplace_back(2, 3);
    linesets.emplace_back(4, 5);
    linesets.emplace_back(4, 6);
    linesets.emplace_back(5, 7);
    linesets.emplace_back(6, 7);
    linesets.emplace_back(0, 4);
    linesets.emplace_back(1, 5);
    linesets.emplace_back(2, 6);
    linesets.emplace_back(3, 7);

    //LineSet(const std::vector<Eigen::Vector3d> &points,
    //const std::vector<Eigen::Vector2i> &lines)
    auto line_set = open3d::geometry::LineSet(pointsets, linesets);

    //create a vector of eigen 3d for colors, size of vector length of line_set
    std::vector<Eigen::Vector3d> colors;
    //get the length of line_set
    auto length = line_set.lines_.size();

    for (int i = 0; i < length; i++) {
        colors.emplace_back(Eigen::Vector3d(1, 0, 0));
    }

    //set the colors of line_set
    line_set.colors_ = colors;

    //std::make_shared<open3d::geometry::LineSet>
    std::shared_ptr<open3d::geometry::LineSet> line_set_ptr = std::make_shared<open3d::geometry::LineSet>(line_set);

    //draw line_set
    double zoom = 0.8;
    //auto *zoom = &zoomv;
    // create *zoom=0.8

    open3d::visualization::DrawGeometries({line_set_ptr}, "LineSet", 640, 480, 50, 50, false, false, false, nullptr,
                                          nullptr, nullptr, &zoom);
    //default

    return 0;
}
