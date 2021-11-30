cmake_minimum_required(VERSION 3.20)
project(test_open3D)

set(CMAKE_CXX_STANDARD 14)

#this is your install directory of open3D
find_package(Open3D HINTS /home/dinhnambkhn/open3d_install/lib/cmake)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

if (NOT Open3D_FOUND)
    message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
endif ()

add_executable(testPCL test_PCL.cpp)
target_link_libraries(testPCL Open3D::Open3D)

add_executable(read_image read_image.cpp)
target_link_libraries(read_image Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(mesh Mesh_test.cpp)
target_link_libraries(mesh Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(Transformation Transformation.cpp)
target_link_libraries(Transformation Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(rgbd_image rgbd_image.cpp)
target_link_libraries(rgbd_image Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(rgbd_odometry RGBD_odometry.cpp)
target_link_libraries(rgbd_odometry Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(Geometry_primitives Geometry_primitives.cpp)
target_link_libraries(Geometry_primitives Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(KDTree KDTree.cpp)
target_link_libraries(KDTree Open3D::Open3D ${OpenCV_LIBRARIES})

add_executable(ICP_Registration ICP_Registration.cpp)
target_link_libraries(ICP_Registration Open3D::Open3D ${OpenCV_LIBRARIES})