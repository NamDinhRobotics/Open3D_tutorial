cmake_minimum_required(VERSION 3.20)
project(test_open3D)

set(CMAKE_CXX_STANDARD 14)

find_package(Open3D HINTS /home/dinhnambkhn/open3d_install/lib/cmake)


if(NOT Open3D_FOUND)
    message(FATAL_ERROR "Open3D not found, please use -DCMAKE_INSTALL_PREFIX=open3d_install_dir")
endif()

add_executable(testopen3D main.cpp)
target_link_libraries(testopen3D Open3D::Open3D)