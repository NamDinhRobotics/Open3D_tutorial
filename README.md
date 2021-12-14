# The Open3D tutorials in C++

I wrote this tutorial to help you get started with Open3D using C++.

The original tutorial is [here](http://www.open3d.org/docs/latest/tutorial). That was writen in Python.
## Installed Open3D Package in CMake
This is one of the two CMake examples showing how to use Open3D in your CMake
project:

* [Find Pre-Installed Open3D Package in CMake](https://github.com/intel-isl/open3d-cmake-find-package)
* [Use Open3D as a CMake External Project](https://github.com/intel-isl/open3d-cmake-external-project)
* For more details, check out the [Open3D repo](https://github.com/intel-isl/Open3D) and
  [Open3D docs](http://www.open3d.org/docs/release/cpp_project.html).

## Step 1: Compile and install Open3D
Follow the [Open3D compilation guide](http://www.open3d.org/docs/release/compilation.html), 
compile and install Open3D in your preferred location.
You can specify the installation path with `CMAKE_INSTALL_PREFIX` and the number of parallel jobs
to speed up compilation.

On Ubuntu/macOS:
```bash
git clone --recursive https://github.com/intel-isl/Open3D.git
cd Open3D
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=${HOME}/open3d_install ..
cmake --build . --config Release --parallel 12 --target install
cd ../..
```
Note: `-DBUILD_SHARED_LIBS=ON` is recommended if `-DBUILD_CUDA_MODULE=ON`.
If error:
please install dependencies
# Only needed for Ubuntu
util/install_deps_ubuntu.sh

and add to .bashrc file as,
export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
export PATH=$PATH:$CUDA_HOME/bin

In my case, I downloaded the source code to 
>/home/dinhnambkhn/Open3D

After install on my Ubuntu 18.04, I got the HOME folder is:
>/home/dinhnambkhn/Open3D_install


## Step 2: The topics are covered in this tutorial:
+ How to config cmake with open3D
+ PCL with open3D
+ Image
+ Transformation
+ RGB-D image
  ![Screenshot](./images/rgbd_img.png)

+ RGB-D odometry
  ![Screenshot](./images/RGBD_odom.png)

+ Geometry primitives
  ![Screenshot](./images/geometry.png)

+ KDTree
  ![Screenshot](./images/kdtree.png)

+ ICP registration
  ![Screenshot](./images/icp.png)

