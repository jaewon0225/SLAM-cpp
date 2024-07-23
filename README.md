Pose Graph Optimisation SLAM Library in C++

Current file format is .graph but any text file that matches the format of the examples in the data directory will work.

This library requires Eigen3 library to be installed in your system. Refer to https://eigen.tuxfamily.org/index.php?title=Main_Page#Download for more details, on most platforms, you will be able to install the library with one of the following:

MacOS:
```
$ brew install eigen
```
Linux:
```
$ sudo apt install libeigen3-dev
```
You can then build the project by running the following in your project root:
```
$ mkdir -p build && cd build
```
```
$ cmake ..
```
```
$ make
```
Now you can import this library in your project as cpp_slam!

Import edges/vertices with PoseGraph::importEdges/Poses function and start optimising the pose graph with PoseGraph::optimise.
