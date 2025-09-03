# sdsl_ros2
ROS2 Node for the Sparse Distance Sampling Localization (SDSL) algorithm for Online Life-long indoor localization.

Notice that the `third-party` directory containes a git submodule (the `sdsl` C++ library).
To fetch it, after cloning this repository, run:

```
 git submodule update --init --recursive
```

Before building, make sure to install all dependencies:

1. CGAL (Version 6.0.1 at least!). Easiest way is to clone from [https://github.com/CGAL/cgal](https://github.com/CGAL/cgal), then running:
    ```
    cmake -B build
    cd build
    make
    sudo make install
    ```
2. glm (TODO: Check if it's strictly necessary): ```sudo apt-get install libglm-dev```
