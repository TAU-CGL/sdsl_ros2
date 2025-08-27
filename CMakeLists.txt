cmake_minimum_required(VERSION 3.13)
project(sdsl_ros2)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# OpenMP hotfix for Apple Clang
if(APPLE)
    set(CMAKE_C_COMPILER clang)
    set(CMAKE_CXX_COMPILER clang++)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY omp)
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY omp)
    endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(CGAL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(fmt REQUIRED)
find_package(glm REQUIRED)
find_package(Boost COMPONENTS container program_options REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/third-party/sdsl/sdsl/include)

# OpenMP hotfix for Apple Clang
if(APPLE)
    include_directories("/opt/homebrew/opt/libomp/include")
    link_directories("/opt/homebrew/opt/libomp/lib")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(sdsl_ros2 src/sdsl_ros2.cpp)
ament_target_dependencies(sdsl_ros2 rclcpp std_msgs nav_msgs sensor_msgs)
target_link_libraries(sdsl_ros2 CGAL::CGAL OpenMP::OpenMP_CXX fmt::fmt ${Boost_LIBRARIES})
install(TARGETS
  sdsl_ros2
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
