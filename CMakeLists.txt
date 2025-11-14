cmake_minimum_required(VERSION 4.0)
project(preint_imu)

set(CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

set(G2O_ROOT "/home/fan/ssd/study/slambook2/3rdparty/g2o/install")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
Find_Package(G2O REQUIRED)
message(STATUS "include: ${G2O_INCLUDE_DIR}")
message(STATUS "core: ${G2O_CORE_LIBRARY}")

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
)
FetchContent_MakeAvailable(googletest)

add_subdirectory(src)
add_subdirectory(test)