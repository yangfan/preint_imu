
add_library(utils imu.cpp gnss.cpp initializer.cpp processor.cpp gins.cpp eskf.cpp
    ${PROJECT_SOURCE_DIR}/3rdparty/GeoConvert/tranmerc.cc
    ${PROJECT_SOURCE_DIR}/3rdparty/GeoConvert/utm.cc
)
target_include_directories(utils PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/3rdparty/GeoConvert
)
target_link_libraries(utils Eigen3::Eigen Sophus::Sophus glog::glog g2o_types)

add_subdirectory(optimizer/g2o)