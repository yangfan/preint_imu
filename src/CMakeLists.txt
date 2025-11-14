add_executable(main_gins main_gins.cpp)
target_include_directories(main_gins PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(main_gins PRIVATE utils glog::glog ${GFLAGS_LIBRARIES})

add_executable(main_eskf estimate_eskf.cpp)
target_include_directories(main_eskf PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(main_eskf PRIVATE utils glog::glog ${GFLAGS_LIBRARIES})

add_subdirectory(tools)