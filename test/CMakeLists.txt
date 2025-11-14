enable_testing()

add_executable(test_preint test_preint.cpp)

# target_link_libraries(test_preint GTest::gtest_main GTest::gmock utils)
target_link_libraries(test_preint utils gtest GTest::gmock)
target_include_directories(test_preint PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src/tools/optimizer/g2o
)

include(GoogleTest)
gtest_discover_tests(test_preint)