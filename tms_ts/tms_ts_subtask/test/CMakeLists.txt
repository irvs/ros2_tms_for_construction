find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_zx120_sample_boom_subtask test_zx120_sample_boom_subtask.cpp)
target_link_libraries(test_zx120_sample_boom_subtask ${PROJECT_NAME})

target_include_directories(test_zx120_sample_boom_subtask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi)

target_link_directories(test_zx120_sample_boom_subtask PUBLIC
  /usr/local/lib)

target_link_libraries(test_zx120_sample_boom_subtask
mongocxx
bsoncxx)