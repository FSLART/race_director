cmake_minimum_required(VERSION 3.8)
project(race_director)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(race_director_lib SHARED
  src/race_director.cpp
)
ament_target_dependencies(race_director_lib rclcpp std_msgs lart_msgs std_srvs)
target_include_directories(race_director_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(race_director_lib PUBLIC c_std_99 cxx_std_17)

add_executable(race_director
  src/race_director.cpp
)
target_link_libraries(race_director race_director_lib)
ament_target_dependencies(race_director rclcpp std_msgs lart_msgs std_srvs)

install(TARGETS race_director race_director_lib
  DESTINATION lib/${PROJECT_NAME})

# Update the test target to link to the library
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(race_director_test test/race_director_test.cpp)
  if(TARGET race_director_test)
    ament_target_dependencies(race_director_test rclcpp lart_msgs std_srvs)
    target_link_libraries(race_director_test race_director_lib)
    target_include_directories(race_director_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()
endif()

ament_package()