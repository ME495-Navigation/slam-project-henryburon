cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)



add_executable(turtle_control 
              src/turtle_control.cpp)

add_executable(odometry 
               src/odometry.cpp)

ament_target_dependencies(turtle_control
  rclcpp
  geometry_msgs
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  )

ament_target_dependencies(odometry
  rclcpp)


target_include_directories(turtle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(turtle_control turtlelib::turtlelib)
target_link_libraries(odometry turtlelib::turtlelib)



# target_compile_features(turtle_control PUBLIC c_std_99 cxx_std_17)

install(DIRECTORY
  launch
  DESTINATION share/nuturtle_control/
)

install(TARGETS 
  turtle_control
  odometry
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()