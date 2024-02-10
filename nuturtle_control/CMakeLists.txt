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
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(catch_ros2 REQUIRED)


add_executable(turtle_control 
              src/turtle_control.cpp)

add_executable(odometry 
               src/odometry.cpp)

add_executable(circle
               src/circle.cpp)

add_executable(turtle_control_test
               src/turtle_control_test.cpp)

ament_target_dependencies(turtle_control
  rclcpp
  geometry_msgs
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  )

ament_target_dependencies(odometry
  rclcpp
  sensor_msgs
  nav_msgs
  turtlelib
  tf2
  tf2_ros
  )

ament_target_dependencies(circle
  rclcpp
  geometry_msgs
  std_srvs
  turtlelib
  )

ament_target_dependencies(turtle_control_test
  rclcpp
  geometry_msgs
  nuturtlebot_msgs
  sensor_msgs
  turtlelib
  catch_ros2
  )


########## Begin_Citation [10] ##########

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/InitialPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME})

target_include_directories(turtle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

########## End_Citation [10] ##########

target_link_libraries(turtle_control turtlelib::turtlelib)
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(turtle_control_test turtlelib::turtlelib)


install(DIRECTORY
  launch
  srv
  config
  test
  DESTINATION share/nuturtle_control/
)

install(TARGETS 
  turtle_control
  odometry
  circle
  turtle_control_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ########## Begin_Citation [13] ##########
  ########## End_Citation [13] ##########

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
