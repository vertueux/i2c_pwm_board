cmake_minimum_required(VERSION 3.8)
project(i2cpwm_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(i2cpwm_board_msgs REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(i2cpwm_executable src/i2cpwm_controller.cc)
target_link_libraries(i2cpwm_executable i2c)

ament_target_dependencies(i2cpwm_executable 
  rclcpp 
  std_msgs 
  std_srvs
  i2cpwm_board_msgs
  xmlrpcpp 
  geometry_msgs
)

install(TARGETS
  i2cpwm_executable 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
