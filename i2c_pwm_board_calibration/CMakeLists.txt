cmake_minimum_required(VERSION 3.8)
project(i2c_pwm_board_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(i2c_pwm_board_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(node src/calibration_node.cc)

ament_target_dependencies(node i2c_pwm_board_msgs rclcpp std_srvs std_msgs)

install(TARGETS
  node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
