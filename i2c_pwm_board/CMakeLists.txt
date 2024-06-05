cmake_minimum_required(VERSION 3.8)
project(i2c_pwm_board)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(i2c_pwm_board_msgs REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(i2c_pwm_board_xmlrp REQUIRED)

include_directories(include)

add_executable(node src/board.cc src/board_handler.cc src/board_controller.cc)
target_link_libraries(node i2c)

ament_target_dependencies(node rclcpp std_msgs std_srvs i2c_pwm_board_msgs xmlrpcpp geometry_msgs i2c_pwm_board_xmlrp)

install(TARGETS
        node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_package()
