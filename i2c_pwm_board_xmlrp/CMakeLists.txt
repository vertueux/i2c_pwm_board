cmake_minimum_required(VERSION 3.8)
project(i2c_pwm_board_xmlrp)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xmlrpcpp REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
        xmlrpcpp
)

add_library(i2c_pwm_board_xmlrp_lib src/xmlrp.cc)
ament_target_dependencies(i2c_pwm_board_xmlrp_lib ${dependencies})

install(TARGETS i2c_pwm_board_xmlrp_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(i2c_pwm_board_xmlrp_lib)
ament_export_dependencies(${dependencies})

ament_package()
