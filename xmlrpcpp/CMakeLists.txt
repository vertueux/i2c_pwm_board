cmake_minimum_required(VERSION 3.5)
project(xmlrpcpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)

include_directories(
  include
  libb64/include
  ${ament_INCLUDE_DIRS}
)

link_directories(${ament_LIBRARY_DIRS})

add_library(xmlrpcpp
  src/xml_rpc_client.cc
	#TODO(bpwilcox):port for ROS2 
  #src/xml_rpc_Dispatch.cc  
  src/xml_rpc_server.cc
  src/xml_rpc_server_connection.cc
  src/xml_rpc_server_method.cc
  src/xml_rpc_socket.cc
  src/xml_rpc_source.cc
  src/xml_rpc_util.cc
  src/xml_rpc_value.cc
  libb64/src/cdecode.c
  libb64/src/cencode.c
)

install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(xmlrpcpp)
ament_package()

