cmake_minimum_required(VERSION 3.8)
project(motion_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)

add_library(parallel_planner src/parallel_planner.cpp)
target_include_directories(parallel_planner PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(parallel_planner
        rclcpp
        geometry_msgs
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
)



add_executable(motion_pipeline_node src/motion_pipeline_node.cpp)
target_link_libraries(motion_pipeline_node parallel_planner)
ament_target_dependencies(motion_pipeline_node
        rclcpp rclcpp_action interfaces moveit_msgs moveit_core
)

install(TARGETS parallel_planner
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS motion_pipeline_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(parallel_planner)

ament_package()


