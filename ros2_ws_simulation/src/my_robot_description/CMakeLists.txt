cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Installer les dossiers urdf et launch
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Installer les scripts Python comme exécutables ROS2
install(
  PROGRAMS
    src/obstacle_avoidance.py
    src/simple_delivery.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
