cmake_minimum_required(VERSION 3.5)
project(drone_sim)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
    DIRECTORY launch worlds
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    src/controller.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
