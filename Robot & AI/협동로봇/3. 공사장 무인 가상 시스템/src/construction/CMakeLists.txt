cmake_minimum_required(VERSION 3.8)
project(construction)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Launch 파일 설치
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()