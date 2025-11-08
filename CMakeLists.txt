cmake_minimum_required(VERSION 3.8)
project(galaxsea26)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)

# install(
#   DIRECTORY sensors/urdf
#   DESTINATION share/${PROJECT_NAME}/urdf
# )

# install(
#   DIRECTORY sensors/meshes
#   DESTINATION share/${PROJECT_NAME}/meshes
# )

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )


ament_package()