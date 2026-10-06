cmake_minimum_required(VERSION 3.8)
project(leap_hand_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz DESTINATION share/${PROJECT_NAME})
# Meshes stay in the unmodified vendor copy (one copy in the repo); installed as
# share/leap_hand_description/meshes so package://leap_hand_description/meshes/... resolves.
install(DIRECTORY ../vendor/leap_hand/meshes DESTINATION share/${PROJECT_NAME})

ament_package()
