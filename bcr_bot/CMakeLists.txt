cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz config
    DESTINATION share/${PROJECT_NAME}/
)
         
ament_package()
