cmake_minimum_required(VERSION 3.8)
project(mmw_radar_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg"
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
