cmake_minimum_required(VERSION 2.8.3)
project(kubot_duo_box)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
##   cfg/kubot_duo_parameter.cfg
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include include/kubot_duo_box/
)

add_executable(
  kubot_duo_box
  src/main.cpp
  src/base_driver_config.cpp
  src/base_driver.cpp
  src/data_holder.cpp
  src/simple_dataframe_master.cpp
  src/serial_transport.cpp
  src/serial_transport2.cpp
)

add_dependencies(kubot_duo_box ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kubot_duo_box
  ${catkin_LIBRARIES}
)