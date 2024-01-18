cmake_minimum_required(VERSION 3.10)
project(social_force_model)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospack
)

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

find_package(VTK REQUIRED)
find_package(yaml-cpp REQUIRED)

add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YAML_CPP_INCLUDE_DIR})


add_executable(example src/example.cpp)
target_link_libraries(example ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})


