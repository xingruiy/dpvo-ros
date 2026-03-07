cmake_minimum_required(VERSION 3.8)
project(dpvo_retrieval)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(pybind11)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dbow2_ros REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(DBoW2 REQUIRED)
# include_directories(${DBoW2_INCLUDE_DIRS})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
pybind11_add_module(dpvo_retrieval src/main.cpp)
# target_link_libraries(dpvo_retrieval PUBLIC ${OpenCV_LIBS} ${DBoW2_LIBS} )
ament_target_dependencies(dpvo_retrieval PUBLIC dbow2_ros)

ament_package()