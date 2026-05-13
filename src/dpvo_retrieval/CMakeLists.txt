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
pybind11_add_module(dpretrieval src/main.cpp)
ament_target_dependencies(dpretrieval PUBLIC dbow2_ros)
target_link_libraries(dpretrieval PRIVATE ${OpenCV_LIBS})

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
  "import sys; print(f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages')"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)

install(TARGETS dpretrieval
  LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES_DIR})

ament_package()