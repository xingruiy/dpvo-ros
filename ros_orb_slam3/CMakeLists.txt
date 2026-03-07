cmake_minimum_required(VERSION 3.8)
project(ros_orb_slam3 LANGUAGES CXX CUDA)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS} -Wall ")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -Wall")
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")

find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include/orb_slam3
  ${PROJECT_SOURCE_DIR}/include/orb_slam3/CameraModels
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/g2o
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(dbow2_ros REQUIRED)
# find_package(g2o_ros REQUIRED)
# find_package(sophus_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(orb_slam_node
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  # src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  # src/MapDrawer.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  # src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
  src/cuda/Allocator_gpu.cu
  src/cuda/Cuda.cu
  src/cuda/Fast_gpu.cu
  src/cuda/Orb_gpu.cu
)

set(DEPENDENCIES
  # OpenCV
  rclcpp
  # dbow2_ros
  # g2o_ros
  # sophus_ros
  cv_bridge
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  visualization_msgs
)

add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

ament_target_dependencies(orb_slam_node ${DEPENDENCIES})

target_link_libraries(orb_slam_node
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)

# ament_export_libraries(orb_slam3)

# # ROS Node
# add_executable(orb_slam_node src/RosNode.cpp)
# ament_target_dependencies(orb_slam_node
#   ${DEPENDENCIES}
#   Eigen3
#   rclcpp
#   nav_msgs
#   cv_bridge
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   message_filters
# )
# target_link_libraries(orb_slam_node orb_slam3)
install(TARGETS orb_slam_node
  DESTINATION lib/${PROJECT_NAME})

install(Directory configs launch Vocabulary
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
