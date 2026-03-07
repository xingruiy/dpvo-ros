cmake_minimum_required(VERSION 3.8)
project(dbow2_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
find_package(ament_cmake REQUIRED)

set(HDRS
  include/DBoW2/BowVector.h
  include/DBoW2/FBrief.h
  include/DBoW2/QueryResults.h
  include/DBoW2/TemplatedDatabase.h
  include/DBoW2/FORB.h
  include/DBoW2/DBoW2.h
  include/DBoW2/FClass.h
  include/DBoW2/FeatureVector.h
  include/DBoW2/ScoringObject.h
  include/DBoW2/TemplatedVocabulary.h)

set(SRCS 
  src/BowVector.cpp
  src/FBrief.cpp
  src/FORB.cpp
  src/FeatureVector.cpp 
  src/QueryResults.cpp  
  src/ScoringObject.cpp)

set(DEPENDENCY_DIR ${CMAKE_CURRENT_BINARY_DIR}/dependencies)
set(DEPENDENCY_INSTALL_DIR ${DEPENDENCY_DIR}/install)

find_package(OpenCV REQUIRED)

set(LIB_SHARED "SHARED")
if(WIN32)
  set(LIB_SHARED "STATIC")
endif(WIN32)

add_library(${PROJECT_NAME} ${LIB_SHARED} ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)

configure_file(src/DBoW2.cmake.in
  "${PROJECT_BINARY_DIR}/DBoW2Config.cmake" @ONLY)

ament_export_include_directories(include)

# Install
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()