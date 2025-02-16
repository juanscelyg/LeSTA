cmake_minimum_required(VERSION 3.0.2)
project(lesta)
add_compile_options(-std=c++17)

set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/libs/libtorch)
# find_package(Torch REQUIRED HINTS ${Torch_DIR})

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    height_mapping_core
    height_mapping_io
    roscpp
    tf2_ros
    pcl_conversions
    message_generation
)

add_service_files(
  FILES
  save_training_data.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    height_mapping_core
    height_mapping_io
    roscpp
    tf2_ros
    pcl_conversions
)


###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

# Node: Label generation
add_executable(label_generation_node 
  src/ros/label_generation_node.cpp
  src/core/HeightMapper.cpp
  src/core/GlobalMapper.cpp
  src/core/FeatureExtractor.cpp
  src/core/LabelGenerator.cpp
)
target_link_libraries(label_generation_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Node: Traversability estimation
add_executable(trav_estimation_node
  src/ros/trav_estimation_node.cpp
  src/core/HeightMapper.cpp
  src/core/FeatureExtractor.cpp
  src/core/TraversabilityEstimator.cpp
)
target_link_libraries(trav_estimation_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# # Node: Traversability mapping
# add_executable(traversability_prediction_node
#   src/traversability_prediction/traversability_prediction_node.cpp
#   src/traversability_prediction/TraversabilityPrediction.cpp
#   src/traversability_prediction/TraversabilityMap.cpp
#   src/traversability_prediction/classifiers/LearnedTraversability.cpp
# )
# target_link_libraries(traversability_prediction_node
#   ${PCL_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${TORCH_LIBRARIES}
# )