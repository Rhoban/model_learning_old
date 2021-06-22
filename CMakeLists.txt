cmake_minimum_required(VERSION 3.16.3)
project(rhoban_model_learning)

option(BUILD_MODEL_LEARNING_TOOLS "Build the tools from model learning" OFF)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

###########
## Build ##
###########

# TODO integrate back ball_models and humanoid_models

# Directories containing sources files
set(DIRECTORIES
  src/rhoban_model_learning/
  src/rhoban_model_learning/basic_models
  src/rhoban_model_learning/camera_calibration/
#  src/rhoban_model_learning/ball_models
#  src/rhoban_model_learning/models
  src/rhoban_model_learning/humanoid_models
  src/rhoban_model_learning/tags
  )

# Build ALL_SOURCES
set (SOURCES)
set (ALL_SOURCES)
set (PREFIXED_SOURCES)
foreach (DIRECTORY ${DIRECTORIES})
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/${DIRECTORY}")
    include (${DIRECTORY}/sources.cmake)

    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)

    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

find_package(OpenCV 4.2 REQUIRED)

add_library(rhoban_model_learning ${ALL_SOURCES})
target_link_libraries(rhoban_model_learning PUBLIC
  rhoban_bbo
  robot_model
  hl_monitoring
  ${OpenCV_LIBRARIES}
)
target_include_directories(rhoban_model_learning PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Binaries
# TODO integrate back as a test
#add_executable(test_simple_angular_model src/tests/test_simple_angular_model.cpp)
#target_link_libraries(test_simple_angular_model ${catkin_LIBRARIES} rhoban_model_learning)

add_executable(model_learner apps/model_learner.cpp)
target_link_libraries(model_learner PRIVATE rhoban_model_learning)

add_executable(model_learning_analyzer apps/model_learning_analyzer.cpp)
target_link_libraries(model_learning_analyzer PRIVATE rhoban_model_learning)

#add_executable(vision_correction_debug tools/vision_correction_debug.cpp)
#target_link_libraries(vision_correction_debug ${catkin_LIBRARIES} rhoban_model_learning)
#
#add_executable(ball_sequences_filter tools/ball_sequences_filter.cpp)
#target_link_libraries(ball_sequences_filter ${catkin_LIBRARIES} rhoban_model_learning)
#
#add_executable(ball_trajectory_predictor tools/ball_trajectory_predictor.cpp)
#target_link_libraries(ball_trajectory_predictor ${catkin_LIBRARIES} rhoban_model_learning)
if (BUILD_MODEL_LEARNING_TOOLS)
  add_executable(marker_board_extractor tools/marker_board_extractor.cpp)
  target_link_libraries(marker_board_extractor PRIVATE rhoban_model_learning)

  add_executable(opencv_camera_calibration tools/opencv_camera_calibration.cpp)
  target_link_libraries(opencv_camera_calibration PRIVATE rhoban_model_learning)

  add_executable(aruco_extractor tools/aruco_extractor.cpp)
  target_link_libraries(aruco_extractor PRIVATE rhoban_model_learning)
endif()

enable_testing()
 
set(TESTS
  composite_model
  basic_models/linear_model
  basic_models/pose_2d_model
  humanoid_models/calibration_model
  humanoid_models/pose_model
  humanoid_models/poses_optimization_data_set_reader
  humanoid_models/infered_poses_data_set_reader
  tags/aruco_cube
  tags/tags_sheet
  tags/aruco_tag
# TODO: reintegrate
#  humanoid_models/vision_correction_model
)

if (CATKIN_ENABLE_TESTING)
  # create an executable for each test
  foreach(test ${TESTS})
    string(REPLACE "/" "_" id ${test})
    catkin_add_gtest(${id} tests/${test}.cpp)
    target_link_libraries(${id} rhoban_model_learning)
  endforeach(test)
endif()

