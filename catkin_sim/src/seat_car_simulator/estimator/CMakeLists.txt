cmake_minimum_required(VERSION 2.8.3)
project(estimator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS std_msgs geometry_msgs tf2_ros tf2 tf ar_track_alvar ar_track_alvar_msgs roscpp l4vehicle_msgs lpv_mpc message_generation )

find_package(Eigen3 REQUIRED)

# EIGEN3_INCLUDE_DIRS = /usr/include/eigen3
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_message_files(
   FILES
   Gains.msg
   SchedVars.msg
   ErrorStates.msg
   ErrorsInfo.msg
   DataInfo.msg
   LandmarksInfo.msg
   LandmarkInfo.msg
   SendCov.msg
 )

add_service_files(
   FILES
   lmi_data.srv
 )

generate_messages(DEPENDENCIES std_msgs geometry_msgs ar_track_alvar_msgs l4vehicle_msgs lpv_mpc)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS std_msgs geometry_msgs ar_track_alvar tf tf2_ros tf2 tf message_filters ar_track_alvar_msgs l4vehicle_msgs lpv_mpc message_runtime 
 DEPENDS eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/est_lib.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#add_executable(zekf src/zekf.cpp)

add_executable(correctMeasurements src/parseTopics.cpp)
add_executable(class_zekf src/class_ekf.cpp)
add_executable(sync src/TimeSync.cpp)
add_executable(zekf src/zekf_SLAM_LMIng.cpp)
add_executable(ekf src/zekf_SLAMi.cpp)

target_link_libraries(correctMeasurements  ${catkin_LIBRARIES})
target_link_libraries(zekf ${catkin_LIBRARIES})
target_link_libraries(ekf ${catkin_LIBRARIES})
target_link_libraries(sync ${catkin_LIBRARIES})
target_link_libraries(class_zekf ${catkin_LIBRARIES})


add_dependencies(correctMeasurements ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(zekf ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ekf ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sync ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(class_zekf ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies( ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
