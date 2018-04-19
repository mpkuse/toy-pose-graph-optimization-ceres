cmake_minimum_required(VERSION 2.8.3)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

set( CMAKE_CXX_FLAGS "-fpermissive" )


include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DBOW3_INCLUDE_DIRS}
)


#add_executable( ceres_try ceres_try.cpp )
#target_link_libraries( ceres_try ${CERES_LIBRARIES}   )


add_executable( toy_pose_graph ceres_vertigo.cpp )
target_link_libraries( toy_pose_graph ${CERES_LIBRARIES}   )
