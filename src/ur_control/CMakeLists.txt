cmake_minimum_required(VERSION 3.0.2)
project(ur_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_visual_tools
)
find_package(Eigen3 REQUIRED)

catkin_package(
   CATKIN_DEPENDS   roscpp
)

###########
## Build ##
###########

INCLUDE_DIRECTORIES(src/3_party_library/kdl/include  src/3_party_library/track_Ik/include   src/3_party_library/velocity_profile/include  ${catkin_INCLUDE_DIRS})
LINK_DIRECTORIES(src/3_party_library/track_Ik/lib  src/3_party_library/kdl/lib   )

add_executable(ur_movej src/movej/ur_movej.cpp  src/3_party_library/velocity_profile/src/interpolate.cc  )
target_link_libraries(ur_movej PUBLIC    kdl-1-5     trac_ik    ${catkin_LIBRARIES})

