cmake_minimum_required(VERSION 3.0.2)
project(dyn_pkg)

######===========================--- 版本设定 ---===========================######
SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -w -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -w")

######===========================--- packages ---===========================######
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  # interface
  # gazebo_ros
)

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(qpOASES REQUIRED)

catkin_package()

######===========================--- 包含路径 ---===========================######
link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

######===========================--- 生成C++可执行文件 ---===========================######
add_executable(${PROJECT_NAME}_node  
  # ======================================================= # 
  # src/test.cpp
  src/main.cpp
  src/ross/RosInterface.cpp
  src/fed/dynamic_solution
  src/fed/RobotFeed.cpp
  src/ref/RefTrajes.cpp
  src/wbc/dataRW/DataRW.cpp
  
  src/wbc/trajes/TrajEuc.cpp
  src/wbc/constraints/Constraints_Base.cpp
  src/wbc/constraints/Constraint_Equality.cpp
  src/wbc/constraints/Constraint_Target.cpp
  src/wbc/constraints/Constraint_InEquality.cpp
  src/wbc/constraints/Constraint_Bound.cpp
  src/wbc/constraints/Constraint_Zero.cpp
  src/wbc/tasks/BaseTasksConst.cpp
  src/wbc/tasks/TaskAll.cpp
  src/wbc/tasks/ConstDynamicConsistence.cpp
  src/wbc/tasks/ConstJointsTauLimit.cpp
  src/wbc/qpSolve/QpWbc.cpp
  src/wbc/Formulations.cpp

  src/wbc/dataPlot/DataPlot.cpp
  # ======================================================= # 
)
target_link_libraries(${PROJECT_NAME}_node    
    # qpOASES
    ${catkin_LIBRARIES}
    ${PYTHON_LIBRARIES}
    pinocchio::pinocchio  
    yaml-cpp
    # lib_xenomai
)

######===========================--- 生成C++可执行文件 ---===========================######