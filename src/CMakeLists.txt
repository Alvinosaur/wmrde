cmake_minimum_required(VERSION 2.8.12.2)
project(TerrainPlanning)

set(CFLAGS
    -std=c++11
    -g 
    -m64
	-I /usr/local/include/eigen3
)

add_compile_options(${CFLAGS})
find_package(Eigen3 REQUIRED)

include_directories(
    ../include
	${EIGEN3_INCLUDE_DIR}
)

set(ALL_LIBS
	${EIGEN3_INCLUDE_DIR}
)

set(${PROJECT_NAME}_SOURCES
  algebra/matrix.cpp
  algebra/spatial.cpp
  algebra/transform.cpp
  demo/rockymodel.cpp
  demo/talonmodel.cpp
  demo/terrains.cpp
  demo/zoemodel.cpp
  surface/GridSurf.cpp
  surface/PlaneSurf.cpp
  surface/Surface.cpp
  surface/TriMeshSurf.cpp
  actuator.cpp
  collision.cpp
  dynamics.cpp
  eigensolve.cpp
  kinematics.cpp
  main.cpp
  state.cpp
  wheelgroundcontact.cpp
  WmrModel.cpp  
)

# Main
add_executable(${PROJECT_NAME}_main ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME}_main ${ALL_LIBS})
