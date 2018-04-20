cmake_minimum_required(VERSION 3.0)
project( g2o_simple_test )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#设定二进制文件存放路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#设定库文件放置路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#设定.cmake 文件路径
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# find packages

find_package( G2O REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( CSparse REQUIRED )

include_directories( ${EIGEN3_INCLUDE_DIR})
include_directories( ${CSPARSE_INCLUDE_DIR})


SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core
g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba
g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data
g2o_types_sim3 cxsparse )

add_subdirectory( ${PROJECT_SOURCE_DIR}/src )

