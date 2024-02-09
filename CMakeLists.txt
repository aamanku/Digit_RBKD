cmake_minimum_required(VERSION 3.5.0)
project(Digit_RBKD VERSION 0.1.0 LANGUAGES C CXX)
# message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
#optimization flags for release and debug

set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -march=native -DNDEBUG -msse4")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)


##  Main  ##

include_directories(${EIGEN3_INCLUDE_DIR} include )

add_executable(Digit_RBKD src/main.cpp)
target_link_libraries(Digit_RBKD Eigen3::Eigen casadi)

###  Testing  ###
add_executable(casadi_symbolic_eigen src/test/casadi_symbolic_eigen.cpp)
target_link_libraries(casadi_symbolic_eigen Eigen3::Eigen casadi)
add_test(NAME casadi_symbolic_eigen COMMAND casadi_symbolic_eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
