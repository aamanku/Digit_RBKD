cmake_minimum_required(VERSION 3.5.0)
project(Digit_RBKD VERSION 0.1.0 LANGUAGES C CXX)
# message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
#optimization flags for release and debug

set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG -march=native ")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# define a variable for the source directory to be used in the code\
add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)

## Library ##
add_library(digit_model_library SHARED src/digit_model_library.cpp)
target_link_libraries(digit_model_library Eigen3::Eigen casadi)
target_include_directories(digit_model_library PUBLIC include)


##  Main  ##
include_directories(${EIGEN3_INCLUDE_DIR} include )
add_executable(generate_code src/generate_code.cpp)
target_link_libraries(generate_code Eigen3::Eigen casadi digit_model_library)

include_directories(${EIGEN3_INCLUDE_DIR} include codegen/include)
add_executable(use_generated_code src/use_generated_code.cpp
                                    codegen/src/digit_rbkd.cpp)
target_link_libraries(use_generated_code Eigen3::Eigen)





###  Testing  ###
add_executable(digit_model_test src/test/digit_model.cpp)
target_link_libraries(digit_model_test Eigen3::Eigen casadi digit_model_library)
add_test(NAME digit_model_test COMMAND digit_model_test )

###
add_executable(casadi_symbolic_eigen src/test/casadi_symbolic_eigen.cpp)
#target_precompile_headers(casadi_symbolic_eigen PRIVATE include/digit_rbkd/rigid_body.hxx
#                                                        include/digit_rbkd/digit_model.hxx)
target_link_libraries(casadi_symbolic_eigen Eigen3::Eigen casadi digit_model_library)
add_test(NAME casadi_symbolic_eigen COMMAND casadi_symbolic_eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
