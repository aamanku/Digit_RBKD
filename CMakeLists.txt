cmake_minimum_required(VERSION 3.5.0)
project(Digit_RBKD VERSION 0.1.0 LANGUAGES C CXX)
# message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
#optimization flags for release and debug

set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -march=native -DNDEBUG -msse4")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
## check if cppad there exists a cppad in include folder
if(NOT EXISTS "/usr/local/include/cppad")
    message(FATAL_ERROR "cppad not found in include folder")
endif()
## check if cppadcg there exists a cppadcg in include folder
if(NOT EXISTS "/usr/local/include/cppad/cg")
    message(FATAL_ERROR "cppadcg not found in include folder")
endif()


##  Main  ##

include_directories(${EIGEN3_INCLUDE_DIR} include ${Boost_INCLUDE_DIR})

add_executable(Digit_RBKD src/main.cpp)
target_link_libraries(Digit_RBKD Eigen3::Eigen Boost::filesystem)

###  Testing  ###
add_executable(CppADTest src/test/CppADTest.cpp src/temp_file_fix.cpp)
include_directories( ${EIGEN3_INCLUDE_DIR} include ${Boost_INCLUDE_DIR})
target_link_libraries(CppADTest Eigen3::Eigen  Boost::filesystem)

add_executable(CodeGenTest src/test/CodeGenTest.cpp src/temp_file_fix.cpp )
include_directories( ${EIGEN3_INCLUDE_DIR} include ${Boost_INCLUDE_DIRS})
target_link_libraries(CodeGenTest Boost::filesystem Eigen3::Eigen)

add_executable(CodeGenEigenTest src/test/CodeGenEigenTest.cpp src/temp_file_fix.cpp )
include_directories( ${EIGEN3_INCLUDE_DIR} include ${Boost_INCLUDE_DIRS})
target_link_libraries(CodeGenEigenTest Boost::filesystem Eigen3::Eigen)




set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
