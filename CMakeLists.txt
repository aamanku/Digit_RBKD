cmake_minimum_required(VERSION 3.0.0)
project(Digit_RBKD VERSION 0.1.0 LANGUAGES C CXX)
# message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
#optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3 REQUIRED)
# find_package(Gperftools REQUIRED) 
include(CTest)
enable_testing()

include_directories(${EIGEN3_INCLUDE_DIR} include)

add_executable(Digit_RBKD src/main.cpp)
target_link_libraries(Digit_RBKD Eigen3::Eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
