cmake_minimum_required(VERSION 3.10)
project(ekf)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(ekflib STATIC 
${CMAKE_CURRENT_SOURCE_DIR}/dcm.cpp)
target_include_directories(ekflib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
	${extlibs}/eigen
)