cmake_minimum_required(VERSION 3.5)
project(rsky)
set(CMAKE_CXX_STANDARD 14)

set(mySRC ${CMAKE_SOURCE_DIR}/src) 
set(myInclude ${CMAKE_SOURCE_DIR}/include)
set(myLib ${CMAKE_SOURCE_DIR}/libs)

add_subdirectory(${myLib}/thermal-camera)
add_subdirectory(${myLib}/Navio2)
include_directories(${myInclude} ${myLib}/eigen-5.0.0) 

add_executable(rsky ${mySRC}/main.cpp)
target_link_libraries(rsky PRIVATE thermal-camera Common Navio2)
add_executable(throttle ${mySRC}/throttle_pass.cpp)
target_link_libraries(rsky PRIVATE thermal-camera Common Navio2)
#target_link_libraries(rsky PRIVATE thermal-camera)
