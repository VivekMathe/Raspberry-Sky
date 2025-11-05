cmake_minimum_required(VERSION 3.10)
project(mlx90640_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define source files for the MLX90640 library
set(MLX90640_API_SOURCES
    ./MLX90640_API.c
    ./MLX90640_I2C_Driver.cpp
)

# Create MLX90640 API static library
add_library(MLX90640_API STATIC ${MLX90640_API_SOURCES})

# Add include directories for MLX90640_API
target_include_directories(MLX90640_API PUBLIC
    ${CMAKE_SOURCE_DIR}
)