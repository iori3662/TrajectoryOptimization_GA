cmake_minimum_required(VERSION 3.16)
project(TrajectoryOptimization_GA LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenMP)
add_executable(trajectory_ga src/main.cpp)
if(OpenMP_CXX_FOUND)
  target_link_libraries(trajectory_ga PRIVATE OpenMP::OpenMP_CXX)
endif()
