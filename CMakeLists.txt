cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ScanReconstruction)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

file (GLOB MY_SRC src/*.h
                  src/*.cpp)

add_executable(ScanReconstruction "${MY_SRC}")

target_link_libraries(ScanReconstruction "${MY_LIBS}" ${PCL_LIBRARIES})



 


 




