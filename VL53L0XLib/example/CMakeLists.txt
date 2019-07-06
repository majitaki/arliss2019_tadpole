cmake_minimum_required(VERSION 2.8)
set(PROJ_NAME vl53l0x_example)
set(VERSION_NAME develop)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

project(${PROJ_NAME})
link_directories(/usr/local/lib/)

add_executable(
	example	
	example.c
)

target_link_libraries(
	example
	-lvl53l0x
)