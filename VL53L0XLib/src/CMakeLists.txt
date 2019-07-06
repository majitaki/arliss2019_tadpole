cmake_minimum_required(VERSION 2.8)
set(PROJ_NAME vl53l0x)
set(VERSION_NAME develop)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

project(${PROJ_NAME})

set(libsrc
	tof.c
)

add_library(
	${PROJ_NAME}
	${libsrc}  
)

INSTALL(TARGETS ${PROJ_NAME} DESTINATION lib)
INSTALL(FILES tof.h DESTINATION include)
