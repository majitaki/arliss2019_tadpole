cmake_minimum_required(VERSION 2.8)
set(PROJ_NAME tadpole)
set(VERSION_NAME develop)
#SET(CMAKE_CXX_COMPILER /usr/bin/g++-6)
SET(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Wextra")

project(${PROJ_NAME})

# find opengl via pkg-config
find_package(PkgConfig)
pkg_check_modules(GPSD REQUIRED libgps)
include_directories(${GPSD_INCLUDE_DIRS})
include_directories(/usr/local/include)

link_directories(/usr/lib/)
link_directories(/usr/local/lib/)

set(rover_src
	rover_util/utils.cpp 
	rover_util/task.cpp 
	rover_util/serial_command.cpp 
	rover_util/alias.cpp 
	rover_util/delayed_execution.cpp
	rover_util/logging.cpp 
	actuator/motor.cpp 
	actuator/servo.cpp 
	sensor/gps.cpp 
	sensor/light.cpp 
	sensor/nineaxis.cpp 
	sensor/pressure.cpp 
	sensor/distance.cpp
	sensor/lora.cpp
	noisy/buzzer.cpp
	noisy/led.cpp
	sequence/testing_sequence.cpp
	sequence/separating_sequence.cpp
	sequence/waiting_sequence.cpp
	sequence/falling_sequence.cpp
	sequence/navigating_sequence.cpp
	sequence/closing_sequence.cpp
	sequence/arliss_sequence.cpp
	sub_sequence/waking_turnside.cpp
	sub_sequence/waking_turnback.cpp
	sub_sequence/stucking.cpp
	sub_sequence/near_navigating.cpp
	sub_sequence/digging.cpp
	main.cpp	
)

add_executable(
	${PROJ_NAME}
	${rover_src}  
)

set(LIBRARIES
	-lrt 
	-lpthread
	-lwiringPi
	-lvl53l0x
	${GPSD_LIBRARIES}
	)

target_link_libraries(${PROJ_NAME} 
	${LIBRARIES}
	RTIMULib
)
