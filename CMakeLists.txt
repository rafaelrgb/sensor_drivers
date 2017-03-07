cmake_minimum_required(VERSION 2.8.3)
project(sensor_drivers)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/pi/wiringPi)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

link_directories(
)

set(H_FILES
)

set(CPP_FILES
)

add_executable(sonar_driver src/sonar_driver.cpp ${CPP_FILES})
add_dependencies(sonar_driver ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(sonar_driver ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
