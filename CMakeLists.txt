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

add_executable(sonar_driver_serial src/sonar_driver_serial.cpp ${CPP_FILES})
add_dependencies(sonar_driver_serial ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(sonar_driver_serial ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})

add_executable(sonar_driver_pwm src/sonar_driver_pwm.cpp ${CPP_FILES})
add_dependencies(sonar_driver_pwm ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(sonar_driver_pwm ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})

add_executable(sonar_driver_undersample src/sonar_driver_undersample.cpp ${CPP_FILES})
add_dependencies(sonar_driver_undersample ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(sonar_driver_undersample ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
