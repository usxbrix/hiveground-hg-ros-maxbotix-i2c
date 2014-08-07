cmake_minimum_required(VERSION 2.8.3)
project(maxbotix_i2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
    DEPENDS
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_executable(maxbotix_i2c
    src/maxbotix_i2c.cpp    
)

target_link_libraries(maxbotix_i2c
    ${catkin_LIBRARIES}
)


