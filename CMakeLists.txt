cmake_minimum_required(VERSION 3.19)
project(PRP_projekt)

set(CMAKE_CXX_STANDARD 14)

find_package(Yaml-cpp)
find_package(Threads REQUIRED)
find_package(Doxygen)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(SOURCES
        #include/RosCommunication.h
        #include/communication.h
        #include/Robot.h
        src/main.cpp
        src/UDPSocket.cpp
        src/Robot.cpp
        include/RvizCommunication.h)

add_executable(PRP_projekt ${SOURCES})

#add_executable(PRP_projekt src/main.cpp)

target_link_libraries(PRP_projekt yaml-cpp Threads::Threads ${catkin_LIBRARIES})