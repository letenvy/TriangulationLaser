cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0167 NEW)
project(LaserScanner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#====================MODBUS ====================#
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)
#====================BOOST  ====================#
find_package(Boost REQUIRED COMPONENTS system)

add_executable(laser_scanner    
    src/laser_test.cpp
    src/main.cpp
    src/TriangulationLaser.cpp
)

target_include_directories(laser_scanner PRIVATE 
${MODBUS_INCLUDE_DIRS}
include
)

target_link_directories(laser_scanner PRIVATE 
${MODBUS_LIBRARY_DIRS}
)

target_link_libraries(laser_scanner 
${MODBUS_LIBRARIES} 
Boost::system
ws2_32
)