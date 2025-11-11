cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0167 NEW)
project(LaserScanner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======/  Dependendencies /======
#   ---/   MODBUS  /---
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)

#   ---/   BOOST   /---
find_package(Boost REQUIRED COMPONENTS system)

#   ---/  JSON    /---
find_package(nlohmann_json REQUIRED)

# ======/   SOURCES  /======
set(SOURCES
    src/TriangulationLaser.cpp
    src/ConsoleInput.cpp
    src/laser_riftek_test.cpp
)
#   ---/  Executable  /---
add_executable(laser_scanner ${SOURCES})
#   ---/  Headers /---
target_include_directories(laser_scanner PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
#   ---/ Link /---
target_link_libraries(laser_scanner 
    PkgConfig::MODBUS
    Boost::system
    nlohmann_json::nlohmann_json
)

#   ---/  Windows-special  /---
if(WIN32)
    target_link_libraries(laser_scanner ws2_32)
endif()