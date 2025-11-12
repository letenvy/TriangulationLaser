cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0167 NEW)
project(LaserScanner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======/  Dependendencies /======
#   ---/   MODBUS  /---
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)

#   ---/   BOOST   /---
find_package(Boost REQUIRED COMPONENTS system)

#   ---/  JSON    /---
find_package(nlohmann_json REQUIRED)

# ======/   SOURCES  /======
set(simple_test
    src/TriangulationLaser.cpp
    src/ConsoleInput.cpp
    src/test.cpp
)
set(riftek_test
    src/TriangulationLaser.cpp
    src/ConsoleInput.cpp
    src/riftek.cpp
)
#   ---/  Executable  /---
add_executable(tester_simple ${simple_test})
add_executable(tester_riftek ${riftek_test})
#   ---/  Headers /---
target_include_directories(tester_simple PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(tester_riftek PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
#   ---/ Link /---
target_link_libraries(tester_simple 
    #PkgConfig::MODBUS
    Boost::system
    nlohmann_json::nlohmann_json
)
target_link_libraries(tester_riftek 
    #PkgConfig::MODBUS
    Boost::system
    nlohmann_json::nlohmann_json
)

#   ---/  Windows-special  /---
if(WIN32)
    target_link_libraries(tester_simple ws2_32)
    target_link_libraries(tester_riftek ws2_32)
endif()