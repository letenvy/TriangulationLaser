#include "TriangulationLaser.hpp"
#include "LaserConfig.hpp"
#include <iostream>
#include <thread>
#include <chrono>

LaserTestConfig default_config ={
#ifdef _WIN32
   .port = "COM3",
#else
    .port ="/dev/ttyUSB0", // didn't check port for Linux yet 
#endif
    .baudrate = 9600,
    .parity = 'E',
    .dataBits = 8,
    .stopBits = 1,
    .slaveId = 1,
    .sensorRangeMm = 500.0,
    .updateIntervalMs = 100
};

int main(){
    std::cout<<"Detected OS: "
#ifdef _WIN32
    <<"Windows"
#else
    <<"Linux"
#endif
    <<std::endl;

    //nothing to do until class ConsoleInput not finished

    return 0;
}