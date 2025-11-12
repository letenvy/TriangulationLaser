#pragma once
#include <string>

struct LaserConfig {
    std::string deviceName="RIFTEK RF 603 125/500-232-U";
    std::string port;
    int baudrate=9600;
    char parity='E';
    int dataBits=8;
    int stopBits=1;
    double sensorRangeMm=500.0;
    double baseRangeMm = 125.0;
    int updateIntervalMs=100;

    //int slaveId=1;
};