#pragma once
#include <string>

struct LaserConfig {
    std::string deviceName="Generic";
    std::string port;
    int baudrate;
    char parity;
    int dataBits;
    int stopBits;
    int slaveId;
    double sensorRangeMm;
    int updateIntervalMs;
};