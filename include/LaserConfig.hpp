#pragma once
#include <string>

struct LaserTestConfig {
    std::string port;
    int baudrate;
    char parity;
    int dataBits;
    int stopBits;
    int slaveId;
    double sensorRangeMm;
    int updateIntervalMs;
};