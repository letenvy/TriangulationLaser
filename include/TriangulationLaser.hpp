#pragma once
#include <string>
#include <cstdint>
#include <vector>
#include <modbus/modbus.h>
#include <boost/asio.hpp>

class TriangulationLaser{
public:
    explicit TriangulationLaser(const std::string& portName,
                                int baudrate=9600,
                                char parity='E',
                                int dataBits=8,
                                int stopBits=1,
                                int slaveId=1,
                                double sensorRangeMm=500.0);
    ~TriangulationLaser();

    bool connect();
    void disconnect();
    bool isConnected() const;
    bool setLaserEnabled(bool enable);

    bool switchToModbus();

    bool startRawMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename);
    bool startConvertedMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename);
    bool readRawMeasurement(uint16_t& rawValue);
    double convertRawToMm(uint16_t rawValue) const;
    
private:
    std::string normalizePortName(const std::string& port) const;
    void handleError(const std::string&context) const;
    
    std::string portName_;
    int baudrate_;
    int dataBits_;
    int stopBits_;
    int slaveId_;
    char parity_;
    double sensorRangeMm_;
    modbus_t* ctx_=nullptr;
    bool connected_=false;
};