#pragma once
#include <string>
#include <cstdint>
#include <vector>
#include <memory>
#include <array>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>

#include <modbus/modbus.h>
#include <boost/asio.hpp>

enum class LaserMode{
    Unknown,
    Riftek,
    Modbus
};

class TriangulationLaser{
public:
    enum class RiftekCommand : uint8_t {
        ID = 0x01,
        READ_PARAM = 0x02,
        WRITE_PARAM = 0x03,
        FLASH = 0x04,
        LATCH_RESULT = 0x05,
        REQUEST_RESULT = 0x06,
        START_STREAM = 0x07,
        STOP_STREAM = 0x08
    };

    enum class RiftekFlashAction{
        Save,
        Restore
    };

    enum class RiftekParameter : uint8_t {
        LASER_ENABLE = 0x00,
        ANALOG_OUTPUT_ENABLE = 0x01,
        CONTROL_REG = 0x02,
        PROTOCOL_MODE = 0x8A
    };

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

    bool riftek_requestId(std::vector<uint8_t>* response);
    bool riftek_requestResult(uint16_t& rawValue);
    bool riftek_startStream();
    bool riftek_stopStream();
    bool riftek_writeParameter(uint8_t paramCode,uint8_t value);
    bool riftek_readParameter(uint8_t paramCode,uint8_t& value);
    bool riftek_flashAction(RiftekFlashAction action);
    bool riftek_switchToModbus();

    bool modbus_setLaserEnabled(bool enable);
    bool modbus_readRawMeasurement(uint16_t& rawValue);

    double convertRawToMm(uint16_t rawValue) const;

    LaserMode getMode() const{return mode_;}
    
private:
    std::string portName_;
    int baudrate_;
    int dataBits_;
    int stopBits_;
    int slaveId_;
    char parity_;
    double sensorRangeMm_;

    boost::asio::io_context io_;
    std::unique_ptr<boost::asio::serial_port> serial_port_ = nullptr;
    bool serial_connected_=false;
    uint8_t riftek_address_=1;
    uint8_t last_riftek_cnt_=0;
    bool riftek_streaming_=false;

    modbus_t* ctx_ = nullptr;
    bool modbus_connected_= false;

    LaserMode mode_=LaserMode::Unknown;
    
    std::string normalizePortName(const std::string& port) const;
    void handleError(const std::string&context) const;
    bool setupSerialPort(boost::asio::serial_port&serial);
};