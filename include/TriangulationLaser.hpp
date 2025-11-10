#pragma once
#include <string>
#include <cstdint>
#include <vector>
#include <memory>
#include <array>

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
        SAVE_TO_FLASH = 0x04,
        RESTORE_DEFAULTS = 0x04,
        LATCH_RESULT = 0x05,
        REQUEST_RESULT = 0x06,
        START_STREAM = 0x07,
        STOP_STREAM = 0x08
    };

    enum class RiftekParameter : uint8_t {
        LASER_ENABLE = 0x00,
        ANALOG_OUTPUT_ENABLE = 0x01,
        CONTROL_REG = 0x02,
        PROTOCOL_MODE = 0x8A
    };

    static constexpr std::array<uint8_t,4> CMD_SWITCH_TO_MODBUS={0x01,0x03,0x8A,0x02};


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
    bool riftek_switchToModbus();
    bool riftek_startRawMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename);
    bool riftek_startConvertedMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename);

    bool modbus_setLaserEnabled(bool enable);
    bool modbus_readRawMeasurement(uint16_t& rawValue);
    bool modbus_startRawMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename);
    bool modbus_startConvertedMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename);
    
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

    modbus_t* ctx_ = nullptr;
    bool modbus_connected_= false;

    LaserMode mode_=LaserMode::Unknown;

    std::string normalizePortName(const std::string& port) const;
    void handleError(const std::string&context) const;
    bool setupSerialPort(boost::asio::serial_port&serial);
};