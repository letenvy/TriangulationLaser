#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <memory>
#include <optional>

#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
//#include <modbus/modbus.h>

enum class LaserMode {
    Unknown,
    Riftek
    //Modbus
};

class TriangulationLaser{
public:
    class Riftek{
    public:
        explicit Riftek(TriangulationLaser& parent): parent_(parent){}
        
        bool sendRequest(const std::vector<uint8_t>& request);
        std::optional<std::vector<uint8_t>>listenAnswer(int timeout_ms=2000);

        std::optional<uint16_t> requestResult();
        std::optional<std::vector<uint8_t>> requestID();
        bool startStream();
        bool stopStream();
        bool writeParameter(uint8_t paramCode,uint8_t value);
        std::optional<uint8_t> readParameter(uint8_t paramCode);

    private:
        TriangulationLaser& parent_;
    };

    explicit TriangulationLaser(const std::string&portName,
                                int baudrate=9600,
                                char parity='E',
                                int dataBits=8,
                                int stopBits=1,
                                double semsorRangeMm=500.0,
                                double baseRangeMm=125);
    ~TriangulationLaser();

    Riftek riftek(){return Riftek(*this);}

    bool connect();
    void disconnect();
    bool isConnected() const;
    //bool switchToModbus();

    double convertRawToMm(uint16_t rawValue) const;

    LaserMode getMode() const{return mode_;}
    
private:
    std::string portName_;
    int baudrate_;
    int dataBits_;
    int stopBits_;
    char parity_;
    double sensorRangeMm_;
    double minDistanceMm_;

    boost::asio::io_context io_;
    std::unique_ptr<boost::asio::serial_port> serial_port_=nullptr;
    bool serial_connected_=false;

    LaserMode mode_=LaserMode::Unknown;
    uint8_t riftek_address_=1;

    std::string normalizePortName(const std::string&port) const;
    bool setupSerialPort(boost::asio::serial_port& serial);
};