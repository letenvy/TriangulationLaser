#include "TriangulationLaser.hpp"

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>

#include <modbus/modbus.h>

std::string TriangulationLaser::normalizePortName(const std::string& port) const{
    if(!port.empty()&&port.find("COM")==0&&port.size()>4)
        return"\\\\.\\"+port;
    return port;
}

bool TriangulationLaser::setupSerialPort(boost::asio::serial_port&serial){
    std::string port=normalizePortName(portName_);
    serial.open(port);

    serial.set_option(boost::asio::serial_port::baud_rate(baudrate_));
    serial.set_option(boost::asio::serial_port::character_size(dataBits_));

    auto par=boost::asio::serial_port::parity::none;
    if(parity_=='E') par=boost::asio::serial_port::parity::even;
    else if (parity_=='O') par=boost::asio::serial_port::parity::odd;
    serial.set_option(boost::asio::serial_port::parity(par));

    auto stop=boost::asio::serial_port_base::stop_bits::one;
    if(stopBits_==2) stop=boost::asio::serial_port_base::stop_bits::two;
    serial.set_option(boost::asio::serial_port_base::stop_bits(stop));

    return true;
}

void TriangulationLaser::handleError(const std::string&context)const{
    const char*err=modbus_strerror(errno);
    if(!err||std::strlen(err)==0){
        err=strerror(errno);
    }
    std::cerr<<"[Laser ERROR] "<<context<<": "<<err<<" (errno="<<errno<<")"<<std::endl;
}

double TriangulationLaser::convertRawToMm(uint16_t rawValue) const{
    constexpr double FULL_SCALE = 16384; //0x4000
    if (sensorRangeMm_<=0.0){
        std::cerr<<"[Laser] Invalid sensor range: "<<sensorRangeMm_<<std::endl;
        return 0.0;
    }
    return static_cast<double>(rawValue)*sensorRangeMm_/FULL_SCALE;
}

TriangulationLaser::TriangulationLaser( const std::string& portName,
                                        int baudrate,char parity,
                                        int dataBits,int stopBits,
                                        int slaveId, double sensorRangeMm)
    :portName_(portName)
    ,baudrate_(baudrate)
    ,parity_(parity)
    ,dataBits_(dataBits)
    ,stopBits_(stopBits)
    ,slaveId_(slaveId)
    ,sensorRangeMm_(sensorRangeMm)
    ,ctx_(nullptr)
    ,serial_connected_(false)
    ,modbus_connected_(false)
    ,mode_(LaserMode::Unknown)
{
    portName_=normalizePortName(portName_);
}

TriangulationLaser::~TriangulationLaser(){
    disconnect();
}

bool TriangulationLaser::connect(){
    if(serial_connected_) return true;

    try{
        serial_port_=std::make_unique<boost::asio::serial_port>(io_);
        setupSerialPort(*serial_port_);
        serial_connected_=true;
        mode_=LaserMode::Riftek;
        std::cout<<"[Laser] Connected in RIFTEK mode to "<<portName_<<std::endl;
        return true;
    }
    catch(const std::exception&e)
    {
        std::cerr<<"[Laser ERROR] connect: "<<e.what()<<std::endl;
        return false;
    }

    return true;
}

void TriangulationLaser::disconnect(){
    if(serial_port_&&serial_port_->is_open()){
        serial_port_->close();
        serial_connected_=false;
    }
    if(ctx_){
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_=nullptr;
        modbus_connected_=false;
    }
    mode_=LaserMode::Unknown;
    std::cout<<"[Laser] Disconnected"<<std::endl;
}

bool TriangulationLaser::isConnected() const{
    return serial_connected_||modbus_connected_;
}
/*
bool TriangulationLaser::sendRiftekCommand(const std::vector<uint8_t>& command){
    try{
        boost::asio::io_context io;
        boost::asio::serial_port serial(io);

        setupSerialPort(serial);

        boost::asio::write(serial,boost::asio::buffer(command));
        serial.close();

        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr<<"[Laser ERROR] sendRiftekCommand: "<<e.what()<<std::endl;
        return false;
    }
}

bool TriangulationLaser::setLaserEnabledRiftekMode(bool enable){

    std::vector<uint8_t> command={
        0x01U,0x03U,0x00U,static_cast<uint8_t>(enable?0x01U:0x00U)
    };

    if(!sendRiftekCommand(command)){
        std::cerr<<"[Laser] Failed to send laser "<<(enable?"ON":"OFF")<<" command (RIFTEK mode)"<<std::endl;
        return false;  
    }
    std::cerr<<"[Laser] Laser"<<(enable?"ON":"OFF")<<" (RIFTEK mode)"<<std::endl;
    return true;
}
*/
bool TriangulationLaser::riftek_switchToModbus(){
    try{
        boost::asio::io_context io;
        boost::asio::serial_port serial(io);
        
        setupSerialPort(serial);

        std::cout<<"[Switch] Sending RIFTEK -> MODBUS command to "<<portName_<<"..."<<std::endl;

        std::vector<uint8_t>command={0x01U,0x03U,0x8AU,0x02U};
        boost::asio::write(serial, boost::asio::buffer(command));

        std::cout<<"[Switch] Command sent. Waiting 1s for device to switch..."<<std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        serial.close();
        std::cout<<"[Switch] Success! Device is now in MODBUS RTU mode."<<std::endl;
        return true;

    }catch(const std::exception& e){
        std::cerr<<"[Switch ERROR] "<<e.what()<<std::endl;
        return false;
    }
}

bool TriangulationLaser::modbus_setLaserEnabled(bool enable){
    if(!modbus_connected_){
        std::cerr<<"[Laser] Not connected!"<<std::endl;
        return false;
    }
    uint16_t state = enable ? 1:0;
    if(modbus_write_register(ctx_,10,state)!=1){
        handleError("modbus_write_register");
        return false;
    }
    std::cout<<"[Laser] "<<(enable?"ON":"OFF")<<std::endl;
    return true;
}

bool TriangulationLaser::modbus_readRawMeasurement(uint16_t& rawValue){
    if(!modbus_connected_){
        std::cerr<<"[Laser] Not connected"<<std::endl;
        return false;
    }

    if(modbus_read_input_registers(ctx_,11,1,&rawValue)!=1){
        handleError("modbus_read_input_registers");
        return false;
    }
    return true;
}

bool TriangulationLaser::modbus_startRawMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename){
    if(!modbus_connected_){
        std::cerr<<"[Laser] Not connected!"<<std::endl;
        return false;
    }

    std::vector<uint16_t>data;
    auto start=std::chrono::steady_clock::now();
    auto duration =std::chrono::seconds(durationSeconds);
    auto interval=std::chrono::milliseconds(intervalMs);

    std::cout<<"[Laser] Starting raw measurement cycle..."<<std::endl;

    while(std::chrono::steady_clock::now()-start<duration){
        uint16_t rawVal;
        if(modbus_readRawMeasurement(rawVal)){
            data.push_back(rawVal);
        }
        std::this_thread::sleep_for(interval);
    }

    std::ofstream file(filename);
    if(!file.is_open()){
        std::cerr<<"[Laser] Cannot open file: "<<filename<<std::endl;
        return false;
    }

    for(uint16_t v:data){
        file<<v<<'\n';
    }
    file.close();

    std::cout<<"[Laser] Saved "<<data.size()<<" points to "<<filename<<std::endl;
    return true;
}

bool TriangulationLaser::modbus_startConvertedMeasurementCycle(int durationSeconds,int intervalMs,const std::string&filename){
    if(!modbus_connected_){
            std::cerr<<"[Laser] Not connected!"<<std::endl;
            return false;
        }

    std::vector<double>data;
    auto start=std::chrono::steady_clock::now();
    auto duration =std::chrono::seconds(durationSeconds);
    auto interval=std::chrono::milliseconds(intervalMs);

    std::cout<<"[Laser] Starting converted measurement cycle..."<<std::endl;

    while(std::chrono::steady_clock::now()-start<duration){
        uint16_t rawVal;
        if(modbus_readRawMeasurement(rawVal)){
            double mm=convertRawToMm(rawVal);
            data.push_back(mm);
        }
        std::this_thread::sleep_for(interval);
    }

    std::ofstream file(filename);
    if(!file.is_open()){
        std::cerr<<"[Laser] Cannot open file: "<<filename<<std::endl;
        return false;
    }

    for(double v:data){
        file<<std::fixed<<std::setprecision(3)<<v<<'\n';
    }
    file.close();

    std::cout<<"[Laser] Saved "<<data.size()<<" points to "<<filename<<std::endl;
    return true;
}