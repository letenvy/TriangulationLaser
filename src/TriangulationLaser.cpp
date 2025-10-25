#include "TriangulationLaser.hpp"
#include <modbus/modbus.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>

constexpr int LASER_CONTROL_REG =0x000A;
constexpr int MEASUREMENT_REG =5;

bool TriangulationLaser::switchToModbus(){
    try{
        boost::asio::io_context io;
        boost::asio::serial_port serial(io);
        
        std::string port=portName_;
    #ifdef _WIN32
        if(!port.empty()&&port.find("COM")==0&&port.size()>4){
            port="\\\\.\\"+port;
        }
    #endif
        serial.open(port);

        serial.set_option(boost::asio::serial_port::baud_rate(baudrate_));
        serial.set_option(boost::asio::serial_port::character_size(dataBits_));
        serial.set_option(boost::asio::serial_port::parity(
            parity_=='E'?boost::asio::serial_port::parity::even:
            parity_=='O'?boost::asio::serial_port::parity::odd:
            boost::asio::serial_port::parity::none
        ));
        serial.set_option(boost::asio::serial_port_base::stop_bits(
            stopBits_==1?boost::asio::serial_port_base::stop_bits::one:
            stopBits_==2?boost::asio::serial_port_base::stop_bits::two:
            boost::asio::serial_port_base::stop_bits::one
        ));

        std::cout<<"[Switch] Sending RIFTEK -> MODBUS command to "<<portName_<<"..."<<std::endl;

        std::vector<uint8_t>command={0x01,0x03,0x8A,0x02};
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
    ,connected_(false)
{
}

TriangulationLaser::~TriangulationLaser(){
    disconnect();
}

std::string TriangulationLaser::normalizePortName(const std::string& port) const{
    if(!port.empty()&&port.find("COM")==0&&port.size()>4)
        return"\\\\.\\"+port;
    return port;
}

bool TriangulationLaser::connect(){
    if(connected_) return true;
    
    std::string fullPort=normalizePortName(portName_);
    ctx_=modbus_new_rtu(fullPort.c_str(),baudrate_,parity_,dataBits_,stopBits_);
    if(!ctx_){
        handleError("modbus_new_rtu");
        return false;
    }

    if(modbus_set_slave(ctx_,slaveId_)==-1){
        handleError("modbus_set_slave");
        modbus_free(ctx_);
        ctx_=nullptr;
        return false;
    }

    modbus_rtu_set_serial_mode(ctx_,MODBUS_RTU_RS485);
    struct timeval timeout={3,0};
    modbus_set_response_timeout(ctx_,timeout.tv_sec,timeout.tv_usec);

    if (modbus_connect(ctx_)==-1){
        handleError("modbus_connect");
        modbus_free(ctx_);
        ctx_=nullptr;
        return false;
    }

    connected_=true;
    std::cout<<"[Laser] Connected to "<<portName_<<std::endl;
    return true;
}

void TriangulationLaser::disconnect(){
    if(ctx_){
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_=nullptr;
        connected_=false;
        std::cout<<"[Laser] Disconnected"<<std::endl;
    }
}

bool TriangulationLaser::isConnected() const{
    return connected_;
}

bool TriangulationLaser::setLaserEnabled(bool enable){
    if(!connected_){
        std::cerr<<"[Laser] Not connected!"<<std::endl;
        return false;
    }
    uint16_t state = enable ? 1:0;
    if(modbus_write_register(ctx_,LASER_CONTROL_REG,state)!=1){
        handleError("modbus_write_register");
        return false;
    }
    std::cout<<"[Laser] "<<(enable?"ON":"OFF")<<std::endl;
    return true;
}

bool TriangulationLaser::readRawMeasurement(uint16_t& rawValue){
    if(!connected_){
        std::cerr<<"[Laser] Not connected"<<std::endl;
        return false;
    }

    if(modbus_read_input_registers(ctx_,MEASUREMENT_REG,1,&rawValue)!=1){
        handleError("modbus_read_input_registers");
        return false;
    }
    return true;
}

bool TriangulationLaser::startRawMeasurementCycle(int durationSeconds,int intervalMs,const std::string& filename){
    if(!connected_){
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
        if(readRawMeasurement(rawVal)){
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

bool TriangulationLaser::startConvertedMeasurementCycle(int durationSeconds,int intervalMs,const std::string&filename){
    if(!connected_){
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
        if(readRawMeasurement(rawVal)){
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