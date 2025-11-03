#include "TriangulationLaser.hpp"
#include "LaserConfig.hpp"
#include "ConsoleInput.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <iterator>
#include <iomanip>

LaserConfig default_config ={
    .deviceName="RIFTEK RF 603 125/500-232-U",
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

void clearConsole(){
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

void waitForEnter(){
    std::cout<<"\nPress ENTER to continue . . .";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
}

void runLiveMode(TriangulationLaser& laser, int intervalMs);
void runLoggingMode(TriangulationLaser& laser, int intervalMs,const std::string& mode, double targetValue);

int main(){
    std::cout<<"Detected OS: "
#ifdef _WIN32
    <<"Windows"
#else
    <<"Linux"
#endif
    <<std::endl;

    std::cout<<"==========/\tlaser tester\t/=========="<<std::endl;
    std::cout<<"Device:\t\t"<<default_config.deviceName<<std::endl;
    std::cout<<"Port:\t\t"<<default_config.port<<std::endl;
    std::cout<<"Baudrate:\t"<<default_config.baudrate<<std::endl;
    std::cout<<"Parity:\t\t"<<default_config.parity<<std::endl;
    std::cout<<"Data bits:\t"<<default_config.dataBits<<std::endl;
    std::cout<<"Stop bits:\t"<<default_config.stopBits<<std::endl;
    std::cout<<"Slave ID:\t"<<default_config.slaveId<<std::endl;
    std::cout<<"Sensor range:\t"<<default_config.sensorRangeMm<<" mm"<<std::endl;
    std::cout<<"Update every:\t"<<default_config.updateIntervalMs<<" ms"<<std::endl;
    waitForEnter();
    clearConsole();

    std::cout<<"\n\nConnecting to laser module . . ."<<std::endl;

    TriangulationLaser laser(
        default_config.port,
        default_config.baudrate,
        default_config.parity,
        default_config.dataBits,
        default_config.stopBits,
        default_config.slaveId,
        default_config.sensorRangeMm
    );

    if(!laser.connect()){
        std::cerr<<"Failed to connect to the laser!"<<std::endl;
        return 1;
    }

    std::cout<<"Connected successfully!"<<std::endl;

    waitForEnter();

    std::cout<<"Switching to MODBUS. . ."<<std::endl;
    laser.switchToModbus();
    std::cout<<"Switched successfully!"<<std::endl;
    waitForEnter();
    clearConsole();

    laser.setLaserEnabled(false);
    
    std::string command;

    std::cout   <<"Available commands:\n"
                <<" on\t\t\t- enable laser\n"
                <<" off\t\t\t- disable laser\n"
                <<" start\t\t- live display (ESC to stop)\n"
                <<" start-log ontime N\t-log for N seconds\n"
                <<" start-log onsample N\t-log for N samples\n"
                <<" start-log onenter\t\t-log until ESC pressed\n"
                <<" exit\t\t\t-quit\n"
                <<" \nEnter command: ";

    while (std::getline(std::cin,command)){
        std::istringstream iss(command);
        std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                        std::istream_iterator<std::string>{}};
        
        if (tokens.empty()){
            std::cout<<"\nEnter command: ";
            continue;
        }

        std::string cmd=tokens[0];

        if(cmd=="exit"){
            break;
        }else if (cmd=="on"){
            laser.setLaserEnabled(true);
            std::cout<<"Laser ON"<<std::endl;
        }else if (cmd=="off"){
            laser.setLaserEnabled(false);
            std::cout<<"Laser OFF"<<std::endl;
        }else if (cmd=="start"){
            runLiveMode(laser,default_config.updateIntervalMs);
            
        }else if (cmd=="start-log"){
            std::string mode="onenter";
            std::string value="";

            if (tokens.size()>=2){
                mode=tokens[1];
                if(tokens.size()>=3){
                    value=tokens[2];
                }
            }

            if(mode=="ontime"&& !value.empty()){
                double duration=std::stod(value);
                runLoggingMode(laser,default_config.updateIntervalMs,"time",duration);
            }else if(mode=="ontime"&& !value.empty()){
                int samples=std::stoi(value);
                runLoggingMode(laser,default_config.updateIntervalMs,"sample",static_cast<double>(samples));
            }else if(mode=="onenter"&& !value.empty()){
                runLoggingMode(laser,default_config.updateIntervalMs,"enter",0.0);
            }else{
                std::cout<<"Usage: start-log [ontime N | onsample | onenter]"<<std::endl;
            }
        }else{
            std::cout<<"Unknown command. Type 'exit' to quit.\n";
        }
        std::cout<<"\nEnter command: ";
    }
    
    laser.setLaserEnabled(false);
    laser.disconnect();
    std::cout<<"\nGoodbye!"<<std::endl;
    return 0;
}

void runLiveMode(TriangulationLaser& laser, int intervalMs){
    // to be continued
}
void runLoggingMode(TriangulationLaser& laser, int intervalMs,const std::string& mode, double targetValue){
    // to be continued
}