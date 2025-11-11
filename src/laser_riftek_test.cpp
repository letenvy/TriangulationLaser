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
#include <limits>
#include <fstream>
#include <algorithm>

#include <nlohmann/json.hpp>

LaserConfig default_config ={
    .deviceName="RIFTEK RF 603 125/500-232-U",
#ifdef _WIN32
    .port = "COM5",
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
void runLiveLoggingMode(TriangulationLaser& laser, int intervalMs,const std::string& mode, double targetValue);

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

    TriangulationLaser laser(
        default_config.port,
        default_config.baudrate,
        default_config.parity,
        default_config.dataBits,
        default_config.stopBits,
        default_config.slaveId,
        default_config.sensorRangeMm
    );

    std::cout << "\n\nConnecting to laser module . . ." << std::endl;

    if (!laser.connect()) {
        std::cerr << "Failed to connect to the laser!" << std::endl;
        waitForEnter();
        return 1;
    }

    std::cout << "Connected successfully in " << (laser.getMode() == LaserMode::Riftek ? "RIFTEK" : "MODBUS") << " mode!" << std::endl;
    waitForEnter();

    std::vector<uint8_t> idResponse;
    if (laser.riftek_requestId(&idResponse) && idResponse.size() >= 4) {
        std::cout << "Device ID Response: ";
        for (auto b : idResponse) {
            std::cout << std::hex << static_cast<int>(b) << " ";
        }
        std::cout << std::dec << std::endl;
    } else {
        std::cerr << "Failed to read device ID." << std::endl;
    }

    std::cout << "Turning laser OFF . . ." << std::endl;
    if (!laser.riftek_writeParameter(static_cast<uint8_t>(TriangulationLaser::RiftekParameter::LASER_ENABLE), 0)) {
        std::cerr << "Failed to disable laser!" << std::endl;
        waitForEnter();
        return 1;
    }
    std::cout << "Laser is now OFF." << std::endl;
    waitForEnter();
    clearConsole();

    std::string command;

    std::cout << "Available commands:\n"
                 " on\t\t\t- enable laser\n"
                 " off\t\t\t- disable laser\n"
                 " start\t\t- live display (ESC to stop)\n"
                 " start-log ontime N\t- log for N seconds\n"
                 " start-log onsample N\t- log for N samples\n"
                 " start-log onenter\t- log until ESC pressed\n"
                 " exit\t\t\t- quit\n"
                 " \nEnter command: ";
    

    std::cout << "\nTurning laser OFF before exit . . .\n";
    laser.riftek_writeParameter(static_cast<uint8_t>(TriangulationLaser::RiftekParameter::LASER_ENABLE), 0);
    laser.disconnect();
    std::cout << "\nGoodbye!" << std::endl;
    return 0;
}

struct MeasurementData {
    uint16_t raw = 0;
    double mm = 0.0;
    bool success = false;
    std::chrono::steady_clock::time_point measurementStart;
    std::chrono::steady_clock::time_point measurementEnd;
    std::chrono::system_clock::time_point globalTime;
};

MeasurementData performMeasurement(TriangulationLaser& laser) {
    MeasurementData data;

    data.measurementStart = std::chrono::steady_clock::now();

    data.success = laser.riftek_requestResult(data.raw);
    if (data.success) {
        data.mm = laser.convertRawToMm(data.raw);
    }

    data.measurementEnd = std::chrono::steady_clock::now();
    data.globalTime = std::chrono::system_clock::now();

    return data;
}

void runLiveMode(TriangulationLaser& laser, int intervalMs) {
    clearConsole();
    std::cout << " Live mode\t(press ESC to stop)\n" << std::endl;

    ConsoleInput input;
    bool running = true;
    auto lastMeasurementStart = std::chrono::steady_clock::now();

    while (running && laser.isConnected()) {
        if (input.isEscapePressed()) {
            running = false;
            break;
        }

        MeasurementData data = performMeasurement(laser);

        double dt = std::chrono::duration<double>(data.measurementStart - lastMeasurementStart).count();
        double hz = (dt > 0) ? 1.0 / dt : 0.0;
        lastMeasurementStart = data.measurementStart;

        if (data.success) {
            std::cout << data.mm << " mm\r|\r" << std::fixed << std::setprecision(1) << hz << " Hz\r";
        } else {
            std::cout << "Measurement failed!\r";
        }
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    std::cout << "\nReturning" << std::endl;
    waitForEnter();
    clearConsole();
}

void runLiveLoggingMode(TriangulationLaser& laser, int intervalMs, const std::string& mode, double targetValue) {
    clearConsole();
    std::cout << " Live logging mode\t(press ESC to stop)" << std::endl;

    ConsoleInput input;
    bool running = true;
    int sampleCount = 0;
    auto startTime = std::chrono::steady_clock::now();

    auto logTime = std::chrono::system_clock::now();
    std::time_t logTimeT = std::chrono::system_clock::to_time_t(logTime);
    std::stringstream filenameSS;
    filenameSS << "laser_log_" << std::put_time(std::localtime(&logTimeT), "%Y%m%d_%H%M%S") << ".json";
    std::string filename = filenameSS.str();

    std::ofstream logFile(filename);
    if (!logFile.is_open()) {
        std::cerr << "Failed to create log file: " << filename << std::endl;
        return;
    }

    std::cout << "Logging to: " << filename << '\n' << std::endl;

    auto lastMeasurementStart = std::chrono::steady_clock::now();

    logFile << "[\n";
    bool firstEntry = true;

    while (running && laser.isConnected()) {
        if (input.isEscapePressed()) {
            std::cout << "\nLogging stopped by user" << std::endl;
            running = false;
            break;
        }

        if (mode == "time") {
            auto currentTime = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(currentTime - startTime).count();
            if (elapsed >= targetValue) {
                std::cout << "\nLogging time completed" << std::endl;
                break;
            }
        } else if (mode == "sample") {
            if (sampleCount >= static_cast<int>(targetValue)) {
                std::cout << "\nSample count completed" << std::endl;
                break;
            }
        }

        MeasurementData data = performMeasurement(laser);

        double dt = std::chrono::duration<double>(data.measurementStart - lastMeasurementStart).count();
        double hz = (dt > 0) ? 1.0 / dt : 0.0;
        lastMeasurementStart = data.measurementStart;

        if (data.success) {
            std::cout << "Sample " << sampleCount + 1 << ": " << data.mm << " mm | "
                      << std::fixed << std::setprecision(1) << hz << " Hz\r";
        } else {
            std::cout << "Sample " << sampleCount + 1 << " Measurement failed!\r";
        }
        std::cout.flush();

        nlohmann::json jsonEntry;
        jsonEntry["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     data.globalTime.time_since_epoch()).count();
        jsonEntry["raw"] = data.raw;
        jsonEntry["mm"] = data.mm;

        if (!firstEntry) {
            logFile << ",\n";
        }
        logFile << jsonEntry.dump(4);
        logFile.flush();

        firstEntry = false;
        sampleCount++;
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    logFile << "\n]\n";
    logFile.close();

    std::cout << "\nLog saved to: " << filename << std::endl;
    std::cout << "Returning" << std::endl;
    waitForEnter();
    clearConsole();
}
