#include "TriangulationLaser.hpp"
#include "LaserConfig.hpp"
#include "ConsoleInput.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <fstream>
#include <nlohmann/json.hpp>

void clearConsole() {
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

void waitForEnter() {
    std::cout << "\nPress ENTER to continue . . .";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int main() {
    LaserConfig config = {
        .deviceName = "RIFTEK RF 603 125/500-232-U",
#ifdef _WIN32
        .port = "COM5",
#else
        .port = "/dev/ttyUSB0",
#endif
        .baudrate = 9600,
        .parity = 'E',
        .dataBits = 8,
        .stopBits = 1,
        .sensorRangeMm = 500.0,
        .baseRangeMm = 125.0,
        .updateIntervalMs = 100
    };

    std::cout << "==========/\tlaser tester\t/==========" << std::endl;
    std::cout << "Device:\t\t" << config.deviceName << std::endl;
    std::cout << "Port:\t\t" << config.port << std::endl;
    std::cout << "Baudrate:\t" << config.baudrate << std::endl;
    std::cout << "Parity:\t\t" << config.parity << std::endl;
    std::cout << "Data bits:\t" << config.dataBits << std::endl;
    std::cout << "Stop bits:\t" << config.stopBits << std::endl;
    std::cout << "Sensor range:\t" << config.sensorRangeMm << " mm" << std::endl;
    std::cout << "Base range:\t" << config.baseRangeMm << " mm" << std::endl;
    std::cout << "Update every:\t" << config.updateIntervalMs << " ms" << std::endl;
    waitForEnter();

    TriangulationLaser laser(config.port, config.baudrate, config.parity,
                             config.dataBits, config.stopBits,
                             config.sensorRangeMm, config.baseRangeMm);

    std::cout << "\n\nConnecting to laser module . . ." << std::endl;

    if (!laser.connect()) {
        std::cerr << "Failed to connect to the laser!" << std::endl;
        waitForEnter();
        return 1;
    }
    std::cout << "Connected successfully in " << (laser.getMode() == LaserMode::Riftek ? "RIFTEK" : "UNKNOWN") << " mode!" << std::endl;
    waitForEnter();

    auto rf = laser.riftek();
    rf.writeParameter(0x00, 0x01);

    while (true) {
        clearConsole();
        std::cout << "\n==========/\tRIFTEK TESTER\t/==========" << std::endl;
        std::cout << "1. Send custom command" << std::endl;
        std::cout << "2. Request device ID" << std::endl;
        std::cout << "3. Live result display" << std::endl;
        std::cout << "4. Live result + JSON logging" << std::endl;
        std::cout << "5. Exit" << std::endl;
        std::cout << "\nEnter command number: ";

        int choice;
        std::cin >> choice;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // ← добавить


        if (choice == 1) {
            std::cout << "Enter command bytes in hex (e.g. 01 86 for REQUEST_RESULT): ";
            std::string line;
            std::cin.ignore();
            std::getline(std::cin, line);

            std::istringstream iss(line);
            std::vector<uint8_t> cmd;
            std::string hex;

            while (iss >> hex) {
                uint8_t b = std::stoi(hex, nullptr, 16);
                cmd.push_back(b);
            }

            if (!cmd.empty()) {
                rf.sendRequest(cmd);
                std::cout << "Command sent. Waiting for response (2 sec timeout)...\n";
                if (auto resp = rf.listenAnswer(2000, 20)) {
                    std::cout << "Response: ";
                    for (auto b : *resp) {
                        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)b << " ";
                    }
                    std::cout << std::dec << std::endl;
                } else {
                    std::cout << "No response or timeout.\n";
                }
            } else {
                std::cout << "Invalid command.\n";
            }
            waitForEnter();
        }
        else if (choice == 2) {
            if (auto id = rf.requestID()) {
                std::cout << "Device ID Response: ";
                for (auto b : *id) {
                    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)b << " ";
                }
                std::cout << std::dec << std::endl;

                if (id->size() >= 8) {
                    uint8_t device_type = (*id)[0];
                    uint8_t fw_version = (*id)[1];
                    uint16_t serial = (static_cast<uint16_t>((*id)[3]) << 8) | (*id)[2];
                    uint16_t base_dist = (static_cast<uint16_t>((*id)[5]) << 8) | (*id)[4];
                    uint16_t range = (static_cast<uint16_t>((*id)[7]) << 8) | (*id)[6];

                    std::cout << "Device Type: 0x" << std::hex << (int)device_type << std::dec << std::endl;
                    std::cout << "FW Version: " << (int)fw_version << std::endl;
                    std::cout << "Serial: " << serial << std::endl;
                    std::cout << "Base Distance: " << base_dist << " mm" << std::endl;
                    std::cout << "Range: " << range << " mm" << std::endl;
                }
            } else {
                std::cout << "Failed to read ID\n";
            }
            waitForEnter();
        }
        else if (choice == 3) {
            ConsoleInput input;
            bool running = true;
            auto lastMeasurementStart = std::chrono::steady_clock::now();

            clearConsole();
            std::cout << " Live result display (press ESC to stop)\n" << std::endl;

            while (running) {
                if (input.isEscapePressed()) {
                    running = false;
                    break;
                }

                if (auto raw = rf.requestResult()) {
                    double mm = laser.convertRawToMm(*raw);

                    auto now = std::chrono::steady_clock::now();
                    double dt = std::chrono::duration<double>(now - lastMeasurementStart).count();
                    double hz = (dt > 0) ? 1.0 / dt : 0.0;
                    lastMeasurementStart = now;

                    std::cout << "\rRaw: " << *raw << " | Hex: 0x" << std::hex << *raw << std::dec
                            << " | Distance: ";
                    if (*raw != 0 && *raw != 16384) {
                        std::cout << std::fixed << std::setprecision(2) << mm << " mm";
                    } else {
                        std::cout << "null";
                    }
                    std::cout << " | " << std::fixed << std::setprecision(1) << hz << " Hz";


                    std::cout.flush();
                } else {
                    std::cout << "\rMeasurement failed!";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(config.updateIntervalMs));
            }

            std::cout << "\n\nReturning to menu..." << std::endl;
            waitForEnter();
        }
        else if (choice == 4) {
            std::cout << "Enter JSON filename: ";
            std::string filename;
            std::cin >> filename;

            std::ofstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Cannot open file: " << filename << std::endl;
                waitForEnter();
                continue;
            }

            nlohmann::json jsonLog = nlohmann::json::array();

            ConsoleInput input;
            bool running = true;
            auto lastMeasurementStart = std::chrono::steady_clock::now();

            clearConsole();
            std::cout << " Live result + JSON logging (press ESC to stop)\n" << std::endl;

            while (running) {
                if (input.isEscapePressed()) {
                    running = false;
                    break;
                }

                if (auto raw = rf.requestResult()) {
                    double mm = laser.convertRawToMm(*raw);

                    auto now = std::chrono::steady_clock::now();
                    double dt = std::chrono::duration<double>(now - lastMeasurementStart).count();
                    double hz = (dt > 0) ? 1.0 / dt : 0.0;
                    lastMeasurementStart = now;

                    // === JSON запись с временем ===
                    nlohmann::json sample;
                    auto now_sys = std::chrono::system_clock::now();
                    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_sys.time_since_epoch()).count();

                    auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);
                    std::stringstream ss;
                    ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
                    std::string timestamp_str = ss.str();

                    sample["timestamp_ms"] = timestamp_ms;
                    sample["timestamp"] = timestamp_str;
                    sample["raw"] = *raw;
                    sample["mm"] = mm;
                    sample["hz"] = hz;

                    jsonLog.push_back(sample);
                    // ===

                    // Вывод в консоль
                    std::cout << "\rRaw: " << *raw << " | Hex: 0x" << std::hex << *raw << std::dec
                            << " | Distance: ";
                    if (*raw != 0 && *raw != 16384) {
                        std::cout << std::fixed << std::setprecision(2) << mm << " mm";
                    } else {
                        std::cout << "null";
                    }
                    std::cout << " | " << std::fixed << std::setprecision(1) << hz << " Hz";

                    std::cout.flush();
                } else {
                    std::cout << "\rMeasurement failed!";
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(config.updateIntervalMs));
            }

            file << jsonLog.dump(4);
            file.close();

            std::cout << "\n\nData saved to " << filename << ". Returning to menu..." << std::endl;
            waitForEnter();
        }
        else if (choice == 5) {
            break;
        }
        else {
            std::cout << "Invalid command. Press ENTER to continue.\n";
            waitForEnter();
        }
    }

    rf.writeParameter(0x00, 0x00); // выключить перед выходом
    laser.disconnect();
    std::cout << "\nGoodbye!" << std::endl;
    return 0;
}