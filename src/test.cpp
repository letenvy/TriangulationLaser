#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <iomanip>
#include <optional>

int main() {
    boost::asio::io_context io;
    boost::asio::serial_port port(io, "COM5");

    port.set_option(boost::asio::serial_port::baud_rate(9600));
    port.set_option(boost::asio::serial_port::character_size(8));
    port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::even));
    port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));

    std::cout << "Starting pseudo-stream via REQUEST_RESULT with range check...\n";

    auto last_time = std::chrono::steady_clock::now();
    int count = 0;

    uint8_t req[] = {0x01, 0x86}; // REQUEST_RESULT

    while (true) {
        boost::system::error_code ec;

        // Отправляем запрос
        port.write_some(boost::asio::buffer(req), ec);
        if (ec) {
            std::cerr << "Write error: " << ec.message() << std::endl;
            break;
        }

        // Читаем ответ
        std::vector<uint8_t> resp(4);
        auto len = port.read_some(boost::asio::buffer(resp), ec);
        if (ec) {
            std::cerr << "Read error: " << ec.message() << std::endl;
            break;
        }

        if (len != 4) {
            std::cerr << "Unexpected response length: " << len << std::endl;
            continue;
        }

        // Разбор: 4 байта = 2 байта данных
        uint8_t low1 = resp[0] & 0x0F;
        uint8_t high1 = (resp[1] & 0x0F) << 4;
        uint8_t low2 = resp[2] & 0x0F;
        uint8_t high2 = (resp[3] & 0x0F) << 4;

        uint16_t raw = (low1 | high1) | ((low2 | high2) << 8);

        // Параметры диапазона
        constexpr double MIN_DISTANCE_MM = 125.0;
        constexpr double MAX_DISTANCE_MM = 500.0;
        constexpr double FULL_SCALE = 16384.0;

        std::optional<double> valid_mm = std::nullopt;

        if (raw != 0 && raw != 16384) {
            double mm = MIN_DISTANCE_MM + static_cast<double>(raw) * (MAX_DISTANCE_MM - MIN_DISTANCE_MM) / FULL_SCALE;
            valid_mm = mm;
        }

        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        double hz = (dt > 0) ? 1.0 / dt : 0.0;

        std::cout << "\rRaw: 0x" << std::hex << raw << std::dec
                  << " | ";

        if (valid_mm.has_value()) {
            std::cout << std::fixed << std::setprecision(2) << valid_mm.value() << " mm";
        } else {
            std::cout << "null";
        }

        std::cout << " | " << std::fixed << std::setprecision(1) << hz << " Hz";

        std::cout.flush();

        count++;
        if (count > 10000) break; // для теста
    }

    std::cout << "\nDone.\n";

    return 0;
}