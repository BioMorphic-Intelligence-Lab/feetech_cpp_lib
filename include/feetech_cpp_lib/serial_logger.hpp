#ifndef SERIAL_LOGGER_HPP
#define SERIAL_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <string>
#include <vector>
#include <sstream>

class ServoSerialLogger {
public:
    explicit ServoSerialLogger(const std::string& filename)
        : log_file_(filename, std::ios::app) {
        if (!log_file_) {
            throw std::runtime_error("Failed to open log file: " + filename);
        }
    }

    ~ServoSerialLogger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    // Log data sent from SBC to MCU
    void logTx(const std::vector<uint8_t>& data) {
        log("TX", data);
    }

    // Log data received from MCU to SBC
    void logRx(const std::vector<uint8_t>& data) {
        log("RX", data);
    }

private:
    std::ofstream log_file_;

    void log(const std::string& direction, const std::vector<uint8_t>& data) {
        using namespace std::chrono;

        auto now = system_clock::now();
        auto time = system_clock::to_time_t(now);
        auto us = duration_cast<microseconds>(now.time_since_epoch()).count() % 1000000;
        
        std::ostringstream log_entry_;
        log_entry_ << "[" << std::put_time(std::localtime(&time), "%F %T")
                  << "." << std::setw(6) << std::setfill('0') << us << "] "
                  << direction << ": " << bytesToHex(data) << std::endl;
        // Output to log file
        if (log_file_.is_open()) {
            log_file_ << log_entry_.str();
            log_file_
                .flush(); // Ensure immediate write to file
        }
    }

    std::string bytesToHex(const std::vector<uint8_t>& data) {
        std::ostringstream oss;
        for (auto byte : data) {
            oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        return oss.str();
    }
};

#endif // SERIAL_LOGGER_HPP
