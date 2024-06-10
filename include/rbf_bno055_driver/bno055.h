#ifndef BNO055_H
#define BNO055_H

#include <rbf_bno055_driver/serial_port.h>
#include "rbf_bno055_driver/bno055_reg.h"
#include "rbf_bno055_driver/bno055_struct.h"
#include <string>
#include <exception>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace rbf_bno055_driver {
class BNO055 {
public:
    class BNO055Exception : public std::exception {
        public:
            explicit BNO055Exception(const std::string& message)
                : message_(message) {}

            virtual const char* what() const noexcept override {
                return message_.c_str();
            }

        private:
            std::string message_;
    };


    explicit BNO055(const std::string& port_name);
    ~BNO055() {
        serial_port_.close();
    }

    void initialize();
    void initialize_calib(std::vector<uint16_t>& acc_offset, std::vector<uint16_t>& mag_offset, std::vector<uint16_t>& gyro_offset, int16_t acc_radius, int16_t mag_radius);

    RawBNO055Data read_raw_data();
    CalibrationBNO055DataAcc read_calib_data_acc();
    CalibrationBNO055DataMag read_calib_data_mag();
    CalibrationBNO055DataGyro read_calib_data_gyro();
    CalibrationBNO055Status read_calib_status();
    
   
    
private:
    // Add private member variables as needed
    SerialPort serial_port_;
    static size_t create_write_command_buffer(BNO055Register reg_adr, const uint8_t* data, size_t data_len, uint8_t* buffer);
    static size_t create_write_command_buffer(BNO055ConfigRegister reg_adr, const uint8_t* data, size_t data_len, uint8_t* buffer);
    static size_t create_read_command_buffer(BNO055Register reg_adr, size_t data_len, uint8_t* buffer);
    static bool check_receive_command(const uint8_t* buffer, size_t data_len);
    static bool check_write_status(const uint8_t* buffer);
    static uint8_t* create_offset_array(std::vector<uint16_t>& offset);
    static uint8_t* create_radius_array(int16_t radius);
    uint8_t sending_buffer[128];
    uint8_t receiving_buffer[128];
};
}; // namespace rbf_bno055_driver
#endif // BNO055_H
