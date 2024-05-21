#include "rbf_bno055_driver/bno055.h"

#include <cstring>
#include <iostream>


// Path: rbf_bno055_driver/src/bno055.cpp
// Compare this snippet from rbf_bno055_driver/include/rbf_bno055_driver/bno055.h:

// Constructor
namespace rbf_bno055_driver {

    BNO055::BNO055(const std::string& port_name) {
        serial_port_.set_port_name(port_name.c_str());
        serial_port_.open();
        serial_port_.configure(115200, 8, 'N', 1);
    }

    void BNO055::initialize(std::vector<int16_t>& acc_offset, std::vector<int16_t>& mag_offset, std::vector<int16_t>& gyro_offset, int16_t acc_radius, int16_t mag_radius) {
        size_t size = 0;
        // SET PAGE ID
        size = create_read_command_buffer(BNO055Register::CHIP_ID, 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 3);
        if(check_receive_command(receiving_buffer, 1) == false){
            throw BNO055Exception("CHIP ID response error");
        }
        else{
            if(receiving_buffer[0] != BNO055MessageType::READ_RESPONSE || receiving_buffer[2] != BNO055ChipID::BNO_CHIP_ID){
                throw BNO055Exception("Error reading chip id");
            }
        }

        // SET OPERATION MODE
        uint8_t operation_mode = BNO055OperationMode::CONFIG;
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OPERATION MODE response error");
        }

        // SYS RESET
        uint8_t osc_sel = 0x20;
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        // serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        // if(check_write_status(receiving_buffer) == false){
        //     throw BNO055Exception("OSC SEL response error");
        // }
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // SET PAGE ID
        size = create_read_command_buffer(BNO055Register::CHIP_ID, 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 3);
        if(check_receive_command(receiving_buffer, 1) == false){
            throw BNO055Exception("CHIP ID response error");
        }
        else{
            if(receiving_buffer[0] != BNO055MessageType::READ_RESPONSE || receiving_buffer[2] != BNO055ChipID::BNO_CHIP_ID){
                throw BNO055Exception("Error reading chip id");
            }
        }

        // SET POWER MODE
        uint8_t power_mode = BNO055PowerMode::NORMAL;
        size = create_write_command_buffer(BNO055Register::PWR_MODE, reinterpret_cast<const uint8_t*>(&power_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("POWER MODE response error");
        }

        uint8_t page_id = 0;
        size = create_write_command_buffer(BNO055Register::PAGE_ID, reinterpret_cast<const uint8_t*>(&page_id), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("PAGE ID response error");
        }

        // SYS RESET
        osc_sel = 0x00;
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        // serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        // if(check_write_status(receiving_buffer) == false){
        //     throw BNO055Exception("OSC SEL response error");
        // }
        rclcpp::sleep_for(std::chrono::milliseconds(50));

        // SET OPERATION MODE
       operation_mode = BNO055OperationMode::NDOF;
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OPERATION MODE response error");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

                // SYS RESET
        osc_sel = 0x80;
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        // serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        // if(check_write_status(receiving_buffer) == false){
        //     throw BNO055Exception("OSC SEL response error");
        // }

        rclcpp::sleep_for(std::chrono::milliseconds(500));

    }


    size_t BNO055::create_write_command_buffer(BNO055Register reg_adr, const uint8_t* data, size_t data_len, uint8_t* buffer) {
        size_t index = 0;
        buffer[index++] = BNO055MessageType::REGISTER_COMMAND;
        buffer[index++] = BNO055RegisterCommand::WRITE;
        buffer[index++] = reg_adr;
        buffer[index++] = data_len;
        if(data_len > 0){
            for(size_t i = 0; i < data_len; i++){
                buffer[index++] = data[i];
            }
        }
        return index;
    }

    size_t BNO055::create_write_command_buffer(BNO055ConfigRegister reg_adr, const uint8_t* data, size_t data_len, uint8_t* buffer) {
        size_t index = 0;
        buffer[index++] = BNO055MessageType::REGISTER_COMMAND;
        buffer[index++] = BNO055RegisterCommand::WRITE;
        buffer[index++] = reg_adr;
        buffer[index++] = data_len;
        if(data_len > 0){
            for(size_t i = 0; i < data_len; i++){
                buffer[index++] = data[i];
            }
        }
        return index;
    }

    size_t BNO055::create_read_command_buffer(BNO055Register reg_adr, size_t data_len, uint8_t* buffer) {
        size_t index = 0;
        buffer[index++] = BNO055MessageType::REGISTER_COMMAND;
        buffer[index++] = BNO055RegisterCommand::READ;
        buffer[index++] = reg_adr;
        buffer[index++] = data_len;
        return index;
    }

    bool BNO055::check_receive_command(const uint8_t* buffer, size_t data_len){
        if(buffer[0] != BNO055MessageType::READ_RESPONSE){
            return false;
        }
        if(buffer[1] != data_len){
            return false;
        }
        return true;
    }

    bool BNO055::check_write_status(const uint8_t* buffer){
        if(buffer[0] != BNO055MessageType::RESPONSE_STATUS){
            return false;
        }
        if(buffer[1] != 0x01){
            return false;
        }
        return true;
    }

    uint8_t* BNO055::create_offset_array(std::vector<int16_t>& offset){
        uint8_t *offset_array = new uint8_t[offset.size()*2];
        for(size_t i = 0; i < offset.size(); i++){
            offset_array[i*2] = offset[i] & 0xFF;
            offset_array[i*2+1] = (offset[i] >> 8) & 0xFF;
        }
        return offset_array;
    }

    uint8_t* BNO055::create_radius_array(int16_t radius){
        uint8_t *radius_array = new uint8_t[sizeof(int16_t)];
        radius_array[0] = radius & 0xFF;
        radius_array[1] = (radius >> 8) & 0xFF;
        return radius_array;
    }

    RawBNO055Data BNO055::read_raw_data(){
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::ACC_DATA_X_LSB, sizeof(RawBNO055Data), sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), sizeof(RawBNO055Data) + 2);
        if(check_receive_command(receiving_buffer, 45) == false){
            memset(receiving_buffer, 0, sizeof(RawBNO055Data) + 2);
            throw BNO055Exception("ACC DATA response error");
        }
        RawBNO055Data data;
        std::memcpy(&data, &receiving_buffer[2], sizeof(RawBNO055Data));
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data:");
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data x: %d", data.acc_x);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data y: %d", data.acc_y);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data z: %d", data.acc_z);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data gyro x: %d", data.gyro_x);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data gyro y: %d", data.gyro_y);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "raw data gyro z: %d", data.gyro_z);
        return data;
    }
    
    CalibrationBNO055Data BNO055::read_calib_data() {

        
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::CALIB_STAT, 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 3);
        if (check_receive_command(receiving_buffer, 1) == false) {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055Data) + 2);
            throw BNO055Exception("CALIB DATA response error");
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CALIB STAT: %d %d %d ", receiving_buffer[0], receiving_buffer[1], receiving_buffer[2]);
        }
         CalibrationBNO055Data data;
        // std::memcpy(&data, &receiving_buffer[2], sizeof(CalibrationBNO055Data));
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data:");
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_offset_x: %d", data.accel_offset_x);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_offset_y: %d", data.accel_offset_y);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_offset_z: %d", data.accel_offset_z);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mag_offset_x: %d", data.mag_offset_x);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mag_offset_y: %d", data.mag_offset_y);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mag_offset_z: %d", data.mag_offset_z);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gyro_offset_x: %d", data.gyro_offset_x);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gyro_offset_y: %d", data.gyro_offset_y);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gyro_offset_z: %d", data.gyro_offset_z);
        
        return data;
    }




} // namespace rbf_bno055_driver

