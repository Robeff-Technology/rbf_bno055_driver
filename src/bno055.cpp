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
        uint8_t page_id = 0;
        size = create_write_command_buffer(BNO055Register::PAGE_ID, reinterpret_cast<const uint8_t*>(&page_id), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("PAGE ID response error");
        }
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

        // SET UNITS 
        uint8_t units = 0x02;
        size = create_write_command_buffer(BNO055Register::UNIT_SEL, reinterpret_cast<const uint8_t*>(&units), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("UNIT SEL response error");
        }

        // SET POWER MODE
        uint8_t power_mode = BNO055PowerMode::NORMAL;
        size = create_write_command_buffer(BNO055Register::PWR_MODE, reinterpret_cast<const uint8_t*>(&power_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("POWER MODE response error");
        }

        // SELECT OSCILLATOR
        uint8_t osc_sel = 0x00;
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OSC SEL response error");
        }



        // SET ACC OFFSET
        size = create_write_command_buffer(BNO055Register::ACCEL_OFFSET_X_LSB_ADDR, create_offset_array(acc_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("ACC OFFSET response error");
        }

        // SET MAG OFFSET
        size = create_write_command_buffer(BNO055Register::MAG_OFFSET_X_LSB_ADDR, create_offset_array(mag_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("MAG OFFSET response error");
        }

        // SET GYRO OFFSET
        size = create_write_command_buffer(BNO055Register::GYRO_OFFSET_X_LSB_ADDR, create_offset_array(gyro_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("GYRO OFFSET response error");
        }

        // SET ACC RADIUS
        size = create_write_command_buffer(BNO055Register::ACCEL_RADIUS_LSB_ADDR, create_radius_array(acc_radius), 2, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("ACC RADIUS response error");
        }

        // SET MAG RADIUS
        size = create_write_command_buffer(BNO055Register::MAG_RADIUS_LSB_ADDR, create_radius_array(mag_radius), 2, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("MAG RADIUS response error");
        }

        // SET OPERATION MODE
        operation_mode = BNO055OperationMode::NDOF;
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OPERATION MODE response error");
        }

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
        return data;
    }
    
    CalibrationBNO055Data BNO055::read_calib_data() {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::ACCEL_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055Data), sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), sizeof(CalibrationBNO055Data) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055Data)) == false) {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055Data) + 2);
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055Data data;
        std::memcpy(&data, &receiving_buffer[2], sizeof(CalibrationBNO055Data));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data:");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_offset_x: %d", data.accel_offset_x);
        return data;
    }




} // namespace rbf_bno055_driver

