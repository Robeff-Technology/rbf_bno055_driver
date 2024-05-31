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

    void BNO055::initialize(std::vector<uint16_t>& acc_offset, std::vector<uint16_t>& mag_offset, std::vector<uint16_t>& gyro_offset, int16_t acc_radius, int16_t mag_radius) {
        size_t size = 0;
        // SET PAGE ID
        uint8_t page_id = 0;
        size = create_write_command_buffer(BNO055Register::PAGE_ID, reinterpret_cast<const uint8_t*>(&page_id), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("PAGE ID response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PAGE ID response success");

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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CHIP ID response success");

        // SET OPERATION MODE
        uint8_t operation_mode = BNO055OperationMode::CONFIG;
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OPERATION MODE response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OPERATION MODE response success");
        // SET UNITS 
        uint8_t units = 0x02;
        size = create_write_command_buffer(BNO055Register::UNIT_SEL, reinterpret_cast<const uint8_t*>(&units), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("UNIT SEL response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UNIT SEL response success");

        // SET POWER MODE
        uint8_t power_mode = BNO055PowerMode::NORMAL;
        size = create_write_command_buffer(BNO055Register::PWR_MODE, reinterpret_cast<const uint8_t*>(&power_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("POWER MODE response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "POWER MODE response success");
        // SELECT OSCILLATOR
        uint8_t osc_sel = 0x00;
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OSC SEL response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSC SEL response success");

        // SET ACC OFFSET
        size = create_write_command_buffer(BNO055Register::ACCEL_OFFSET_X_LSB_ADDR, create_offset_array(acc_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("ACC OFFSET response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC OFFSET response success");

        // SET MAG OFFSET
        size = create_write_command_buffer(BNO055Register::MAG_OFFSET_X_LSB_ADDR, create_offset_array(mag_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("MAG OFFSET response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MAG OFFSET response success");

        // SET GYRO OFFSET
        size = create_write_command_buffer(BNO055Register::GYRO_OFFSET_X_LSB_ADDR, create_offset_array(gyro_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("GYRO OFFSET response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GYRO OFFSET response success");

        // SET ACC RADIUS
        size = create_write_command_buffer(BNO055Register::ACCEL_RADIUS_LSB_ADDR, create_radius_array(acc_radius), 2, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("ACC RADIUS response error");
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC RADIUS response success");

        // SET MAG RADIUS
        size = create_write_command_buffer(BNO055Register::MAG_RADIUS_LSB_ADDR, create_radius_array(mag_radius), 2, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("MAG RADIUS response error");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MAG RADIUS response success");

        // SET NDOF MODE
        operation_mode = BNO055OperationMode::NDOF;
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        if(check_write_status(receiving_buffer) == false){
            throw BNO055Exception("OPERATION MODE response error");
        } 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDOF MODE response success");

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

    uint8_t* BNO055::create_offset_array(std::vector<uint16_t>& offset){
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

    CalibrationBNO055Status BNO055::read_calib_status() {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::CALIB_STAT, sizeof(CalibrationBNO055Status), sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), sizeof(CalibrationBNO055Status) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055Status)) == false) {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055Status) + 2);
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055Status calib_status;
        std::memcpy(&calib_status, &receiving_buffer[2], sizeof(CalibrationBNO055Status));

        // const uint8_t read_calib_sys = (calib_status.system >> 6) & 0x03;
        // const uint8_t read_calib_gyro = (calib_status.system >> 4) & 0x03;
        // const uint8_t read_calib_acc = (calib_status.system >> 2) & 0x03;
        // const uint8_t read_calib_mag = calib_status.system & 0x03;
        // // const uint8_t read_self_test = data.self_test & 0x0F;
        // // const uint8_t read_sys_status = data.sys_status;
        // // const uint8_t read_sys_error = data.sys_error;


        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_sys: %d", calib_status.system);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_acc: %d", calib_status.acc);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_gyr: %d", calib_status.gyro);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_mag: %d", calib_status.mag);


        if(calib_status.system == 3 && calib_status.acc == 3 && calib_status.gyro == 3 && calib_status.mag == 3){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data: Fully calibrated");
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data: Not fully calibrated");
        }

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data:");
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "self_test: %d", self_tes);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data:");
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sys_status: %d", sys_statu);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration data:");
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sys_error: %d", sys_erro);

        return calib_status;
    }
    
    CalibrationBNO055DataAcc BNO055::read_calib_data_acc() {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::ACCEL_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055DataAcc), sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), sizeof(CalibrationBNO055DataAcc) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055DataAcc)) == false) {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055DataAcc) + 2);
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055DataAcc acc_offset;
        std::memcpy(&acc_offset, &receiving_buffer[2], sizeof(CalibrationBNO055DataAcc));

        uint16_t accel_offset_read_x = ((acc_offset.x_msb << 8) | acc_offset.x_lsb);
        uint16_t accel_offset_read_y = ((acc_offset.y_msb << 8) | acc_offset.y_lsb);
        uint16_t accel_offset_read_z = ((acc_offset.z_msb << 8) | acc_offset.z_lsb);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_offset_x : %d", accel_offset_read_x);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "acc_offset_y : %d", accel_offset_read_y);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "acc_offset_z: %d", accel_offset_read_z);
  
        return acc_offset;
    }

    CalibrationBNO055DataMag BNO055::read_calib_data_mag() {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::MAG_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055DataMag), sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), sizeof(CalibrationBNO055DataMag) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055DataMag)) == false) {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055DataMag) + 2);
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055DataMag mag_offset;
        std::memcpy(&mag_offset, &receiving_buffer[2], sizeof(CalibrationBNO055DataMag));
        
        uint16_t mag_offset_read_x = ((mag_offset.x_msb << 8) | mag_offset.x_lsb);
        uint16_t mag_offset_read_y = ((mag_offset.y_msb << 8) | mag_offset.y_lsb);
        uint16_t mag_offset_read_z = ((mag_offset.z_msb << 8) | mag_offset.z_lsb);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mag_offset_x: %d", mag_offset_read_x);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mag_offset_y: %d", mag_offset_read_y);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mag_offset_z: %d", mag_offset_read_z);


        return mag_offset;
    }

    CalibrationBNO055DataGyro BNO055::read_calib_data_gyro() {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::GYRO_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055DataGyro), sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), sizeof(CalibrationBNO055DataGyro) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055DataGyro)) == false) {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055DataGyro) + 2);
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055DataGyro gyro_offset;
        std::memcpy(&gyro_offset, &receiving_buffer[2], sizeof(CalibrationBNO055DataGyro));

        uint16_t gyro_offset_read_x = ((gyro_offset.x_msb << 8) | gyro_offset.x_lsb);
        uint16_t gyro_offset_read_y = ((gyro_offset.y_msb << 8) | gyro_offset.y_lsb);
        uint16_t gyro_offset_read_z = ((gyro_offset.z_msb << 8) | gyro_offset.z_lsb);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gyro_offset_x: %d", gyro_offset_read_x);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gyro_offset_y: %d", gyro_offset_read_y);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gyro_offset_z: %d", gyro_offset_read_z);

        return gyro_offset;
    }

    









} // namespace rbf_bno055_driver

