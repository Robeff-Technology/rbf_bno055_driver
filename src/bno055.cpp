#include "rbf_bno055_driver/bno055.h"

#include <cstring>
#include <iostream>

// Path: rbf_bno055_driver/src/bno055.cpp
// Compare this snippet from rbf_bno055_driver/include/rbf_bno055_driver/bno055.h:

// Constructor
namespace rbf_bno055_driver
{

    /**
     * @brief Constructor for BNO055 class.
     * @param port_name The name of the serial port to be used.
     */

    BNO055::BNO055(const std::string &port_name)
    {
        serial_port_.set_port_name(port_name.c_str());
        serial_port_.open();
        serial_port_.configure(115200, 8, 'N', 1);
    }

    void BNO055::retry_until_success(std::function<bool()> action, const int timeout_seconds = 180, const int retry_interval_ms = 1)
    {
        auto start_time = std::chrono::steady_clock::now();

        while (true)
        {
            if (action())
            {
                return; // Eğer işlem başarılı olursa çıkış yap
            }

            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

            if (elapsed_time >= timeout_seconds)
            {
                throw BNO055Exception("Operation timeout after multiple attempts");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms)); // Yeniden denemeden önce kısa bir süre bekle
        }
    }

    /**
     * @brief Initializes the BNO055 sensor.
     */

    void BNO055::initialize()
    {
        size_t size = 0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BNO055 initialization started.");

        // CHIP ID Doğrulama
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CHIP ID sended.");
        size = create_read_command_buffer(BNO055Register::CHIP_ID, 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 3);

        if (check_receive_command(receiving_buffer, 1)) {
            if (receiving_buffer[0] == BNO055MessageType::READ_RESPONSE && receiving_buffer[2] == BNO055ChipID::BNO_CHIP_ID) {
                return true;
            }
        }
        return false; }, 180, 100); // 180 saniye boyunca, her 100 ms'de bir dene
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CHIP ID response success");

        // SET OPERATION MODE
        uint8_t operation_mode = BNO055OperationMode::CONFIG;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OPERATION MODE sended.");
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); }, 180, 100);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OPERATION MODE response success");

        // SET POWER MODE
        uint8_t power_mode = BNO055PowerMode::NORMAL;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "POWER MODE sended.");
        size = create_write_command_buffer(BNO055Register::PWR_MODE, reinterpret_cast<const uint8_t*>(&power_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); }, 180, 100);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "POWER MODE response success");

        // SET PAGE ID
        uint8_t page_id = 0;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PAGE ID sended.");
        size = create_write_command_buffer(BNO055Register::PAGE_ID, reinterpret_cast<const uint8_t*>(&page_id), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); }, 180, 100);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PAGE ID response success");

        // SELECT OSCILLATOR
        uint8_t osc_sel = 0x00;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSC SEL sended.");
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); }, 180, 100);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSC SEL response success");

        // SET UNITS
        uint8_t units = 0x02;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UNIT SEL sended.");
        size = create_write_command_buffer(BNO055Register::UNIT_SEL, reinterpret_cast<const uint8_t*>(&units), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); }, 180, 100);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UNIT SEL response success");

        // SET NDOF MODE
        operation_mode = BNO055OperationMode::NDOF;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDOF MODE sended.");
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); }, 180, 100);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDOF MODE response success");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BNO055 initialization fully done.");
    }

    /**
     * @brief Initializes the BNO055 sensor with calibration data.
     * @param acc_offset Accelerometer offset values.
     * @param mag_offset Magnetometer offset values.
     * @param gyro_offset Gyroscope offset values.
     * @param acc_radius Accelerometer radius.
     * @param mag_radius Magnetometer radius.
     */

    void BNO055::initialize_calib(std::vector<uint16_t> &acc_offset, std::vector<uint16_t> &mag_offset, std::vector<uint16_t> &gyro_offset, int16_t acc_radius, int16_t mag_radius)
    {
        size_t size = 0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration initialize started.");

        // CHIP ID Doğrulama
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CHIP ID sended.");
        size = create_read_command_buffer(BNO055Register::CHIP_ID, 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 3);

        if (check_receive_command(receiving_buffer, 1) && 
            receiving_buffer[0] == BNO055MessageType::READ_RESPONSE &&
            receiving_buffer[2] == BNO055ChipID::BNO_CHIP_ID) {
            return true;
        }
        return false; });

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "CHIP ID response success");

        // Operation Mode Ayarı
        uint8_t operation_mode = BNO055OperationMode::CONFIG;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OPERATION MODE sended.");
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OPERATION MODE response success");

        // Power Mode Ayarı
        uint8_t power_mode = BNO055PowerMode::NORMAL;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "POWER MODE sended.");
        size = create_write_command_buffer(BNO055Register::PWR_MODE, reinterpret_cast<const uint8_t*>(&power_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "POWER MODE response success");

        // Page ID Ayarı
        uint8_t page_id = 0;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PAGE ID sended.");
        size = create_write_command_buffer(BNO055Register::PAGE_ID, reinterpret_cast<const uint8_t*>(&page_id), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PAGE ID response success");

        // Oscillator Selection (OSC_SEL)
        uint8_t osc_sel = 0x00;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSC SEL sended.");
        size = create_write_command_buffer(BNO055Register::SYS_TRIGGER, reinterpret_cast<const uint8_t*>(&osc_sel), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OSC SEL response success");

        // Unit Selection (UNIT_SEL)
        uint8_t units = 0x02;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UNIT SEL sended.");
        size = create_write_command_buffer(BNO055Register::UNIT_SEL, reinterpret_cast<const uint8_t*>(&units), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UNIT SEL response success");

        // ACC OFFSET Ayarı
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC OFFSET sended.");
        size = create_write_command_buffer(BNO055Register::ACCEL_OFFSET_X_LSB_ADDR, create_offset_array(acc_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC OFFSET response success");

        // MAG OFFSET Ayarı
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MAG OFFSET sended.");
        size = create_write_command_buffer(BNO055Register::MAG_OFFSET_X_LSB_ADDR, create_offset_array(mag_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MAG OFFSET response success");

        // GYRO OFFSET Ayarı
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GYRO OFFSET sended.");
        size = create_write_command_buffer(BNO055Register::GYRO_OFFSET_X_LSB_ADDR, create_offset_array(gyro_offset), 6, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GYRO OFFSET response success");

        // ACC RADIUS Ayarı
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC RADIUS sended.");
        size = create_write_command_buffer(BNO055Register::ACCEL_RADIUS_LSB_ADDR, create_radius_array(acc_radius), 2, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACC RADIUS response success");

        // MAG RADIUS Ayarı
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MAG RADIUS sended.");
        size = create_write_command_buffer(BNO055Register::MAG_RADIUS_LSB_ADDR, create_radius_array(mag_radius), 2, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MAG RADIUS response success");

        // İşlem modu (NDOF) Ayarı
        operation_mode = BNO055OperationMode::NDOF;
        retry_until_success([&]()
                            {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDOF MODE sended.");
        size = create_write_command_buffer(BNO055Register::OPR_MODE, reinterpret_cast<const uint8_t*>(&operation_mode), 1, sending_buffer);
        serial_port_.write(reinterpret_cast<char*>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char*>(receiving_buffer), 2);
        return check_write_status(receiving_buffer); });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "NDOF MODE response success");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration initialize fully done.");
    }

    size_t BNO055::create_write_command_buffer(BNO055Register reg_adr, const uint8_t *data, size_t data_len, uint8_t *buffer)
    {
        size_t index = 0;
        buffer[index++] = BNO055MessageType::REGISTER_COMMAND;
        buffer[index++] = BNO055RegisterCommand::WRITE;
        buffer[index++] = reg_adr;
        buffer[index++] = data_len;
        if (data_len > 0)
        {
            for (size_t i = 0; i < data_len; i++)
            {
                buffer[index++] = data[i];
            }
        }
        return index;
    }

    /**
     * @brief Creates a write command buffer for the BNO055 sensor.
     * @param reg The register to write to.
     * @param data The data to write.
     * @param length The length of the data.
     * @param buffer The buffer to store the command.
     * @return The size of the command.
     */

    size_t BNO055::create_write_command_buffer(BNO055ConfigRegister reg_adr, const uint8_t *data, size_t data_len, uint8_t *buffer)
    {
        size_t index = 0;
        buffer[index++] = BNO055MessageType::REGISTER_COMMAND;
        buffer[index++] = BNO055RegisterCommand::WRITE;
        buffer[index++] = reg_adr;
        buffer[index++] = data_len;
        if (data_len > 0)
        {
            for (size_t i = 0; i < data_len; i++)
            {
                buffer[index++] = data[i];
            }
        }
        return index;
    }

    /**
     * @brief Creates a read command buffer for the BNO055 sensor.
     * @param reg The register to read from.
     * @param length The length of data to read.
     * @param buffer The buffer to store the command.
     * @return The size of the command.
     */

    size_t BNO055::create_read_command_buffer(BNO055Register reg_adr, size_t data_len, uint8_t *buffer)
    {
        size_t index = 0;
        buffer[index++] = BNO055MessageType::REGISTER_COMMAND;
        buffer[index++] = BNO055RegisterCommand::READ;
        buffer[index++] = reg_adr;
        buffer[index++] = data_len;
        return index;
    }

    /**
     * @brief Checks the status of a received read command.
     * @param buffer The buffer containing the received data.
     * @param length The expected length of the data.
     * @return True if the status is successful, false otherwise.
     */

    bool BNO055::check_receive_command(const uint8_t *buffer, size_t data_len)
    {
        if (buffer[0] != BNO055MessageType::READ_RESPONSE)
        {
            return false;
        }
        if (buffer[1] != data_len)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Checks the status of a received write command.
     * @param buffer The buffer containing the received data.
     * @return True if the status is successful, false otherwise.
     */

    bool BNO055::check_write_status(const uint8_t *buffer)
    {
        if (buffer[0] != BNO055MessageType::RESPONSE_STATUS)
        {
            return false;
        }
        if (buffer[1] != 0x01)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Creates an offset array for the BNO055 sensor.
     * @param offsets The offsets to set.
     * @return A pointer to the array of offsets.
     */

    uint8_t *BNO055::create_offset_array(std::vector<uint16_t> &offset)
    {
        uint8_t *offset_array = new uint8_t[offset.size() * 2];
        for (size_t i = 0; i < offset.size(); i++)
        {
            offset_array[i * 2] = offset[i] & 0xFF;
            offset_array[i * 2 + 1] = (offset[i] >> 8) & 0xFF;
        }
        return offset_array;
    }

    /**
     * @brief Creates a radius array for the BNO055 sensor.
     * @param radius The radius to set.
     * @return A pointer to the array of radius values.
     */

    uint8_t *BNO055::create_radius_array(int16_t radius)
    {
        uint8_t *radius_array = new uint8_t[sizeof(int16_t)];
        radius_array[0] = radius & 0xFF;
        radius_array[1] = (radius >> 8) & 0xFF;
        return radius_array;
    }

    RawBNO055Data BNO055::read_raw_data()
    {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::ACC_DATA_X_LSB, sizeof(RawBNO055Data), sending_buffer);
        serial_port_.write(reinterpret_cast<char *>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char *>(receiving_buffer), sizeof(RawBNO055Data) + 2);
        if (check_receive_command(receiving_buffer, 45) == false)
        {
            memset(receiving_buffer, 0, sizeof(RawBNO055Data) + 2);
            throw BNO055Exception("ACC DATA response error");
        }

        RawBNO055Data data;
        std::memcpy(&data, &receiving_buffer[2], sizeof(RawBNO055Data));
        return data;
    }

    CalibrationBNO055Status BNO055::read_calib_status()
    {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::CALIB_STAT, sizeof(CalibrationBNO055Status), sending_buffer);
        serial_port_.write(reinterpret_cast<char *>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char *>(receiving_buffer), sizeof(CalibrationBNO055Status) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055Status)) == false)
        {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055Status) + 2);
            throw BNO055Exception("1");
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055Status calib_status;
        std::memcpy(&calib_status, &receiving_buffer[2], sizeof(CalibrationBNO055Status));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_sys: %d", calib_status.system);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_acc: %d", calib_status.acc);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_gyr: %d", calib_status.gyro);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calib_mag: %d", calib_status.mag);

        return calib_status;
    }

    CalibrationBNO055DataAcc BNO055::read_calib_data_acc()
    {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::ACCEL_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055DataAcc), sending_buffer);
        serial_port_.write(reinterpret_cast<char *>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char *>(receiving_buffer), sizeof(CalibrationBNO055DataAcc) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055DataAcc)) == false)
        {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055DataAcc) + 2);
            throw BNO055Exception("2");
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055DataAcc acc_offset;
        std::memcpy(&acc_offset, &receiving_buffer[2], sizeof(CalibrationBNO055DataAcc));

        return acc_offset;
    }

    CalibrationBNO055DataMag BNO055::read_calib_data_mag()
    {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::MAG_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055DataMag), sending_buffer);
        serial_port_.write(reinterpret_cast<char *>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char *>(receiving_buffer), sizeof(CalibrationBNO055DataMag) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055DataMag)) == false)
        {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055DataMag) + 2);
            throw BNO055Exception("3");
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055DataMag mag_offset;
        std::memcpy(&mag_offset, &receiving_buffer[2], sizeof(CalibrationBNO055DataMag));

        return mag_offset;
    }

    CalibrationBNO055DataGyro BNO055::read_calib_data_gyro()
    {
        size_t size = 0;
        size = create_read_command_buffer(BNO055Register::GYRO_OFFSET_X_LSB_ADDR, sizeof(CalibrationBNO055DataGyro), sending_buffer);
        serial_port_.write(reinterpret_cast<char *>(sending_buffer), size);
        serial_port_.read(reinterpret_cast<char *>(receiving_buffer), sizeof(CalibrationBNO055DataGyro) + 2);
        if (check_receive_command(receiving_buffer, sizeof(CalibrationBNO055DataGyro)) == false)
        {
            memset(receiving_buffer, 0, sizeof(CalibrationBNO055DataGyro) + 2);
            throw BNO055Exception("4");
            throw BNO055Exception("CALIB DATA response error");
        }
        CalibrationBNO055DataGyro gyro_offset;
        std::memcpy(&gyro_offset, &receiving_buffer[2], sizeof(CalibrationBNO055DataGyro));

        return gyro_offset;
    }

} // namespace rbf_bno055_driver
