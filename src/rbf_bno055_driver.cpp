#include "rbf_bno055_driver/rbf_bno055_driver.hpp"
#include "rbf_bno055_driver/bno055_struct.h"
#include "rbf_bno055_driver/bno055_reg.h"
#include <rclcpp/clock.hpp>


namespace rbf_bno055_driver
{
    BNO055Driver::BNO055Driver(const rclcpp::NodeOptions& options) : Node("rbf_bno055_driver", options) {
        load_parameters();


        RCLCPP_INFO(get_logger(), "Serial port name: %s", config_.serial_port.port.c_str());
        RCLCPP_INFO(get_logger(), "Serial port baud rate: %d", config_.serial_port.baudrate);
        RCLCPP_INFO(get_logger(), "BNO055 acc factor: %f", config_.bno055.acc_factor);
        RCLCPP_INFO(get_logger(), "BNO055 mag factor: %f", config_.bno055.mag_factor);
        RCLCPP_INFO(get_logger(), "BNO055 gyro factor: %f", config_.bno055.gyro_factor);
        RCLCPP_INFO(get_logger(), "BNO055 grav factor: %f", config_.bno055.grav_factor);
        RCLCPP_INFO(get_logger(), "BNO055 acc offset: [%d, %d, %d]", config_.bno055.acc_offset[0], config_.bno055.acc_offset[1], config_.bno055.acc_offset[2]);
        RCLCPP_INFO(get_logger(), "BNO055 mag offset: [%d, %d, %d]", config_.bno055.mag_offset[0], config_.bno055.mag_offset[1], config_.bno055.mag_offset[2]);
        RCLCPP_INFO(get_logger(), "BNO055 gyro offset: [%d, %d, %d]", config_.bno055.gyro_offset[0], config_.bno055.gyro_offset[1], config_.bno055.gyro_offset[2]);

        try{
            bno_ = std::make_shared<BNO055>("/dev/ttyUSB0");
        }
        catch (const SerialPortException& e){
            RCLCPP_ERROR(get_logger(), "Serial port error = %s", e.what());
            rclcpp::shutdown();
        }

        // Initialize the sensor
        try{
            bno_->initialize(config_.bno055.acc_offset, config_.bno055.mag_offset, config_.bno055.gyro_offset, 0, 0);
            
        }
        catch (const BNO055::BNO055Exception& e){
            RCLCPP_ERROR(get_logger(), "BNO055 initialization error = %s", e.what());
            rclcpp::shutdown();
        }

        pub_imu_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
        pub_grav_ = this->create_publisher<geometry_msgs::msg::Vector3>("gravity", 10);

        // Create a timer with a 10 ms period (100 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BNO055Driver::timerCallback, this));
    
        timer_2_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BNO055Driver::newTimerCallback, this));

    }
    void BNO055Driver::load_parameters() {
        // Load serial port parameters
        config_.serial_port.port = this->declare_parameter<std::string>("serial_port.name", "/dev/ttyUSB0");
        config_.serial_port.baudrate = this->declare_parameter<int>("serial_port.baud_rate", 115200);

        // Load BNO055 parameters
        config_.bno055.acc_factor = this->declare_parameter<float>("BNO055.acc_factor", 100.0);
        config_.bno055.mag_factor = this->declare_parameter<float>("BNO055.mag_factor", 16000000.0);
        config_.bno055.gyro_factor = this->declare_parameter<float>("BNO055.gyro_factor", 900.0);
        config_.bno055.grav_factor = this->declare_parameter<float>("BNO055.grav_factor", 100.0);
        
        auto acc_offset_param = this->declare_parameter<std::vector<int>>("BNO055.acc_offset", {0xFFEC, 0x00A5, 0xFFE8});
        auto mag_offset_param = this->declare_parameter<std::vector<int>>("BNO055.mag_offset", {0xFFB4, 0xFE9E, 0x027D});
        auto gyro_offset_param = this->declare_parameter<std::vector<int>>("BNO055.gyro_offset", {0x0002, 0xFFFF, 0xFFFF});

        // Convert std::vector<int> to std::vector<uint16_t>
        config_.bno055.acc_offset.assign(acc_offset_param.begin(), acc_offset_param.end());
        config_.bno055.mag_offset.assign(mag_offset_param.begin(), mag_offset_param.end());
        config_.bno055.gyro_offset.assign(gyro_offset_param.begin(), gyro_offset_param.end());
    }


    void BNO055Driver::timerCallback(){
        RawBNO055Data raw_;
        try{
            raw_ = bno_->read_raw_data();
        }
        catch (const BNO055::BNO055Exception& e){
        }
        // Publish IMU data
        pub_imu_raw_->publish(create_raw_imu_message(raw_));
        pub_imu_->publish(create_imu_message(raw_));
        pub_mag_->publish(create_mag_message(raw_));
        pub_grav_->publish(create_grav_message(raw_));

    }

    void BNO055Driver::newTimerCallback() {
        CalibrationBNO055Status calib_status_;
        try {
                calib_status_ = bno_->read_calib_status();
                RCLCPP_INFO(get_logger(), "System: %d, Gyro: %d, Acc: %d, Mag: %d", calib_status_.system, calib_status_.gyro, calib_status_.acc, calib_status_.mag);
            if (calib_status_.system == 3 && calib_status_.gyro == 3 && calib_status_.acc == 3 && calib_status_.mag == 3) {
                RCLCPP_INFO(get_logger(), "Calibration is fully done.");
                // Calibration is fully done, read new offset values
                readAndWriteNewOffsets();
                timer_2_->cancel();

            }
        } 
        catch (const BNO055::BNO055Exception& e) {
            RCLCPP_ERROR(get_logger(), "Error reading calibration status: %s", e.what());
        }
        
        RCLCPP_INFO(get_logger(), "Timer 2 callback.");

    }
    
    void BNO055Driver::readAndWriteNewOffsets() {
        CalibrationBNO055DataAcc calb_acc_;
        CalibrationBNO055DataMag calb_mag_;
        CalibrationBNO055DataGyro calb_gyro_;
        try {
            RCLCPP_INFO(get_logger(), "GIRDIMMM.");
            calb_acc_ = bno_->read_calib_data_acc();
            calb_mag_ = bno_->read_calib_data_mag();
            calb_gyro_ = bno_->read_calib_data_gyro();

            // Write new offsets to the device
            std::vector<uint16_t> new_acc_offset = {calb_acc_.x_msb << 8 | calb_acc_.x_lsb, calb_acc_.y_msb << 8 | calb_acc_.y_lsb, calb_acc_.z_msb << 8 | calb_acc_.z_lsb};
            std::vector<uint16_t> new_mag_offset = {calb_mag_.x_msb << 8 | calb_mag_.x_lsb, calb_mag_.y_msb << 8 | calb_mag_.y_lsb, calb_mag_.z_msb << 8 | calb_mag_.z_lsb};
            std::vector<uint16_t> new_gyro_offset = {calb_gyro_.x_msb << 8 | calb_gyro_.x_lsb, calb_gyro_.y_msb << 8 | calb_gyro_.y_lsb};
            
            bno_->initialize(new_acc_offset, new_mag_offset, new_gyro_offset, 0, 0);

        
            RCLCPP_INFO(get_logger(), "New offsets have been written to the device.");
        } catch (const BNO055::BNO055Exception& e) {
            RCLCPP_ERROR(get_logger(), "Error reading or writing new offsets: %s", e.what());
        }
    }

    sensor_msgs::msg::Imu BNO055Driver::create_raw_imu_message(const RawBNO055Data& raw_data){
        sensor_msgs::msg::Imu imu_raw_msg;
        imu_raw_msg.header.stamp = now();
        imu_raw_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed

    // Fill in IMU data
        imu_raw_msg.orientation.x = 0.0; // Adjust if orientation data is available
        imu_raw_msg.orientation.y = 0.0;
        imu_raw_msg.orientation.z = 0.0;
        imu_raw_msg.orientation.w = 1.0; // No orientation data available, setting quaternion to identity
        
    // Acceleration data
        imu_raw_msg.linear_acceleration.x = raw_data.acc_x;
        imu_raw_msg.linear_acceleration.y = raw_data.acc_y;
        imu_raw_msg.linear_acceleration.z = raw_data.acc_z;

    // Gyroscope data
        imu_raw_msg.angular_velocity.x = raw_data.gyro_x;
        imu_raw_msg.angular_velocity.y = raw_data.gyro_y;
        imu_raw_msg.angular_velocity.z = raw_data.gyro_z;


        return imu_raw_msg;
    }
    sensor_msgs::msg::Imu BNO055Driver::create_imu_message(const RawBNO055Data& imu_data){
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed

        // Quaternion data
        geometry_msgs::msg::Quaternion q;
        q.w = imu_data.quaternion_w;
        q.x = imu_data.quaternion_x;        
        q.y = imu_data.quaternion_y;
        q.z = imu_data.quaternion_z;

        float norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        imu_msg.orientation.x = q.x / norm;
        imu_msg.orientation.y = q.y / norm;
        imu_msg.orientation.z = q.z / norm;
        imu_msg.orientation.w = q.w / norm;


        // Acceleration data
        float acc_factor = config_.bno055.acc_factor;
        imu_msg.linear_acceleration.x = imu_data.acc_x / acc_factor;
        imu_msg.linear_acceleration.y = imu_data.acc_y / acc_factor;
        imu_msg.linear_acceleration.z = imu_data.acc_z / acc_factor;
        
        // Gyroscope data
        float gyro_factor = config_.bno055.gyro_factor;
        imu_msg.angular_velocity.x = imu_data.gyro_x / gyro_factor;
        imu_msg.angular_velocity.y = imu_data.gyro_y / gyro_factor;
        imu_msg.angular_velocity.z = imu_data.gyro_z / gyro_factor;
        return imu_msg;
    }

    sensor_msgs::msg::MagneticField BNO055Driver::create_mag_message(const RawBNO055Data& mag_data){

        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = now();
        mag_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed
        float mag_factor = config_.bno055.mag_factor;

        // Magnetometer data
        mag_msg.magnetic_field.x = mag_data.mag_x / mag_factor;
        mag_msg.magnetic_field.y = mag_data.mag_y / mag_factor;
        mag_msg.magnetic_field.z = mag_data.mag_z / mag_factor;

        return mag_msg;
    }

    geometry_msgs::msg::Vector3 BNO055Driver::create_grav_message(const RawBNO055Data& grav_data){
        geometry_msgs::msg::Vector3 grav_msg;
        float grav_factor = config_.bno055.grav_factor;
        
        grav_msg.x = grav_data.gravity_x / grav_factor;
        grav_msg.y = grav_data.gravity_y / grav_factor;
        grav_msg.z = grav_data.gravity_z / grav_factor;
        return grav_msg;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_bno055_driver::BNO055Driver)