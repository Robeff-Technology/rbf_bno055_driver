#include "rbf_bno055_driver/rbf_bno055_driver.hpp"
#include "rbf_bno055_driver/bno055_struct.h"

namespace rbf_bno055_driver
{
    BNO055Driver::BNO055Driver(const rclcpp::NodeOptions& options) : Node("rbf_ntrip_driver", options) 
    {
        load_parameters();

        try{
            serial_port_.set_port_name(config_.serial_port.port.c_str());
            serial_port_.open();
            serial_port_.configure(config_.serial_port.baudrate, 8, 'N', 1);
        }
        catch (const SerialPortException& e){
            RCLCPP_ERROR(get_logger(), e.what());
            rclcpp::shutdown();
        }

        // Call the function to read sensor data
        readSensorData();
    }

    void BNO055Driver::readSensorData()
    {
        // Read data from the sensor
        bno055_struct::BNO055Data bno055_data_; // Corrected struct name here
        serial_port_.read(reinterpret_cast<char*>(&bno055_data_), sizeof(bno055_data_)); // Corrected variable name here

        // Publish IMU data
        publishIMUData(bno055_data_); // Corrected variable name here
    }

    void BNO055Driver::publishIMUData(const bno055_struct::BNO055Data& imu_data)
    {
        // Create IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed

        // Fill in IMU data
        imu_msg.orientation.x = 0.0; // Adjust if orientation data is available
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0; // No orientation data available, setting quaternion to identity

        imu_msg.angular_velocity.x = imu_data.x_gyro_output;
        imu_msg.angular_velocity.y = imu_data.y_gyro_output;
        imu_msg.angular_velocity.z = imu_data.z_gyro_output;

        imu_msg.linear_acceleration.x = imu_data.x_accel_output;
        imu_msg.linear_acceleration.y = imu_data.y_accel_output;
        imu_msg.linear_acceleration.z = imu_data.z_accel_output;

        // Magnetometer data
        imu_msg.magnetic_field.x = imu_data.x_magnetometer;
        imu_msg.magnetic_field.y = imu_data.y_magnetometer;
        imu_msg.magnetic_field.z = imu_data.z_magnetometer;

        // Publish IMU message
        // imu_publisher_->publish(imu_msg);
    }

    // Other member function definitions...

} 

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_bno055_driver::BNO055Driver)