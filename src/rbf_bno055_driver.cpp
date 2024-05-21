#include "rbf_bno055_driver/rbf_bno055_driver.hpp"
#include "rbf_bno055_driver/bno055_struct.h"
#include "rbf_bno055_driver/bno055_reg.h"
#include <rclcpp/clock.hpp>


namespace rbf_bno055_driver
{
    BNO055Driver::BNO055Driver(const rclcpp::NodeOptions& options) : Node("rbf_bno055_driver", options) {
        //load_parameters();

        try{
            bno_ = std::make_shared<BNO055>("/dev/ttyUSB0");
        }
        catch (const SerialPortException& e){
            RCLCPP_ERROR(get_logger(), "Serial port error = %s", e.what());
            rclcpp::shutdown();
        }

        // Initialize the sensor
        try{
            bno_->initialize(config_.bno055.acc_offset, config_.bno055.acc_offset, config_.bno055.acc_offset, 0, 0);
        }
        catch (const BNO055::BNO055Exception& e){
            RCLCPP_ERROR(get_logger(), "BNO055 initialization error = %s", e.what());
            rclcpp::shutdown();
        }

        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        // Create a timer with a 10 ms period (100 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BNO055Driver::timerCallback, this));

    }


    void BNO055Driver::timerCallback(){
        RawBNO055Data raw_;
        CalibrationBNO055Data calb_;
        try{
            // raw_= bno_->read_raw_data();
            calb_ = bno_->read_calib_data();
        }
        catch (const BNO055::BNO055Exception& e){
            RCLCPP_ERROR(get_logger(), "BNO055 read error = %s", e.what());
            
        }
        // Publish IMU data
        pub_imu_->publish(create_imu_message(raw_));
    }

    sensor_msgs::msg::Imu BNO055Driver::create_imu_message(const RawBNO055Data& raw_data){
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed

        // Fill in IMU data
        imu_msg.orientation.x = 0.0; // Adjust if orientation data is available
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0; // No orientation data available, setting quaternion to identity

        imu_msg.angular_velocity.x = raw_data.gyro_x;
        imu_msg.angular_velocity.y = raw_data.gyro_y;
        imu_msg.angular_velocity.z = raw_data.gyro_z;

        imu_msg.linear_acceleration.x = raw_data.acc_x;
        imu_msg.linear_acceleration.y = raw_data.acc_y;
        imu_msg.linear_acceleration.z = raw_data.acc_z;

        return imu_msg;
    }

    // void BNO055Driver::publishIMUData(const bno055_struct::BNO055Data& imu_data)
    // {
    //     // Create IMU message
    //     sensor_msgs::msg::Imu imu_msg;
    //     imu_msg.header.stamp = now();
    //     imu_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed

    //     // Fill in IMU data
    //     imu_msg.orientation.x = 0.0; // Adjust if orientation data is available
    //     imu_msg.orientation.y = 0.0;
    //     imu_msg.orientation.z = 0.0;
    //     imu_msg.orientation.w = 1.0; // No orientation data available, setting quaternion to identity

    //     imu_msg.angular_velocity.x = imu_data.x_gyro_output;
    //     imu_msg.angular_velocity.y = imu_data.y_gyro_output;
    //     imu_msg.angular_velocity.z = imu_data.z_gyro_output;

    //     imu_msg.linear_acceleration.x = imu_data.x_accel_output;
    //     imu_msg.linear_acceleration.y = imu_data.y_accel_output;
    //     imu_msg.linear_acceleration.z = imu_data.z_accel_output;

    //     // Magnetometer data
    //     // imu_msg.magnetic_field.x = imu_data.x_magnetometer;
    //     // imu_msg.magnetic_field.y = imu_data.y_magnetometer;
    //     // imu_msg.magnetic_field.z = imu_data.z_magnetometer;

    //     Publish IMU message
    //     imu_publisher_->publish(imu_msg);
    // }

    // Other member function definitions...
} 

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_bno055_driver::BNO055Driver)