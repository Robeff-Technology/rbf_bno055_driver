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

        pub_imu_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
        pub_grav_ = this->create_publisher<geometry_msgs::msg::Vector3>("gravity", 10);

        // Create a timer with a 10 ms period (100 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BNO055Driver::timerCallback, this));

    }


    void BNO055Driver::timerCallback(){
        RawBNO055Data raw_;
        CalibrationBNO055Data calb_;
        try{
            raw_ = bno_->read_raw_data();
            calb_ = bno_->read_calib_data();
        }
        catch (const BNO055::BNO055Exception& e){
        }
        // Publish IMU data
        pub_imu_raw_->publish(create_raw_imu_message(raw_));
        pub_imu_->publish(create_imu_message(raw_));
        pub_mag_->publish(create_mag_message(raw_));
        pub_grav_->publish(create_grav_message(raw_));

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
        double acc_factor = 100.0;
        imu_msg.linear_acceleration.x = imu_data.acc_x / acc_factor;
        imu_msg.linear_acceleration.y = imu_data.acc_y / acc_factor;
        imu_msg.linear_acceleration.z = imu_data.acc_z / acc_factor;
        
        // Gyroscope data
        double gyro_factor = 900.0;
        imu_msg.angular_velocity.x = imu_data.gyro_x / gyro_factor;
        imu_msg.angular_velocity.y = imu_data.gyro_y / gyro_factor;
        imu_msg.angular_velocity.z = imu_data.gyro_z / gyro_factor;
        return imu_msg;
    }

    sensor_msgs::msg::MagneticField BNO055Driver::create_mag_message(const RawBNO055Data& mag_data){

        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = now();
        mag_msg.header.frame_id = "imu_link"; // Adjust frame_id as needed
        double mag_factor = 16000000.0;

        // Magnetometer data
        mag_msg.magnetic_field.x = mag_data.mag_x / mag_factor;
        mag_msg.magnetic_field.y = mag_data.mag_y / mag_factor;
        mag_msg.magnetic_field.z = mag_data.mag_z / mag_factor;

        return mag_msg;
    }

    geometry_msgs::msg::Vector3 BNO055Driver::create_grav_message(const RawBNO055Data& grav_data){
        geometry_msgs::msg::Vector3 grav_msg;

        double grav_factor = 100.0;
        grav_msg.x = grav_data.gravity_x;
        grav_msg.y = grav_data.gravity_y;
        grav_msg.z = grav_data.gravity_z;
        return grav_msg;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_bno055_driver::BNO055Driver)