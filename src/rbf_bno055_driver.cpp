#include "rbf_bno055_driver/rbf_bno055_driver.hpp"
#include "rbf_bno055_driver/bno055_struct.h"
#include "rbf_bno055_driver/bno055_reg.h"
#include <rclcpp/clock.hpp>


namespace rbf_bno055_driver
{
    BNO055Driver::BNO055Driver(const rclcpp::NodeOptions& options) : Node("rbf_bno055_driver", options) {
        //load_parameters();

        try{
            config_.serial_port.port = "/dev/ttyUSB0";
            serial_port_.set_port_name(config_.serial_port.port.c_str());
            serial_port_.open();
            serial_port_.configure(115200, 8, 'N', 1);
        }
        catch (const SerialPortException& e){
            RCLCPP_ERROR(get_logger(), e.what());
            rclcpp::shutdown();
        }

        load_startup_params();
        // Create a timer with a 10 ms period (100 Hz)
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(10),
        //     std::bind(&BNO055Driver::timerCallback, this));

    }


    void BNO055Driver::timerCallback(){
        // // Read data from the sensor
        // bno055_struct::BNO055Data imu_data;
        // serial_port_.read(reinterpret_cast<char*>(&imu_data), sizeof(imu_data));

        // // Publish IMU data
        // publishIMUData(imu_data);
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

    void BNO055Driver::load_startup_params(){
        BNO055WriteCommand cmd;
        cmd.message_type = REGISTER_COMMAND;
        cmd.command = WRITE;
        cmd.address = BNO055Register::PAGE_ID;
        cmd.length = 1;
        cmd.data[0] = 0;
        serial_port_.write(reinterpret_cast<const char*>(&cmd), 4 + cmd.length);
        BNO055ReadCommand response;
        response.message_type = REGISTER_COMMAND;
        response.command = READ;
        response.length = 1;
        response.address = BNO055Register::CHIP_ID;
        serial_port_.write(reinterpret_cast<const char*>(&response), 4);
        char buff[16];
        serial_port_.read(buff, 16);
        if(buff[0] != 0x0A){
            RCLCPP_INFO(this->get_logger(),"olmadi");
        }
        else{
            RCLCPP_INFO(this->get_logger(),"oldu amk");
        }


    }

} 

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_bno055_driver::BNO055Driver)