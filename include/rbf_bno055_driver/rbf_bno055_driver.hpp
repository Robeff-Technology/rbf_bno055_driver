#ifndef RBF_BNO055_DRIVER_HPP
#define RBF_BNO055_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <rbf_bno055_driver/serial_port.h>
#include <rbf_bno055_driver/bno055_struct.h>
#include <rbf_bno055_driver/bno055_reg.h>
#include <rbf_bno055_driver/bno055.h>
#include <chrono>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>


namespace rbf_bno055_driver
{
    class BNO055Driver : public rclcpp::Node
    {
    public:
        BNO055Driver(const rclcpp::NodeOptions & options);
    void readSensorData();
    struct Config{
        struct SerialPort{
            std::string port;
            int baudrate;
        }; 
        struct BNO055{    
            float acc_factor;
            float mag_factor;
            float gyro_factor;
            float grav_factor;
            bool set_offset;
            bool make_calibration;
            std::vector<uint16_t> mag_offset;
            std::vector<uint16_t> acc_offset;
            std::vector<uint16_t> gyro_offset;
            int16_t acc_radius;
            int16_t mag_radius;
        };
        BNO055 bno055;
        SerialPort serial_port;
    };
    Config config_;
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_2_;

        void timerCallback();
        void newTimerCallback();
        void readAndWriteNewOffsets();
        void load_parameters();

        std::shared_ptr<BNO055> bno_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_grav_;
        sensor_msgs::msg::Imu create_raw_imu_message(const RawBNO055Data& raw_imu_data);
        sensor_msgs::msg::Imu create_imu_message(const RawBNO055Data& imu_data);
        sensor_msgs::msg::MagneticField create_mag_message(const RawBNO055Data& mag_data);
        geometry_msgs::msg::Vector3 create_grav_message(const RawBNO055Data& grav_data);
        
    };
} // namespace rbf_bno055_driver

#endif // RBF_BNO055_DRIVER_HPP