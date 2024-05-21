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
            std::vector<int16_t> mag_offset;
            std::vector<int16_t> acc_offset;
            std::vector<int16_t> gyro_offset;
            int16_t acc_radius;
            int16_t mag_radius;
        };
        BNO055 bno055;
        SerialPort serial_port;
    };
    Config config_;
    private:


        rclcpp::TimerBase::SharedPtr timer_;


        void timerCallback();
        void load_parameters();
        void load_startup_params();

        

        std::shared_ptr<BNO055> bno_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
        sensor_msgs::msg::Imu create_imu_message(const RawBNO055Data& raw_data);

    };
} // namespace rbf_bno055_driver

#endif // RBF_BNO055_DRIVER_HPP