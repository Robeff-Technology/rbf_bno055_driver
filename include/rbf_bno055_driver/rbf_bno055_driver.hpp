#ifndef RBF_BNO055_DRIVER_HPP
#define RBF_BNO055_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <rbf_bno055_driver/serial_port.h>
#include <rbf_bno055_driver/bno055_struct.h>
#include <sensor_msgs/msg/imu.hpp>
#include <rbf_bno055_driver/bno055_reg.h>
#include <chrono>


namespace rbf_bno055_driver
{
    class BNO055Driver : public rclcpp::Node
    {
    public:
        BNO055Driver(const rclcpp::NodeOptions & options);
        ~BNO055Driver() override{
        serial_port_.close();
    }
    void readSensorData();
    struct Config{
        struct SerialPort{
            std::string port;
            int baudrate;
        };
        SerialPort serial_port;
    };
    Config config_;
    private:

        /*
         * Messages
         */
        SerialPort serial_port_;

        rclcpp::TimerBase::SharedPtr timer_;


        void timerCallback();
        void load_parameters();

        /*PUBLISHERS*/
        void publishIMUData(const bno055_struct::BNO055Data& imu_data);

    };
} // namespace rbf_bno055_driver

#endif // RBF_BNO055_DRIVER_HPP