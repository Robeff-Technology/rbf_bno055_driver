#ifndef BNO055_STRUCT_H
#define BNO055_STRUCT_H

#include <cstdint>

namespace bno055_struct {
    struct BNO055Data {
        uint32_t imu_status{};
        int32_t z_accel_output{};
        int32_t y_accel_output{};
        int32_t x_accel_output{};
        int32_t z_gyro_output{};
        int32_t y_gyro_output{};
        int32_t x_gyro_output{};
        double latitude{};
        double longitude{};
        double height{};
        double north_velocity{};
        double east_velocity{};
        double up_velocity{};
        double x_magnetometer{};
        double y_magnetometer{};
        double z_magnetometer{};
    }__attribute__((packed));
}

#endif // BNO055_STRUCT_H