#ifndef BNO055_STRUCT_H
#define BNO055_STRUCT_H

#include <cstdint>
#include <rbf_bno055_driver/bno055_reg.h>

namespace rbf_bno055_driver {
  #pragma pack(push, 1)
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
    };

  struct BNO055ReadResponse
  {
    BNO055MessageType message_type;
    union {
      uint8_t length;
      BNO055ResponseStatus response_status;
    };
    uint8_t data[128];
  };

  struct BNO055WriteCommand
  {
    BNO055MessageType message_type;
    BNO055RegisterCommand command;
    BNO055Register address;
    uint8_t length;
    uint8_t data[128];
  };

  struct BNO055ReadCommand
  {
    BNO055MessageType message_type;
    BNO055RegisterCommand command;
    BNO055Register address;
    uint8_t length;
  };

  struct BNO055WriteResponse
  {
    BNO055MessageType message_type;
    BNO055ResponseStatus response_status;
  };
  #pragma pack(pop)
}

#endif // BNO055_STRUCT_H