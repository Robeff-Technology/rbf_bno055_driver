#ifndef BNO055_STRUCT_H
#define BNO055_STRUCT_H

#include <cstdint>

namespace rbf_bno055_driver {
  #pragma pack(push, 1)
    struct RawBNO055Data
    {
      int16_t acc_x;
      int16_t acc_y;
      int16_t acc_z;
      int16_t mag_x;
      int16_t mag_y;
      int16_t mag_z;
      int16_t gyro_x;
      int16_t gyro_y;
      int16_t gyro_z;
      int16_t euler_heading;
      int16_t euler_roll;
      int16_t euler_pitch;
      int16_t quaternion_w;
      int16_t quaternion_x;
      int16_t quaternion_y;
      int16_t quaternion_z;
      int16_t linear_acc_x;
      int16_t linear_acc_y;
      int16_t linear_acc_z;
      int16_t gravity_x;
      int16_t gravity_y;
      int16_t gravity_z;
      int8_t temp;
    };
  #pragma pack(pop)
}

#endif // BNO055_STRUCT_H