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
    struct CalibrationBNO055Status
    {
      uint8_t system : 2;
      uint8_t gyro : 2; 
      uint8_t acc : 2; 
      uint8_t mag : 2; 
      uint8_t self_test : 1; 
      uint8_t sys_status : 1;
      uint8_t sys_error : 1;
      uint8_t reserved : 5; 
    };
    struct CalibrationBNO055DataAcc
    {
      uint8_t x_lsb;
      uint8_t x_msb;
      uint8_t y_lsb;
      uint8_t y_msb;
      uint8_t z_lsb;
      uint8_t z_msb;
    };
    struct CalibrationBNO055DataMag
    {
      uint8_t x_lsb;
      uint8_t x_msb;
      uint8_t y_lsb;
      uint8_t y_msb;
      uint8_t z_lsb;
      uint8_t z_msb;
    };
    struct CalibrationBNO055DataGyro
    {
      uint8_t x_lsb;
      uint8_t x_msb;
      uint8_t y_lsb;
      uint8_t y_msb;
      uint8_t z_lsb;
      uint8_t z_msb;
    };
  #pragma pack(pop)
}

#endif // BNO055_STRUCT_H