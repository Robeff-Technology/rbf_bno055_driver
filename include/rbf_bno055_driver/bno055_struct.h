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
    struct CalibrationBNO055Data
    {
    int16_t accel_offset_x; /**< x acceleration offset */
    int16_t accel_offset_y; /**< y acceleration offset */
    int16_t accel_offset_z; /**< z acceleration offset */

    int16_t mag_offset_x; /**< x magnetometer offset */
    int16_t mag_offset_y; /**< y magnetometer offset */
    int16_t mag_offset_z; /**< z magnetometer offset */

    int16_t gyro_offset_x; /**< x gyroscrope offset */
    int16_t gyro_offset_y; /**< y gyroscrope offset */
    int16_t gyro_offset_z; /**< z gyroscrope offset */

    int16_t accel_radius; /**< acceleration radius */

    int16_t mag_radius; /**< magnetometer radius */
    };
  #pragma pack(pop)
}

#endif // BNO055_STRUCT_H