#ifndef INS_H
#define INS_H

#include "struct_typedef.h"

struct bno055_euler_float_t
{
    float h; /**< Euler h float data */
    float r; /**< Euler r float data */
    float p; /**< Euler p float data */
};
struct bno055_gyro_float_t
{
    float x; /**< Gyro x float data */
    float y; /**< Gyro y float data */
    float z; /**< Gyro z float data */
};



struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};





extern const struct bno055_euler_float_t *get_INS_angle_point(void);
extern const struct bno055_gyro_float_t *get_gyro_data_point(void);
extern void imu_task(void const *pvParameters);
extern uint8_t RxData[100];

#endif