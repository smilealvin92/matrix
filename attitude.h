#ifndef __ATTITUDE_H
#define __ATTITUDE_H
#include "matrix.h"
#include "kalman.h"
#include "Time.h"
#include "inv_mpu.h"
#include "delay.h"
#include "dcm.h"
#include "ekf.h"
#include "math.h"
#include "mpu6050_1.h"
#include "mpu6050_2.h"
#include "mpu6050_3.h"
#include "mpu6050_4.h"
#include "mpu6050_5.h"
//定义不同测量范围的刻度因子
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.536f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f

#define Gyro_500_rad_Scale_Factor 3754.93624f
#define Accel_4_M_Scale_Factor 837.345272f
#define rad_to_deg 57.295780f

// 下面这个是原作者的偏移
#define Accel_Zout_Offset		600
#define Gyro_Xout_Offset	    -70
#define Gyro_Yout_Offset		25
#define Gyro_Zout_Offset		-10

// 下面是我自己通过求均值得到的偏移
#define Gyro_Xout_Offset_u3	    193
#define Gyro_Yout_Offset_u3		-45
#define Gyro_Zout_Offset_u3		-17

#define Gyro_Xout_Offset_u2	    144
#define Gyro_Yout_Offset_u2		-57
#define Gyro_Zout_Offset_u2		60

#define Gyro_Xout_Offset_u4	    185
#define Gyro_Yout_Offset_u4		-24
#define Gyro_Zout_Offset_u4		70

#define Gyro_Xout_Offset_u5	    15
#define Gyro_Yout_Offset_u5		-14
#define Gyro_Zout_Offset_u5		126

#define Gyro_Xout_Offset_u6	    149
#define Gyro_Yout_Offset_u6		1
#define Gyro_Zout_Offset_u6		125

#define Kp 2.5f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.001f   //integral gain governs rate of convergence of gyroscope biases

// 利用宏定义来选择旋转顺序
// 当旋转顺序是ZXY时，此时绕X轴旋转的是pitch，绕y轴旋转的是roll。总之，最后一个旋转的角度，它就是roll
//#define ZXY
// 当旋转顺序是ZYX时(这种旋转顺序比较常用)，此时绕X轴旋转的是roll，绕y轴旋转的是pitch。总之，最后一个旋转的角度，它就是roll
#define ZYX

void init_quaternion(quaternion_struct *quaternion);
void AHRSupdate(matrix *gyro, matrix *accel, matrix *mag, quaternion_struct *quaternion);
void IMUupdate(matrix *gyro, matrix *accel, matrix *mag, quaternion_struct *quaternion, float *yaw, float halfT);
//void MahonyAHRSupdate(matrix *gyro, matrix *accel, matrix *mag, quaternion_struct *quaternion);
void quater_to_euler(quaternion_struct *quaternion);
void gyro_calibrate(float *gyro_bias);
void acc_calibrate(float *acc_bias, float *gyro_bias);
void acc_calibrate_horizon(float *acc_bias);
void transform_data(short *gyro, short* accel, matrix *gyro_mat, matrix *accel_mat, float *bias);
void transform_data_better(short *gyro, short* accel, matrix *gyro_mat, matrix *accel_mat, float *gyro_bias, float *acc_bias);
void gyro_fusion(matrix *gyro_3, matrix *gyro_2, matrix *gyro_4, matrix *gyro_5, matrix *gyro_6);
void acc_fusion(matrix *gyro_3, matrix *acc_2, matrix *acc_4, matrix *acc_5, matrix *acc_6);
void get_centri_pitch(matrix *acc, float pitch, float *acc_centri);
void get_centri_roll(matrix *acc, float roll, float *acc_centri);
void sensor_fusion(matrix *gyro, matrix *acc_2, matrix *acc_4, matrix *acc_5, matrix *acc_6);
void sensor_fusion_kf(matrix *gyro_3, matrix *gyro_2, matrix *gyro_4, matrix *gyro_5, matrix *gyro_6, matrix *acc_2, matrix *acc_4, 
	matrix *acc_5, matrix *acc_6, float time, double *s, double *P);
void sensor_fusion_with_angle(matrix *gyro, matrix *acc_2, matrix *acc_4, matrix *acc_5, matrix *acc_6, float pitch, float roll);
#endif //__ATTITUDE_H
