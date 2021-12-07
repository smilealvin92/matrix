#ifndef __KALMAN_H
#define __KALMAN_H

// 参考的代码地址：https://github.com/gaochq/IMU_Attitude_Estimator

#include "stdio.h"
#include "stdlib.h"
#include "sys.h"
#include "string.h"
#include "matrix.h"
#include "math.h"

#define M_PI       3.14159265358979323846f   // pi

// 定义四元数结构体
typedef struct{
	float w;
	float xi;
	float yj;
	float zk;
}quaternion_struct;

// 定义卡尔曼滤波初始化参数集合结构体，其中的一些参数在后续滤波过程中会不断的改变
// 为了加快卡尔曼滤波器运行的速度，必须把一些矩阵在一开始初始化的时候就分配好内存，
// 后续只要改变其中的值即可，程序结束时，统一free这些参数，不free也是可以的
typedef struct{
	matrix *w_angular;
	matrix *angular_vel;
	matrix *angular_acc;
	matrix *acc_state;
	matrix *mag_state;
}state_trans_paras_struct;

typedef struct{
	matrix *measurement_z_mat;
	matrix *state_x_mat;
	matrix *covaria_p_mat;
	matrix *tran_noise_q_mat;
	matrix *measure_noise_r_mat;
	matrix *obser_h_mat;
	matrix *kalman_gain;
	matrix *state_tran_f_mat;
	matrix *inter_state_tran_f_mat;
}kalman_basic_paras_struct;

typedef struct{
	matrix *y_state;
	matrix *x_state;
	matrix *z_state;
	matrix *rotation_matrix;
}quaternion_paras_struct;

typedef struct{
	matrix *temp_0_12_12;
	matrix *temp_1_12_12;
	matrix *temp9_12;
	matrix *temp_0_12_9;
	matrix *temp_1_12_9;
	matrix *temp_0_9_9;
	matrix *temp_1_9_9;
	matrix *temp3_3;
	matrix *temp3_1;
	matrix *temp_1_3_1;
	matrix *temp12_1;
	matrix *temp9_1;
	matrix *i3;
	matrix *i12;
}temp_paras_struct;

// 初始化四元数结构体
quaternion_struct *quaternion_struct_build(void);

// 初始化状态转移矩阵
state_trans_paras_struct *state_trans_paras_struct_build(void);

// 初始化卡尔曼滤波基础参数矩阵
kalman_basic_paras_struct *kalman_basic_paras_struct_build(void);

// 初始化用来做数据中继的矩阵
temp_paras_struct *temp_paras_struct_build(void);

// 初始化用来计算四元数的矩阵
quaternion_paras_struct *quaternion_paras_struct_build(void);

// kalman滤波器循环调用
void kalman_loop(quaternion_paras_struct *quaternion_paras,
                        quaternion_struct *quaternion,
                        kalman_basic_paras_struct *kalman_basic_paras, 
                        temp_paras_struct *temp_paras, 
                        state_trans_paras_struct *state_trans_paras, 
                        float *sensor, float delta_T);

// kalman算法测试函数
void kalman_main_test(void);

// 初始化，得到初始的状态向量、状态的协方差矩阵、状态转移过程的噪声矩阵、测量噪声矩阵、
// 状态转移矩阵、观测矩阵等等
matrix *variable_mat_init(float *val, u32 row_num, u32 column_num, char *variable_name);

// 从三阶向量构建3阶反对称矩阵，成功则返回0，失败则返回-1
int vector3_to_skew_mat(matrix *val, matrix *result);

// 先验预测
void prior_predict(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras, state_trans_paras_struct *state_trans_paras, float delta_T);

// 后验测量校正
void post_correct(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras);

// 构建四元数
void build_quaternion(kalman_basic_paras_struct *kalman_basic_paras, quaternion_paras_struct *quaternion_paras, state_trans_paras_struct *state_trans_paras);

// 将旋转矩阵转为四元数
void rotation_to_quaterion( quaternion_paras_struct *quaternion_paras, quaternion_struct *quaternion);

// 对原始测量数据进行预处理，主要是把陀螺仪和加速度计换个位置，加速度计和磁力计的数据要进行归一化
void measure_preprocess(matrix *measure, temp_paras_struct *temp_paras);


#endif //__KALMAN_H
