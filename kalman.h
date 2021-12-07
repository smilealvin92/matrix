#ifndef __KALMAN_H
#define __KALMAN_H

// �ο��Ĵ����ַ��https://github.com/gaochq/IMU_Attitude_Estimator

#include "stdio.h"
#include "stdlib.h"
#include "sys.h"
#include "string.h"
#include "matrix.h"
#include "math.h"

#define M_PI       3.14159265358979323846f   // pi

// ������Ԫ���ṹ��
typedef struct{
	float w;
	float xi;
	float yj;
	float zk;
}quaternion_struct;

// ���忨�����˲���ʼ���������Ͻṹ�壬���е�һЩ�����ں����˲������л᲻�ϵĸı�
// Ϊ�˼ӿ쿨�����˲������е��ٶȣ������һЩ������һ��ʼ��ʼ����ʱ��ͷ�����ڴ棬
// ����ֻҪ�ı����е�ֵ���ɣ��������ʱ��ͳһfree��Щ��������freeҲ�ǿ��Ե�
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

// ��ʼ����Ԫ���ṹ��
quaternion_struct *quaternion_struct_build(void);

// ��ʼ��״̬ת�ƾ���
state_trans_paras_struct *state_trans_paras_struct_build(void);

// ��ʼ���������˲�������������
kalman_basic_paras_struct *kalman_basic_paras_struct_build(void);

// ��ʼ�������������м̵ľ���
temp_paras_struct *temp_paras_struct_build(void);

// ��ʼ������������Ԫ���ľ���
quaternion_paras_struct *quaternion_paras_struct_build(void);

// kalman�˲���ѭ������
void kalman_loop(quaternion_paras_struct *quaternion_paras,
                        quaternion_struct *quaternion,
                        kalman_basic_paras_struct *kalman_basic_paras, 
                        temp_paras_struct *temp_paras, 
                        state_trans_paras_struct *state_trans_paras, 
                        float *sensor, float delta_T);

// kalman�㷨���Ժ���
void kalman_main_test(void);

// ��ʼ�����õ���ʼ��״̬������״̬��Э�������״̬ת�ƹ��̵��������󡢲�����������
// ״̬ת�ƾ��󡢹۲����ȵ�
matrix *variable_mat_init(float *val, u32 row_num, u32 column_num, char *variable_name);

// ��������������3�׷��Գƾ��󣬳ɹ��򷵻�0��ʧ���򷵻�-1
int vector3_to_skew_mat(matrix *val, matrix *result);

// ����Ԥ��
void prior_predict(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras, state_trans_paras_struct *state_trans_paras, float delta_T);

// �������У��
void post_correct(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras);

// ������Ԫ��
void build_quaternion(kalman_basic_paras_struct *kalman_basic_paras, quaternion_paras_struct *quaternion_paras, state_trans_paras_struct *state_trans_paras);

// ����ת����תΪ��Ԫ��
void rotation_to_quaterion( quaternion_paras_struct *quaternion_paras, quaternion_struct *quaternion);

// ��ԭʼ�������ݽ���Ԥ������Ҫ�ǰ������Ǻͼ��ٶȼƻ���λ�ã����ٶȼƺʹ����Ƶ�����Ҫ���й�һ��
void measure_preprocess(matrix *measure, temp_paras_struct *temp_paras);


#endif //__KALMAN_H
