#include "kalman.h"

// 参考的代码地址：https://github.com/gaochq/IMU_Attitude_Estimator

// 从三阶向量构建3阶反对称矩阵，成功则返回0，失败则返回-1
int vector3_to_skew_mat(matrix *val, matrix *result)
{
	if(result && result->data)
	{
		// 0行1列
		result->data[1] = (-1)*(val->data[2]);
		// 1行0列
		result->data[3] = val->data[2];
		// 0行2列
		result->data[2] = val->data[1];
		// 2行0列
		result->data[6] = (-1)*(val->data[1]);
		// 1行2列
		result->data[5] = (-1)*(val->data[0]);
		// 2行1列
		result->data[7] = val->data[0];
		return 0;
	}
	else
	{
		printf("memory malloc failed!");
	}
	return -1;
}

// 初始化，得到初始的状态向量、状态的协方差矩阵、状态转移过程的噪声矩阵、测量噪声矩阵、
// 状态转移矩阵、观测矩阵等等
matrix *variable_mat_init(float *val, u32 row_num, u32 column_num, char *variable_name)
{
	matrix *variable_mat = matrix_init(row_num, column_num);
	printf("%s initializing!", variable_name);
	if(variable_mat && variable_mat->data)
	{
		matrix_set_data(variable_mat, val, row_num*column_num);
	}
	else
	{
		printf("%s memory malloc failed!", variable_name);
	}
	return variable_mat;
}

// 初始化四元数结构体
quaternion_struct *quaternion_struct_build(void)
{
	quaternion_struct *quaternion = mymalloc(SRAMIN, sizeof(quaternion_struct));
	if(quaternion)
	{
		mymemset(quaternion, 0, sizeof(quaternion_struct));
	}
	else
	{
		printf("quaternion memory mallco failed!");
	}
	return quaternion;
}

// 初始化状态转移矩阵
state_trans_paras_struct *state_trans_paras_struct_build(void)
{
	state_trans_paras_struct *state_trans_paras = mymalloc(SRAMIN, sizeof(state_trans_paras_struct));
	if(state_trans_paras)
	{
		mymemset(state_trans_paras, 0, sizeof(state_trans_paras_struct));
		state_trans_paras->w_angular = matrix_init(3, 3);
		state_trans_paras->angular_vel = matrix_init(3, 1);
		state_trans_paras->angular_acc = matrix_init(3, 1);
		state_trans_paras->acc_state = matrix_init(3, 1);
		state_trans_paras->mag_state = matrix_init(3, 1);
	}
	else
	{
		printf("quaternion memory malloc failed!");
	}
	return state_trans_paras;
}

// 初始化卡尔曼滤波基础参数矩阵
kalman_basic_paras_struct *kalman_basic_paras_struct_build(void)
{
	matrix *i3 = NULL;
	float state_x[12] = {0.0, -0.0,  0.0,    0.0, -0.0, -0.0, 
											 0.0, -0.0, -0.9998, 0.0,  0.0,  1.0};
	float covaria_p[144] = {0.004, 0.0,    0.0,   0.056,  0.0,     0.0,    0.0,     0.0,     0.0,    0.0,     0.0,    0.0,
													0.0,   0.004,  0.0,   0.0,    0.056,   0.0,    0.0,     0.0,     0.0,    0.0,     0.0,    0.0,
													0.0,   0.0,    0.004, 0.0,    0.0,     0.056,  0.0,     0.0,     0.0,    0.0,     0.0,    0.0,
													0.056, 0.0,    0.0,   0.2971, 0.0,     0.0,    0.0,     -0.0001, 0.0,    0.0,     0.0001, 0.0001,
													0.0,   0.056,  0.0,   0.0,    0.2971,  0.0,    0.0001,  0.0,     0.0,   -0.0001,  0.0,    0.0,
													0.0,   0.0,    0.056, 0.0,    0.0,     0.2971, 0.0,     0.0,     0.0,   -0.0001,  0.0,    0.0,
													0.0,   0.0,    0.0,   0.0,    0.0001,  0.0,    2.9956,  0.0,     0.0,    0.0,     0.0,    0.0,
													0.0,   0.0,    0.0,  -0.0001, 0.0,     0.0,    0.0,     2.9956,  0.0,    0.0,     0.0,    0.0,
													0.0,   0.0,    0.0,   0.0,    0.0,     0.0,    0.0,     0.0,     2.9956, 0.0,     0.0,    0.0,
													0.0,   0.0,    0.0,   0.0,   -0.0001, -0.0001, 0.0,     0.0,     0.0,    0.7046,  0.0,    0.0,
													0.0,   0.0,    0.0,   0.0001, 0.0,     0.0,    0.0,     0.0,     0.0,    0.0,     0.7046, 0.0,
													0.0,   0.0,    0.0,   0.0001, 0.0,     0.0,    0.0,     0.0,     0.0,    0.0,     0.0,    0.7046,};
	float tran_noise_q[12] = {1e-4,  1e-4,  1e-4,  0.08,  0.08,  0.08,
														0.009, 0.009, 0.009, 0.005, 0.005, 0.005};
	float measure_noise_r[9] = {0.0008, 0.0008, 0.0008, 
															 1000, 1000, 1000, 
															 100,  100,  100};
	float obser_h[108] = {1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
												0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
												0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
												0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,  0.0,
												0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0,
												0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,  0.0,
												0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,  0.0,
												0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
												0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1.0,};
	kalman_basic_paras_struct *kalman_basic_paras = mymalloc(SRAMIN, sizeof(kalman_basic_paras_struct));
	if(kalman_basic_paras)
	{
		mymemset(kalman_basic_paras, 0, sizeof(kalman_basic_paras_struct));
		// 1.传感器输入测量值向量Z
		kalman_basic_paras->measurement_z_mat = matrix_init(9, 1);
		// 2.初始化状态向量X
	  kalman_basic_paras->state_x_mat = variable_mat_init(state_x, 12, 1, "state_x_mat");
		// 3.初始化状态向量的协方差矩阵
		kalman_basic_paras->covaria_p_mat = variable_mat_init(covaria_p, 12, 12, "covaria_p_mat");
		// 4.初始化状态向量转移过程中的转移噪声矩阵
		kalman_basic_paras->tran_noise_q_mat = create_diagonal(tran_noise_q, 12);
		// 5.初始化测量或者观测过程中的测量噪声矩阵
		kalman_basic_paras->measure_noise_r_mat = create_diagonal(measure_noise_r, 9);
		// 6.初始化观测或者说测量矩阵
		kalman_basic_paras->obser_h_mat = variable_mat_init(obser_h, 9, 12, "obser_h_mat");
		// 7.卡尔曼增益
		kalman_basic_paras->kalman_gain = matrix_init(12, 9);
		// 8.初始化状态转移矩阵
		i3 = matrix_create_identity(3);
		kalman_basic_paras->state_tran_f_mat = matrix_init(12, 12);
    matrix_padding(kalman_basic_paras->state_tran_f_mat, i3, 0, 3, 1);
		matrix_free(&i3);
		kalman_basic_paras->inter_state_tran_f_mat = matrix_init(12, 12);
	}
	else
	{
		printf("kalman_basic_paras memory malloc failed!");
	}
	return kalman_basic_paras;
}

// 初始化用来做数据中继的矩阵
temp_paras_struct *temp_paras_struct_build(void)
{
	temp_paras_struct *temp_paras = mymalloc(SRAMIN, sizeof(temp_paras_struct));
	if(temp_paras)
	{
		mymemset(temp_paras, 0, sizeof(temp_paras_struct));
		temp_paras->temp_0_12_12 = matrix_init(12, 12);
    temp_paras->temp_1_12_12 = matrix_init(12, 12);		
    temp_paras->temp9_12 = matrix_init(9, 12);		
    temp_paras->temp_0_12_9 = matrix_init(12, 9);
		temp_paras->temp_1_12_9 = matrix_init(12, 9);
    temp_paras->temp_0_9_9 = matrix_init(9, 9);		
    temp_paras->temp_1_9_9 = matrix_init(9, 9);		
    temp_paras->temp3_3 = matrix_init(3, 3);		
    temp_paras->temp3_1 = matrix_init(3, 1);	
		temp_paras->temp_1_3_1 = matrix_init(3, 1);	
    temp_paras->temp12_1 = matrix_init(12, 1);	
    temp_paras->temp9_1 = matrix_init(9, 1);	
    temp_paras->i3 = matrix_create_identity(3);;	
    temp_paras->i12 = matrix_create_identity(12);;	
	}
	else
	{
		printf("temp_paras memory mallco failed!");
	}
	return temp_paras;
}

// 初始化用来计算四元数的矩阵
quaternion_paras_struct *quaternion_paras_struct_build(void)
{
	quaternion_paras_struct *quaternion_paras = mymalloc(SRAMIN, sizeof(quaternion_paras_struct));
	if(quaternion_paras)
	{
		mymemset(quaternion_paras, 0, sizeof(quaternion_paras_struct));
		quaternion_paras->y_state = matrix_init(3, 1);
		quaternion_paras->x_state = matrix_init(3, 1);
		quaternion_paras->z_state = matrix_init(3, 1);
		quaternion_paras->rotation_matrix = matrix_init(3, 3);
	}
	else
	{
		printf("quaternion_paras memory mallco failed!");
	}
	return quaternion_paras;
}

// 先验预测
void prior_predict(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras, state_trans_paras_struct *state_trans_paras, float delta_T)
{
	// 计算本次滤波过程所需的状态转移矩阵
	// 先从旧的状态向量中取出数据，准备数据
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_vel, 0, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_acc, 3, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->acc_state, 6, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->mag_state, 9, 0, 0);
//	printf("\r\nbegin!");
//	printf("\r\nstate_x_mat");
//	printf_matrix(kalman_basic_paras->state_x_mat);
	vector3_to_skew_mat(state_trans_paras->angular_vel, state_trans_paras->w_angular);
	matrix_multiply_scalar( state_trans_paras->w_angular, -1);
	// 利用临时中继结构体，先填充r_acc
	vector3_to_skew_mat(state_trans_paras->acc_state, temp_paras->temp3_3);
	matrix_padding(kalman_basic_paras->state_tran_f_mat, temp_paras->temp3_3, 6, 0, 1);
	
	// 由于要填一个负的r_mag，所以乘以-1
	vector3_to_skew_mat(state_trans_paras->mag_state, temp_paras->temp3_3);
	matrix_multiply_scalar( temp_paras->temp3_3, -1);
	matrix_padding(kalman_basic_paras->state_tran_f_mat, temp_paras->temp3_3, 9, 0, 1);
	// 将数据填充到新的状态转移矩阵中
	matrix_padding(kalman_basic_paras->state_tran_f_mat, state_trans_paras->w_angular, 6, 6, 1);
	matrix_padding(kalman_basic_paras->state_tran_f_mat, state_trans_paras->w_angular, 9, 9, 1);
	
	// 填充完之后，开始计算，此处必须使用一个中继矩阵，来保证下一轮滤波过程中，状态转移矩阵的对角依然为0
	matrix_padding(kalman_basic_paras->inter_state_tran_f_mat, kalman_basic_paras->state_tran_f_mat, 0, 0, 1);
	matrix_multiply_scalar( kalman_basic_paras->inter_state_tran_f_mat, delta_T);
	matrix_add(kalman_basic_paras->inter_state_tran_f_mat, temp_paras->i12, kalman_basic_paras->inter_state_tran_f_mat);
//	printf("\r\nstate_tran_f_mat");
//	printf_matrix(kalman_basic_paras->state_tran_f_mat);
	// 状态转移矩阵准备好之后，进行状态的先验预测
	// 先将角加速度填入新的状态向量，不然填晚了，就不是原来的角加速度了，其值就已经被改变了，这个值从开始到后面好像一直没有变
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_acc, 3, 0, 1);
	// 再将角速度填入新的状态向量
	matrix_multiply_scalar( state_trans_paras->angular_acc, delta_T);
	matrix_add(state_trans_paras->angular_vel, state_trans_paras->angular_acc, state_trans_paras->angular_vel);
	
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_acc, 0, 0, 1);
//	printf_matrix(state_trans_paras->angular_acc);
//	printf_matrix(state_trans_paras->mag_state);
	// 再计算acc_state和mag_state，此时使用二阶法
	matrix_multiply(state_trans_paras->w_angular, state_trans_paras->w_angular, temp_paras->temp3_3);
	matrix_multiply_scalar( temp_paras->temp3_3, delta_T*delta_T/2);
	matrix_multiply_scalar( state_trans_paras->w_angular, delta_T);// 此处w_angular已被改变
	matrix_add(state_trans_paras->w_angular, temp_paras->temp3_3, temp_paras->temp3_3);
	matrix_add(temp_paras->i3, temp_paras->temp3_3, temp_paras->temp3_3);

	// 利用临时中继结构体，完成矩阵相乘的计算
	matrix_padding(state_trans_paras->acc_state, temp_paras->temp3_1, 0, 0, 0);
	matrix_multiply(temp_paras->temp3_3, temp_paras->temp3_1, state_trans_paras->acc_state);
	matrix_padding(state_trans_paras->mag_state, temp_paras->temp3_1, 0, 0, 0);
	matrix_multiply(temp_paras->temp3_3, temp_paras->temp3_1, state_trans_paras->mag_state);
	
	
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->acc_state, 6, 0, 1);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->mag_state, 9, 0, 1);
//	printf("\r\nstate_x_mat");
//	printf_matrix(kalman_basic_paras->state_x_mat);
//	printf("end!");
	// 此时新的状态向量已经更新完毕，开始更新协方差矩阵
	// 此处公式：P = F * P * F转置 + Q
	matrix_multiply(kalman_basic_paras->inter_state_tran_f_mat, kalman_basic_paras->covaria_p_mat, temp_paras->temp_0_12_12);
	// 状态转移矩阵的转置放在临时中继结构体中
	matrix_transpose(kalman_basic_paras->inter_state_tran_f_mat, temp_paras->temp_1_12_12);
	matrix_multiply(temp_paras->temp_0_12_12, temp_paras->temp_1_12_12, kalman_basic_paras->covaria_p_mat);
	matrix_add(kalman_basic_paras->covaria_p_mat, kalman_basic_paras->tran_noise_q_mat, kalman_basic_paras->covaria_p_mat);
//	printf("\r\ntran_noise_q_mat");
//  printf_matrix(kalman_basic_paras->tran_noise_q_mat);
}

// 后验测量校正
void post_correct(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras)
{
	// 后验校正
	// 把观测矩阵的转置存在临时中继结构体中
	// 先计算卡尔曼增益
	// 此处公式 K = P * H转置 * (H * P * H转置 + R)逆
	matrix_transpose(kalman_basic_paras->obser_h_mat, temp_paras->temp_0_12_9);
	matrix_multiply(kalman_basic_paras->obser_h_mat, kalman_basic_paras->covaria_p_mat, temp_paras->temp9_12);
//	printf("temp_paras->temp_0_9_9->ncolumn: %d\n", temp_paras->temp_0_9_9->ncolumn);
//	printf("temp_paras->temp_0_9_9->nrow: %d\n", temp_paras->temp_0_9_9->nrow);
	matrix_multiply(temp_paras->temp9_12, temp_paras->temp_0_12_9, temp_paras->temp_0_9_9);
	matrix_add(temp_paras->temp_0_9_9, kalman_basic_paras->measure_noise_r_mat, temp_paras->temp_0_9_9);
	
	// 计算卡尔曼增益的时候，会计算一个逆矩阵，这个逆矩阵存在临时中继结构体中
	matrix_inverse_gauss(temp_paras->temp_0_9_9, temp_paras->temp_1_9_9);
//	printf("\r\ntemp_1_9_9");
//	printf_matrix(temp_paras->temp_1_9_9);
//	printf("hereabcdefg!!!");
	matrix_multiply(kalman_basic_paras->covaria_p_mat, temp_paras->temp_0_12_9, temp_paras->temp_1_12_9);
	matrix_multiply(temp_paras->temp_1_12_9, temp_paras->temp_1_9_9, kalman_basic_paras->kalman_gain);
//	printf("\r\nkalman_gain");
//	printf_matrix(kalman_basic_paras->kalman_gain);
	// 更新状态向量
	// 此处公式 X = X + K(Z - H * X)
//	printf("\r\nmeasurement_z_mat");
//	printf_matrix(kalman_basic_paras->measurement_z_mat);
	matrix_multiply(kalman_basic_paras->obser_h_mat, kalman_basic_paras->state_x_mat, temp_paras->temp9_1);
	matrix_multiply_scalar( temp_paras->temp9_1, -1);
	matrix_add(temp_paras->temp9_1, kalman_basic_paras->measurement_z_mat, temp_paras->temp9_1);
	matrix_multiply(kalman_basic_paras->kalman_gain, temp_paras->temp9_1, temp_paras->temp12_1);
	matrix_add(temp_paras->temp12_1, kalman_basic_paras->state_x_mat, kalman_basic_paras->state_x_mat);
//	printf("\r\ntemp12_1");
//	printf_matrix(temp_paras->temp12_1);
//	printf("end!");
	// 更新状态向量的协方差矩阵
	// 此处公式 P = (I - K * H) * P
	matrix_multiply(kalman_basic_paras->kalman_gain, kalman_basic_paras->obser_h_mat, temp_paras->temp_0_12_12);
	matrix_multiply_scalar( temp_paras->temp_0_12_12, -1);
	matrix_add(temp_paras->temp_0_12_12, temp_paras->i12, temp_paras->temp_0_12_12);
	matrix_multiply(temp_paras->temp_0_12_12, kalman_basic_paras->covaria_p_mat, temp_paras->temp_1_12_12);
	matrix_padding(kalman_basic_paras->covaria_p_mat, temp_paras->temp_1_12_12, 0, 0, 1);
	
}

// 构建四元数
void build_quaternion(kalman_basic_paras_struct *kalman_basic_paras, quaternion_paras_struct *quaternion_paras, state_trans_paras_struct *state_trans_paras)
{
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->acc_state, 6, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->mag_state, 9, 0, 0);
//	printf("\r\nstate_trans_paras->mag_state->data[0]: %f", state_trans_paras->mag_state->data[0]);
	vector_normalize(state_trans_paras->acc_state);
	vector_normalize(state_trans_paras->mag_state);
	
	// kalman_paras->r_acc_v就是z_state;
	matrix_multiply_scalar( state_trans_paras->acc_state, -1);
	matrix_padding(quaternion_paras->z_state, state_trans_paras->acc_state, 0, 0, 1);
	
	matrix_cross_product_3(quaternion_paras->z_state, state_trans_paras->mag_state, quaternion_paras->y_state);
	vector_normalize(quaternion_paras->y_state);
	matrix_cross_product_3(quaternion_paras->y_state, quaternion_paras->z_state, quaternion_paras->x_state);
	vector_normalize(quaternion_paras->x_state);
	
	// 此处的旋转顺序是X-Y-Z
	
	matrix_padding(quaternion_paras->rotation_matrix, quaternion_paras->x_state, 0, 0, 1);
	matrix_padding(quaternion_paras->rotation_matrix, quaternion_paras->y_state, 0, 1, 1);
	matrix_padding(quaternion_paras->rotation_matrix, quaternion_paras->z_state, 0, 2, 1);
	// 此处用w_angular做一个临时的中继结构体，反正它的数据暂时用不着了
	matrix_transpose(quaternion_paras->rotation_matrix, state_trans_paras->w_angular);
	matrix_padding(quaternion_paras->rotation_matrix, state_trans_paras->w_angular, 0, 0, 1);
}

// 将旋转矩阵转为四元数
void rotation_to_quaterion( quaternion_paras_struct *quaternion_paras, quaternion_struct *quaternion)
{
	// 旋转矩阵是可以由四元数里的(w, xi+yj+zk)的w,x,y,z写成的，因此，在有了旋转矩阵之后，可以反推四元数
	// 公式是：w = 根号(trace(R)+1)/2，x = (m21-m12)/(4w)，y = (m02-m20)/(4w)，x = (m10-m01)/(4w)
	float trace = 0.0f;
	matrix *rotation = quaternion_paras->rotation_matrix;
	trace = matrix_trace(rotation);
//	printf("\r\ntrace: %f", trace);
	quaternion->w = sqrtf(trace+1)/2;
	quaternion->xi = (rotation->data[2*rotation->ncolumn+1]-rotation->data[1*rotation->ncolumn+2])/(4*quaternion->w);
	quaternion->yj = (rotation->data[0*rotation->ncolumn+2]-rotation->data[2*rotation->ncolumn+0])/(4*quaternion->w);
	quaternion->zk = (rotation->data[1*rotation->ncolumn+0]-rotation->data[0*rotation->ncolumn+1])/(4*quaternion->w);	
}

// 对原始测量数据进行预处理，主要是把陀螺仪和加速度计换个位置，加速度计和磁力计的数据要进行归一化
void measure_preprocess(matrix *measure, temp_paras_struct *temp_paras)
{
	// acc_measure
	matrix_padding(measure, temp_paras->temp3_1, 0, 0, 0);
	// gyro_measure
	matrix_padding(measure, temp_paras->temp_1_3_1, 3, 0, 0);
	// 加速度计数据归一化
	vector_normalize(temp_paras->temp3_1);
	// 加速度计和陀螺仪换个位置
	// acc_measure
	matrix_padding(measure, temp_paras->temp3_1, 3, 0, 1);
	// gyro_measure
	matrix_padding(measure, temp_paras->temp_1_3_1, 0, 0, 1);
	
	// mag_measure归一化
	matrix_padding(measure, temp_paras->temp3_1, 6, 0, 0);
	vector_normalize(temp_paras->temp3_1);
	matrix_padding(measure, temp_paras->temp3_1, 6, 0, 1);
}

// kalman算法测试函数
// 看看原有代码的输入的传感器数据的形式，对照下我们的6050数据，对这个代码进行实验
void kalman_main_test(void)
{
	float w, xi, yj, zk = 0.0f;
	float roll, pitch, yaw = 0.0f;
	float sensor[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	state_trans_paras_struct* state_trans_paras = state_trans_paras_struct_build();
	kalman_basic_paras_struct* kalman_basic_paras = kalman_basic_paras_struct_build();
	temp_paras_struct* temp_paras = temp_paras_struct_build();
	quaternion_paras_struct* quaternion_paras = quaternion_paras_struct_build();
	quaternion_struct *quaternion = quaternion_struct_build();
	float delta_T = 0.02;
	while(1)
	{
		// 获取新的测量数据，再进行卡尔曼滤波
		// sensor = ......
		kalman_loop(quaternion_paras, quaternion, kalman_basic_paras, temp_paras, 
	            state_trans_paras, sensor, delta_T);
		// 然后输出欧拉角
		// 旋转矩阵的旋转顺序是Z-Y-X
		w = quaternion->w;
		xi = quaternion->xi;
		yj = quaternion->yj;
		zk = quaternion->zk;
		roll = atan2(2 * (yj*zk + w*xi), (w*w - xi*xi - yj*yj + zk*zk));
    pitch = asin(-2 * xi*zk + 2 * w*yj);
    yaw = atan2(2 * (xi*yj + w*zk), (w*w + xi*xi - yj*yj - zk*zk)) - 8.3f*M_PI/180.0f;
		printf("\nroll: %f度   pitch: %f度   yaw: %f度   ", roll*(180.0f/M_PI), pitch*(180.0f/M_PI), yaw*(180.0f/M_PI));
	}
}

// kalman滤波器循环调用
void kalman_loop(quaternion_paras_struct *quaternion_paras,
                        quaternion_struct *quaternion,
                        kalman_basic_paras_struct *kalman_basic_paras, 
                        temp_paras_struct *temp_paras, 
                        state_trans_paras_struct *state_trans_paras, 
                        float *sensor, float delta_T)
{
	// 准备参数
	matrix_set_data(kalman_basic_paras->measurement_z_mat, sensor, 9);
	
	measure_preprocess(kalman_basic_paras->measurement_z_mat, temp_paras);
	
  prior_predict(kalman_basic_paras, temp_paras, state_trans_paras, delta_T);
	
  post_correct(kalman_basic_paras, temp_paras);
	
	build_quaternion(kalman_basic_paras, quaternion_paras, state_trans_paras);
	
	rotation_to_quaterion(quaternion_paras, quaternion);
}
