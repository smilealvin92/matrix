#include "kalman.h"

// �ο��Ĵ����ַ��https://github.com/gaochq/IMU_Attitude_Estimator

// ��������������3�׷��Գƾ��󣬳ɹ��򷵻�0��ʧ���򷵻�-1
int vector3_to_skew_mat(matrix *val, matrix *result)
{
	if(result && result->data)
	{
		// 0��1��
		result->data[1] = (-1)*(val->data[2]);
		// 1��0��
		result->data[3] = val->data[2];
		// 0��2��
		result->data[2] = val->data[1];
		// 2��0��
		result->data[6] = (-1)*(val->data[1]);
		// 1��2��
		result->data[5] = (-1)*(val->data[0]);
		// 2��1��
		result->data[7] = val->data[0];
		return 0;
	}
	else
	{
		printf("memory malloc failed!");
	}
	return -1;
}

// ��ʼ�����õ���ʼ��״̬������״̬��Э�������״̬ת�ƹ��̵��������󡢲�����������
// ״̬ת�ƾ��󡢹۲����ȵ�
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

// ��ʼ����Ԫ���ṹ��
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

// ��ʼ��״̬ת�ƾ���
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

// ��ʼ���������˲�������������
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
		// 1.�������������ֵ����Z
		kalman_basic_paras->measurement_z_mat = matrix_init(9, 1);
		// 2.��ʼ��״̬����X
	  kalman_basic_paras->state_x_mat = variable_mat_init(state_x, 12, 1, "state_x_mat");
		// 3.��ʼ��״̬������Э�������
		kalman_basic_paras->covaria_p_mat = variable_mat_init(covaria_p, 12, 12, "covaria_p_mat");
		// 4.��ʼ��״̬����ת�ƹ����е�ת����������
		kalman_basic_paras->tran_noise_q_mat = create_diagonal(tran_noise_q, 12);
		// 5.��ʼ���������߹۲�����еĲ�����������
		kalman_basic_paras->measure_noise_r_mat = create_diagonal(measure_noise_r, 9);
		// 6.��ʼ���۲����˵��������
		kalman_basic_paras->obser_h_mat = variable_mat_init(obser_h, 9, 12, "obser_h_mat");
		// 7.����������
		kalman_basic_paras->kalman_gain = matrix_init(12, 9);
		// 8.��ʼ��״̬ת�ƾ���
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

// ��ʼ�������������м̵ľ���
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

// ��ʼ������������Ԫ���ľ���
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

// ����Ԥ��
void prior_predict(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras, state_trans_paras_struct *state_trans_paras, float delta_T)
{
	// ���㱾���˲����������״̬ת�ƾ���
	// �ȴӾɵ�״̬������ȡ�����ݣ�׼������
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_vel, 0, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_acc, 3, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->acc_state, 6, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->mag_state, 9, 0, 0);
//	printf("\r\nbegin!");
//	printf("\r\nstate_x_mat");
//	printf_matrix(kalman_basic_paras->state_x_mat);
	vector3_to_skew_mat(state_trans_paras->angular_vel, state_trans_paras->w_angular);
	matrix_multiply_scalar( state_trans_paras->w_angular, -1);
	// ������ʱ�м̽ṹ�壬�����r_acc
	vector3_to_skew_mat(state_trans_paras->acc_state, temp_paras->temp3_3);
	matrix_padding(kalman_basic_paras->state_tran_f_mat, temp_paras->temp3_3, 6, 0, 1);
	
	// ����Ҫ��һ������r_mag�����Գ���-1
	vector3_to_skew_mat(state_trans_paras->mag_state, temp_paras->temp3_3);
	matrix_multiply_scalar( temp_paras->temp3_3, -1);
	matrix_padding(kalman_basic_paras->state_tran_f_mat, temp_paras->temp3_3, 9, 0, 1);
	// ��������䵽�µ�״̬ת�ƾ�����
	matrix_padding(kalman_basic_paras->state_tran_f_mat, state_trans_paras->w_angular, 6, 6, 1);
	matrix_padding(kalman_basic_paras->state_tran_f_mat, state_trans_paras->w_angular, 9, 9, 1);
	
	// �����֮�󣬿�ʼ���㣬�˴�����ʹ��һ���м̾�������֤��һ���˲������У�״̬ת�ƾ���ĶԽ���ȻΪ0
	matrix_padding(kalman_basic_paras->inter_state_tran_f_mat, kalman_basic_paras->state_tran_f_mat, 0, 0, 1);
	matrix_multiply_scalar( kalman_basic_paras->inter_state_tran_f_mat, delta_T);
	matrix_add(kalman_basic_paras->inter_state_tran_f_mat, temp_paras->i12, kalman_basic_paras->inter_state_tran_f_mat);
//	printf("\r\nstate_tran_f_mat");
//	printf_matrix(kalman_basic_paras->state_tran_f_mat);
	// ״̬ת�ƾ���׼����֮�󣬽���״̬������Ԥ��
	// �Ƚ��Ǽ��ٶ������µ�״̬��������Ȼ�����ˣ��Ͳ���ԭ���ĽǼ��ٶ��ˣ���ֵ���Ѿ����ı��ˣ����ֵ�ӿ�ʼ���������һֱû�б�
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_acc, 3, 0, 1);
	// �ٽ����ٶ������µ�״̬����
	matrix_multiply_scalar( state_trans_paras->angular_acc, delta_T);
	matrix_add(state_trans_paras->angular_vel, state_trans_paras->angular_acc, state_trans_paras->angular_vel);
	
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->angular_acc, 0, 0, 1);
//	printf_matrix(state_trans_paras->angular_acc);
//	printf_matrix(state_trans_paras->mag_state);
	// �ټ���acc_state��mag_state����ʱʹ�ö��׷�
	matrix_multiply(state_trans_paras->w_angular, state_trans_paras->w_angular, temp_paras->temp3_3);
	matrix_multiply_scalar( temp_paras->temp3_3, delta_T*delta_T/2);
	matrix_multiply_scalar( state_trans_paras->w_angular, delta_T);// �˴�w_angular�ѱ��ı�
	matrix_add(state_trans_paras->w_angular, temp_paras->temp3_3, temp_paras->temp3_3);
	matrix_add(temp_paras->i3, temp_paras->temp3_3, temp_paras->temp3_3);

	// ������ʱ�м̽ṹ�壬��ɾ�����˵ļ���
	matrix_padding(state_trans_paras->acc_state, temp_paras->temp3_1, 0, 0, 0);
	matrix_multiply(temp_paras->temp3_3, temp_paras->temp3_1, state_trans_paras->acc_state);
	matrix_padding(state_trans_paras->mag_state, temp_paras->temp3_1, 0, 0, 0);
	matrix_multiply(temp_paras->temp3_3, temp_paras->temp3_1, state_trans_paras->mag_state);
	
	
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->acc_state, 6, 0, 1);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->mag_state, 9, 0, 1);
//	printf("\r\nstate_x_mat");
//	printf_matrix(kalman_basic_paras->state_x_mat);
//	printf("end!");
	// ��ʱ�µ�״̬�����Ѿ�������ϣ���ʼ����Э�������
	// �˴���ʽ��P = F * P * Fת�� + Q
	matrix_multiply(kalman_basic_paras->inter_state_tran_f_mat, kalman_basic_paras->covaria_p_mat, temp_paras->temp_0_12_12);
	// ״̬ת�ƾ����ת�÷�����ʱ�м̽ṹ����
	matrix_transpose(kalman_basic_paras->inter_state_tran_f_mat, temp_paras->temp_1_12_12);
	matrix_multiply(temp_paras->temp_0_12_12, temp_paras->temp_1_12_12, kalman_basic_paras->covaria_p_mat);
	matrix_add(kalman_basic_paras->covaria_p_mat, kalman_basic_paras->tran_noise_q_mat, kalman_basic_paras->covaria_p_mat);
//	printf("\r\ntran_noise_q_mat");
//  printf_matrix(kalman_basic_paras->tran_noise_q_mat);
}

// �������У��
void post_correct(kalman_basic_paras_struct *kalman_basic_paras, temp_paras_struct *temp_paras)
{
	// ����У��
	// �ѹ۲�����ת�ô�����ʱ�м̽ṹ����
	// �ȼ��㿨��������
	// �˴���ʽ K = P * Hת�� * (H * P * Hת�� + R)��
	matrix_transpose(kalman_basic_paras->obser_h_mat, temp_paras->temp_0_12_9);
	matrix_multiply(kalman_basic_paras->obser_h_mat, kalman_basic_paras->covaria_p_mat, temp_paras->temp9_12);
//	printf("temp_paras->temp_0_9_9->ncolumn: %d\n", temp_paras->temp_0_9_9->ncolumn);
//	printf("temp_paras->temp_0_9_9->nrow: %d\n", temp_paras->temp_0_9_9->nrow);
	matrix_multiply(temp_paras->temp9_12, temp_paras->temp_0_12_9, temp_paras->temp_0_9_9);
	matrix_add(temp_paras->temp_0_9_9, kalman_basic_paras->measure_noise_r_mat, temp_paras->temp_0_9_9);
	
	// ���㿨���������ʱ�򣬻����һ��������������������ʱ�м̽ṹ����
	matrix_inverse_gauss(temp_paras->temp_0_9_9, temp_paras->temp_1_9_9);
//	printf("\r\ntemp_1_9_9");
//	printf_matrix(temp_paras->temp_1_9_9);
//	printf("hereabcdefg!!!");
	matrix_multiply(kalman_basic_paras->covaria_p_mat, temp_paras->temp_0_12_9, temp_paras->temp_1_12_9);
	matrix_multiply(temp_paras->temp_1_12_9, temp_paras->temp_1_9_9, kalman_basic_paras->kalman_gain);
//	printf("\r\nkalman_gain");
//	printf_matrix(kalman_basic_paras->kalman_gain);
	// ����״̬����
	// �˴���ʽ X = X + K(Z - H * X)
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
	// ����״̬������Э�������
	// �˴���ʽ P = (I - K * H) * P
	matrix_multiply(kalman_basic_paras->kalman_gain, kalman_basic_paras->obser_h_mat, temp_paras->temp_0_12_12);
	matrix_multiply_scalar( temp_paras->temp_0_12_12, -1);
	matrix_add(temp_paras->temp_0_12_12, temp_paras->i12, temp_paras->temp_0_12_12);
	matrix_multiply(temp_paras->temp_0_12_12, kalman_basic_paras->covaria_p_mat, temp_paras->temp_1_12_12);
	matrix_padding(kalman_basic_paras->covaria_p_mat, temp_paras->temp_1_12_12, 0, 0, 1);
	
}

// ������Ԫ��
void build_quaternion(kalman_basic_paras_struct *kalman_basic_paras, quaternion_paras_struct *quaternion_paras, state_trans_paras_struct *state_trans_paras)
{
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->acc_state, 6, 0, 0);
	matrix_padding(kalman_basic_paras->state_x_mat, state_trans_paras->mag_state, 9, 0, 0);
//	printf("\r\nstate_trans_paras->mag_state->data[0]: %f", state_trans_paras->mag_state->data[0]);
	vector_normalize(state_trans_paras->acc_state);
	vector_normalize(state_trans_paras->mag_state);
	
	// kalman_paras->r_acc_v����z_state;
	matrix_multiply_scalar( state_trans_paras->acc_state, -1);
	matrix_padding(quaternion_paras->z_state, state_trans_paras->acc_state, 0, 0, 1);
	
	matrix_cross_product_3(quaternion_paras->z_state, state_trans_paras->mag_state, quaternion_paras->y_state);
	vector_normalize(quaternion_paras->y_state);
	matrix_cross_product_3(quaternion_paras->y_state, quaternion_paras->z_state, quaternion_paras->x_state);
	vector_normalize(quaternion_paras->x_state);
	
	// �˴�����ת˳����X-Y-Z
	
	matrix_padding(quaternion_paras->rotation_matrix, quaternion_paras->x_state, 0, 0, 1);
	matrix_padding(quaternion_paras->rotation_matrix, quaternion_paras->y_state, 0, 1, 1);
	matrix_padding(quaternion_paras->rotation_matrix, quaternion_paras->z_state, 0, 2, 1);
	// �˴���w_angular��һ����ʱ���м̽ṹ�壬��������������ʱ�ò�����
	matrix_transpose(quaternion_paras->rotation_matrix, state_trans_paras->w_angular);
	matrix_padding(quaternion_paras->rotation_matrix, state_trans_paras->w_angular, 0, 0, 1);
}

// ����ת����תΪ��Ԫ��
void rotation_to_quaterion( quaternion_paras_struct *quaternion_paras, quaternion_struct *quaternion)
{
	// ��ת�����ǿ�������Ԫ�����(w, xi+yj+zk)��w,x,y,zд�ɵģ���ˣ���������ת����֮�󣬿��Է�����Ԫ��
	// ��ʽ�ǣ�w = ����(trace(R)+1)/2��x = (m21-m12)/(4w)��y = (m02-m20)/(4w)��x = (m10-m01)/(4w)
	float trace = 0.0f;
	matrix *rotation = quaternion_paras->rotation_matrix;
	trace = matrix_trace(rotation);
//	printf("\r\ntrace: %f", trace);
	quaternion->w = sqrtf(trace+1)/2;
	quaternion->xi = (rotation->data[2*rotation->ncolumn+1]-rotation->data[1*rotation->ncolumn+2])/(4*quaternion->w);
	quaternion->yj = (rotation->data[0*rotation->ncolumn+2]-rotation->data[2*rotation->ncolumn+0])/(4*quaternion->w);
	quaternion->zk = (rotation->data[1*rotation->ncolumn+0]-rotation->data[0*rotation->ncolumn+1])/(4*quaternion->w);	
}

// ��ԭʼ�������ݽ���Ԥ������Ҫ�ǰ������Ǻͼ��ٶȼƻ���λ�ã����ٶȼƺʹ����Ƶ�����Ҫ���й�һ��
void measure_preprocess(matrix *measure, temp_paras_struct *temp_paras)
{
	// acc_measure
	matrix_padding(measure, temp_paras->temp3_1, 0, 0, 0);
	// gyro_measure
	matrix_padding(measure, temp_paras->temp_1_3_1, 3, 0, 0);
	// ���ٶȼ����ݹ�һ��
	vector_normalize(temp_paras->temp3_1);
	// ���ٶȼƺ������ǻ���λ��
	// acc_measure
	matrix_padding(measure, temp_paras->temp3_1, 3, 0, 1);
	// gyro_measure
	matrix_padding(measure, temp_paras->temp_1_3_1, 0, 0, 1);
	
	// mag_measure��һ��
	matrix_padding(measure, temp_paras->temp3_1, 6, 0, 0);
	vector_normalize(temp_paras->temp3_1);
	matrix_padding(measure, temp_paras->temp3_1, 6, 0, 1);
}

// kalman�㷨���Ժ���
// ����ԭ�д��������Ĵ��������ݵ���ʽ�����������ǵ�6050���ݣ�������������ʵ��
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
		// ��ȡ�µĲ������ݣ��ٽ��п������˲�
		// sensor = ......
		kalman_loop(quaternion_paras, quaternion, kalman_basic_paras, temp_paras, 
	            state_trans_paras, sensor, delta_T);
		// Ȼ�����ŷ����
		// ��ת�������ת˳����Z-Y-X
		w = quaternion->w;
		xi = quaternion->xi;
		yj = quaternion->yj;
		zk = quaternion->zk;
		roll = atan2(2 * (yj*zk + w*xi), (w*w - xi*xi - yj*yj + zk*zk));
    pitch = asin(-2 * xi*zk + 2 * w*yj);
    yaw = atan2(2 * (xi*yj + w*zk), (w*w + xi*xi - yj*yj - zk*zk)) - 8.3f*M_PI/180.0f;
		printf("\nroll: %f��   pitch: %f��   yaw: %f��   ", roll*(180.0f/M_PI), pitch*(180.0f/M_PI), yaw*(180.0f/M_PI));
	}
}

// kalman�˲���ѭ������
void kalman_loop(quaternion_paras_struct *quaternion_paras,
                        quaternion_struct *quaternion,
                        kalman_basic_paras_struct *kalman_basic_paras, 
                        temp_paras_struct *temp_paras, 
                        state_trans_paras_struct *state_trans_paras, 
                        float *sensor, float delta_T)
{
	// ׼������
	matrix_set_data(kalman_basic_paras->measurement_z_mat, sensor, 9);
	
	measure_preprocess(kalman_basic_paras->measurement_z_mat, temp_paras);
	
  prior_predict(kalman_basic_paras, temp_paras, state_trans_paras, delta_T);
	
  post_correct(kalman_basic_paras, temp_paras);
	
	build_quaternion(kalman_basic_paras, quaternion_paras, state_trans_paras);
	
	rotation_to_quaterion(quaternion_paras, quaternion);
}
