#include "ekf.h"

// 实验当地的重力加速度，通过经纬度可以查询得到
//float g0;
// 状态向量
matrix *x_61;
// DCM状态（最下面那行向量）方差，也就是陀螺测量噪音方差
//float q_dcm2;
// 陀螺仪偏差的方差
//float q_gyro_bias2;
// 加速度计测量噪音的方差
//float r_acc2;
// 加速度方差
//float r_a2;
matrix *a_31;
//float yaw;
//float pitch;
//float roll;
matrix *P_66;
matrix *H_36;
matrix *Q_66;
matrix *first_row_31;
matrix *Q_66_temp;
matrix *u;
matrix *c3x;
matrix *c3v;
matrix *ux;
matrix *temp3v;
matrix *c3x_temp;
matrix *A;
matrix *B;
matrix *F;
matrix *temp61;
matrix *x_predict;
matrix *P_predict;
matrix *F_transpose;
matrix *temp66;
matrix *z;
matrix *temp31;
matrix *a_predict;
matrix *y;
matrix *S;
matrix *H_transpose;
matrix *temp36;
matrix *S_inverse;
matrix *K;
matrix *R;
matrix *i33;
matrix *temp63;
matrix *x_last;
matrix *i66;
matrix *IKH;
matrix *IKH_transpose;
matrix *K_transpose;
matrix *J;
matrix *J_transpose;
matrix *u_nb;
matrix *X1;
matrix *X2;
matrix *ux_transpose;
matrix *c3x_transpose;

//	DCM_IMU_uC(const float Gravity = DEFAULT_g0, const float *State = NULL, const float *Covariance = NULL,
//			const float DCMVariance = DEFAULT_q_dcm2, const float BiasVariance = DEFAULT_q_gyro_bias2,
//			const float InitialDCMVariance = DEFAULT_q_dcm2_init, const float InitialBiasVariance = DEFAULT_q_gyro_bias2_init,
//			const float MeasurementVariance = DEFAULT_r_acc2, const float MeasurementVarianceVariableGain = DEFAULT_r_a2);
  void DCM_IMU_uC_init_m(const float Gravity, const float *State, const float *Covariance,
		const float DCMVariance, const float BiasVariance,
		const float InitialDCMVariance, const float InitialBiasVariance,
		const float MeasurementVariance, const float MeasurementVarianceVariableGain){
			int i;
			float temp[] = DEFAULT_state;
			g0 = Gravity;
	    q_dcm2 = DCMVariance;
	    q_gyro_bias2 = BiasVariance;
	    r_acc2 = MeasurementVariance;
	    r_a2 = MeasurementVarianceVariableGain;
			x_61 = matrix_init(6, 1);
			a_31 = matrix_init(3, 1);
			P_66 = matrix_init(6, 6);
			H_36 = matrix_init(3, 6);
			Q_66 = matrix_init(6, 6);
			first_row_31 = matrix_init(3, 1);
		  for (i=0; i < 6; ++i) {
				x_61->data[i] = temp[i];
			}
			matrix_write(P_66, 0, 0, InitialDCMVariance);
			matrix_write(P_66, 1, 1, InitialDCMVariance);
			matrix_write(P_66, 2, 2, InitialDCMVariance);
			matrix_write(P_66, 3, 3, InitialBiasVariance);
			matrix_write(P_66, 4, 4, InitialBiasVariance);
			matrix_write(P_66, 5, 5, InitialBiasVariance);
			
			matrix_write(H_36, 0, 0, g0);
			matrix_write(H_36, 1, 1, g0);
			matrix_write(H_36, 2, 2, g0);
			
			matrix_write(Q_66, 0, 0, q_dcm2);
			matrix_write(Q_66, 1, 1, q_dcm2);
			matrix_write(Q_66, 2, 2, q_dcm2);
			matrix_write(Q_66, 3, 3, q_gyro_bias2);
			matrix_write(Q_66, 4, 4, q_gyro_bias2);
			matrix_write(Q_66, 5, 5, q_gyro_bias2);
			
			matrix_write(first_row_31, 0, 0, 1.0f);
			matrix_write(first_row_31, 2, 0, 0.0f);
			matrix_write(first_row_31, 3, 0, 0.0f);
			yaw = 0.0f;
			pitch = 0.0f;
			roll = 0.0f;
			Q_66_temp = matrix_init(6, 6);
			u = matrix_init(3, 1);
			c3x = matrix_init(3, 3);
		  c3v = matrix_init(3, 1);
			ux = matrix_init(3, 3);
			temp3v = matrix_init(3, 1);
			c3x_temp = matrix_init(3, 3);
			A = matrix_init(6, 6);
			B = matrix_init(6, 3);
			F = matrix_init(6, 6);
			temp61 = matrix_init(6, 1);
		  x_predict = matrix_init(6, 1);
			P_predict = matrix_init(6, 6);
		  F_transpose = matrix_init(6, 6);
			temp66 = matrix_init(6, 6);
			z = matrix_init(3, 1);
			temp31 = matrix_init(3, 1);
			a_predict = matrix_init(3, 1);
			y = matrix_init(3, 1);
			S = matrix_init(3, 3);
		  H_transpose = matrix_init(6, 3);
			temp36 = matrix_init(3, 6);
			S_inverse = matrix_init(3, 3);
			K = matrix_init(6, 3);
			R = matrix_init(3, 3);
		  i33 = matrix_create_identity(3);
			temp63 = matrix_init(6, 3);
			x_last = matrix_init(6, 1);
			i66 = matrix_create_identity(6);
			IKH = matrix_init(6, 6);
			IKH_transpose = matrix_init(6, 6);
			K_transpose = matrix_init(3, 6);
			J = matrix_init(6, 6);
			J_transpose = matrix_init(6, 6);
			u_nb = matrix_init(3, 1);
			X1 = matrix_init(3, 1);
		  X2 = matrix_init(3, 1);
			ux_transpose = matrix_init(3, 3);
			c3x_transpose = matrix_init(3, 3);
		}
	//! A method to perform update and give new measurements.
	/*!
	 *  This method is used regularly to update new gyroscope and accelerometer measurements into the system. To get best performance of the filter, please calibrate accelerometer and gyroscope readings before sending them into this method. The calibration process is documented in http://dx.doi.org/10.1155/2015/503814
	 *  In addition, please measure the used sample period as accurately as possible for each iteration (delay between current and the last data which was used at the previous update)
	 *  All parameters are in SI-units.
	 *
	 *  @param Gyroscope an array of gyroscope measurements (the length is 3 floats, angular velocities around x, y and z axis).
	 *  @param Accelerometer an array of accelerometer measurements (the length is 3 floats, accelerations in x, y and z axis).
	 *  @param SamplePeriod A delay between this measurement and the previous measurement in seconds.
	 */
	void updateIMU_m(float *Gyroscope, float *Accelerometer, float SamplePeriod){
		float a_len;
		float d;
		// Process noise covariance with time dependent noise
		
		matrix_set_data(Q_66_temp, Q_66->data, 36);
		matrix_multiply_scalar(Q_66_temp, SamplePeriod*SamplePeriod);
		// Control input (angular velocities from gyroscopes)
		
		matrix_set_data(u, Gyroscope, 3);
		
		// "rotation operators"
		
		matrix_write(c3v, 0, 0, x_61->data[0]);
		matrix_write(c3v, 1, 0, x_61->data[1]);
		matrix_write(c3v, 2, 0, x_61->data[2]);
		vector3_to_skew_mat(c3v, c3x);
		
		
		
		matrix_write(temp3v, 0, 0, (-1)*x_61->data[3]+u->data[0]);
		matrix_write(temp3v, 1, 0, (-1)*x_61->data[4]+u->data[1]);
		matrix_write(temp3v, 2, 0, (-1)*x_61->data[5]+u->data[2]);
		vector3_to_skew_mat(temp3v, ux);
		
		// Model generation
		
		matrix_set_data(c3x_temp, c3x->data, 9);
		matrix_multiply_scalar(c3x_temp, (-1)*SamplePeriod);
		matrix_set_zero(A);
		matrix_padding(A, c3x_temp, 0, 3, 1);
		matrix_set_zero(B);
		matrix_multiply_scalar(c3x_temp, -1);
		matrix_padding(B, c3x_temp, 0, 0, 1);
		matrix_set_zero(F);
		matrix_multiply_scalar(c3x_temp, -1);
		matrix_padding(F, c3x_temp, 0, 3, 1);
		
		matrix_set_data(c3x_temp, ux->data, 9);
		matrix_multiply_scalar(c3x_temp, (-1)*SamplePeriod);
		matrix_padding(F, c3x_temp, 0, 0, 1);
		
		
		matrix_add(F, i66, F);
		
		// Kalman a priori prediction
		
		matrix_multiply(A, x_61, temp61);
		matrix_add(x_61, temp61, x_predict);
		matrix_multiply(B, u, temp61);
		matrix_add(x_predict, temp61, x_predict);
		
		
		matrix_transpose(F, F_transpose);
		
		matrix_multiply(F, P_66, P_predict);
		matrix_multiply(P_predict, F_transpose, temp66);
		matrix_add(temp66, Q_66_temp, P_predict);
		
		// measurements/observations (acceleromeres)
		
		matrix_set_data(z, Accelerometer, 3);
		
		// recompute R using the error between acceleration and the model of g
	// (estimate of the magnitude of a0 in a = a0 + g)
	  
		
		matrix_padding(x_predict, temp31, 0, 0, 0);
		matrix_multiply_scalar(temp31, g0);
		matrix_substract(z, temp31, a_predict);
		
		a_len = sqrtf(a_predict->data[0]*a_predict->data[0]+a_predict->data[1]*a_predict->data[1]+a_predict->data[2]*a_predict->data[2]);
		
		
		matrix_set_data(R, i33->data, 9);
		matrix_multiply_scalar(R, a_len*r_a2+r_acc2);
		
		// Kalman innovation
		
		matrix_multiply(H_36, x_predict, y);
		matrix_substract(z, y, y);
		
		
		matrix_transpose(H_36, H_transpose);
		
		matrix_multiply(H_36, P_predict, temp36);
		matrix_multiply(temp36, H_transpose, S);
		matrix_add(S, R, S);
		
		// Kalman gain
		
		matrix_inverse_gauss(S, S_inverse);
		
		
		matrix_multiply(P_predict, H_transpose, temp63);
		matrix_multiply(temp63, S_inverse, K);
		
		
		matrix_set_data(x_last, x_61->data, 6);
//		printf_matrix(x_61);
		// 后验校正
		matrix_multiply(K, y, temp61);
		matrix_add(temp61, x_predict, x_61);
		
		// 更新后验校正后的协方差
		
		
		matrix_multiply(K, H_36, temp66);
		matrix_substract(i66, temp66, IKH);
		matrix_transpose(IKH, IKH_transpose);
		matrix_multiply(IKH, P_predict, temp66);
		matrix_multiply(temp66, IKH_transpose, P_66);
		matrix_multiply(K, R, temp63);
		
		matrix_transpose(K, K_transpose);
		matrix_multiply(temp63, K_transpose, temp66);
		matrix_add(P_66, temp66, P_66);
		
		// normalization of x & P (divide by DCM vector length)
		d = sqrtf(x_61->data[0]*x_61->data[0]+x_61->data[1]*x_61->data[1]+x_61->data[2]*x_61->data[2]);
		matrix_set_data(J, i66->data, 36);
		matrix_write(J, 0, 0, (x_61->data[1]*x_61->data[1]+x_61->data[2]*x_61->data[2])/(d*d*d));
		matrix_write(J, 0, 1, ((-1)*x_61->data[0]*x_61->data[1])/(d*d*d));
		matrix_write(J, 0, 2, ((-1)*x_61->data[0]*x_61->data[2])/(d*d*d));
		matrix_write(J, 1, 0, ((-1)*x_61->data[0]*x_61->data[1])/(d*d*d));
		matrix_write(J, 1, 1, (x_61->data[0]*x_61->data[0]+x_61->data[2]*x_61->data[2])/(d*d*d));
		matrix_write(J, 1, 2, ((-1)*x_61->data[1]*x_61->data[2])/(d*d*d));
		matrix_write(J, 2, 0, ((-1)*x_61->data[0]*x_61->data[2])/(d*d*d));
		matrix_write(J, 2, 1, ((-1)*x_61->data[1]*x_61->data[2])/(d*d*d));
		matrix_write(J, 2, 2, (x_61->data[0]*x_61->data[0]+x_61->data[1]*x_61->data[1])/(d*d*d));
		
		// Laplace approximation of normalization function for x to P, J = Jacobian(f,x)
		
		matrix_transpose(J, J_transpose);
		matrix_multiply(J, P_66, temp66);
		matrix_multiply(temp66, J_transpose, P_66);
		
		matrix_padding(x_61, temp31, 0, 0, 0);
		matrix_multiply_scalar(temp31, 1/d);
		matrix_padding(x_61, temp31, 0, 0, 1);
		
		// compute Euler angles (not exactly a part of the extended Kalman filter)
	  // yaw integration through full rotation matrix
		
		matrix_padding(x_61, temp31, 3, 0, 0);
		matrix_substract(u, temp31, u_nb);
		
		//alternative method estimating the whole rotation matrix
		//integrate full rotation matrix (using first row estimate in memory)
		
		matrix_set_zero(X1);
		matrix_set_zero(X2);
		matrix_transpose(ux, ux_transpose);
		matrix_multiply(ux_transpose, first_row_31, X1);
		matrix_multiply_scalar(X1, SamplePeriod);
		matrix_add(first_row_31, X1, X1);
		matrix_multiply(c3x, X1, X2);
		matrix_multiply_scalar(X2, 1/sqrtf(X2->data[0]*X2->data[0]+X2->data[1]*X2->data[1]+X2->data[2]*X2->data[2]));
		
		matrix_transpose(c3x, c3x_transpose);
		matrix_multiply(c3x_transpose, X2, X1);
		matrix_set_data(temp31, X1->data, 3);
		matrix_multiply_scalar(temp31, 1/sqrtf(X1->data[0]*X1->data[0]+X1->data[1]*X1->data[1]+X1->data[2]*X1->data[2]));
		matrix_set_data(first_row_31, temp31->data, 3);
		yaw = atan2(X2->data[0], first_row_31->data[0]);
		yaw = yaw;
//		printf("\r\nX2->data[0]:%f", X2->data[0]);
//		printf("\r\nfirst_row_31->data[0]:%f", first_row_31->data[0]);
		pitch = asin((-1)*x_61->data[0]);
		roll = atan2(x_61->data[1], x_61->data[2]);
//		printf_matrix(x_61);
		
	}

	//! A method to query State.
	/*!
	 *  @param State a 6 units long float array where the current state is stored.
	 */
	void getState_m(float *State){
		
	}

	//! A method to query Covariance.
	/*!
	 *  @param Covariance a 36 units long float array where the current covariance is stored in row-major order.
	 */
	void getCovariance_m(float *Covariance){
		
	}

	//! A method to query non-gravitational acceleration.
	/*!
	 *  @param a a 3 units long float array where the current non-gravitational acceleration is stored (x, y, and z axis).
	 */
	void getNGAcc_m(float *a){
		
	}

	//! A method to return the yaw angle.
	/*!
	 *  @return The yaw angle.
	 */
	float getYaw_m(void){
		return yaw;
	}

	//! A method to return the pitch angle.
	/*!
	 *  @return The pitch angle.
	 */
	float getPitch_m(void){
		return pitch;
	}

	//! A method to return the roll angle.
	/*!
	 *  @return The roll angle.
	 */
	float getRoll_m(void){
		return roll;
	}
