//============================================================================
// Copyright (C) 2020, Heikki Hyyti
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//============================================================================

#ifndef __DCM_H
#define __DCM_H

/**
 * Includes
 */
#include "math.h"
#include "malloc.h"


/**
 * Default constant values if they are not set in the constructor
 */
// ��ɳ��9.7915
// ˳����9.7833
#define DEFAULT_g0 9.7833
#define DEFAULT_state {0,0,1,0,0,0}
#define DEFAULT_q_dcm2 (0.1*0.1)
#define DEFAULT_q_gyro_bias2 (0.01*0.01)
#define DEFAULT_r_acc2 (0.5*0.5)
#define DEFAULT_r_a2 (10*10)
#define DEFAULT_q_dcm2_init (1*1)
#define DEFAULT_q_gyro_bias2_init (0.1*0.1)

#define VARIANCE_MIN_LIMIT (0.0001*0.0001) //set this to a small positive number or 0 to disable the feature.
#define VARIANCE_SAFETY_INCREMENT (0.00001*0.00001) //set this to a small positive number or 0 to disable the feature.
typedef struct{
	float Gravity;
	float *State;
	float *Covariance;
	float DCMVariance;
	float BiasVariance;
	float InitialDCMVariance;
	float InitialBiasVariance;
	float MeasurementVariance;
	float MeasurementVarianceVariableGain;
}DCM_IMU_uC;
//! DCM_IMU_uC class.
/*!
 *  The DCM-IMU algorithm is designed for fusing low-cost triaxial MEMS gyroscope and accelerometer measurements. An extended Kalman filter is used to estimate attitude in direction cosine matrix (DCM) formation and gyroscope biases online. A variable measurement covariance method is implemented for acceleration measurements to ensure robustness against transient non-gravitational accelerations which usually induce errors to attitude estimate in ordinary IMU-algorithms.
 *  If you use the algorithm in any scientific context, please cite: Heikki Hyyti and Arto Visala, "A DCM Based Attitude Estimation Algorithm for Low-Cost MEMS IMUs," International Journal of Navigation and Observation, vol. 2015, Article ID 503814, 18 pages, 2015. http://dx.doi.org/10.1155/2015/503814
 *  This updated version of micro controller c code has forced symmetry of covariance matrices that reduces computational complexity of the filter significantly. In addition, the code divides measurement with g0 before feeding it to filter which increases the stability because covariance matrix update is done with smaller weights (1 vs g0^6).
 *  For further safety, the variance of states is limited to a small postive value and a small non-corelating noise is added to each state to keep the filter stable against rounding errors. Both of these safety additions may be disabled by defining the values to 0 in this file.
 */
	//! DCM_IMU_uC constructor.
	/*!
	 *  Initializes DCM_IMU_uC either with default values or given parameters. All parameters are in SI-units.
         *
	 *  @param Gravity A magnitude of gravity
	 *  @param State An initial state as a array of six floats, DCM states and bias states.
	 *  @param Covariance A covariance matrix (size of 6x6 floats, array of 36 floats in row-major order). If a custom covariance matrix is given, parameters InitialDCMVariance and InitialBiasVariance are not used.
	 *  @param DCMVariance a variance for DCM state update, Q(0,0), Q(1,1), and Q(2,2)
	 *  @param BiasVariance a variance for bias state update, Q(3,3), Q(4,4), and Q(5,5)
	 *  @param InitialDCMVariance an initial variance for DCM state, P(0,0), P(1,1), and P(2,2). If Covariance matrix is given, this parameter is not used.
	 *  @param InitialBiasVariance an initial variance for bias state, P(3,3), P(4,4), and P(5,5). If Covariance matrix is given, this parameter is not used.
	 *  @param MeasurementVariance a constant part of the variance for measurement update, R(0,0), R(1,1), and R(2,2)
	 *  @param MeasurementVarianceVariableGain a gain for the variable part of the variance for measurement update, R(0,0), R(1,1), and R(2,2)
	 */
//	DCM_IMU_uC(const float Gravity = DEFAULT_g0, const float *State = NULL, const float *Covariance = NULL,
//			const float DCMVariance = DEFAULT_q_dcm2, const float BiasVariance = DEFAULT_q_gyro_bias2,
//			const float InitialDCMVariance = DEFAULT_q_dcm2_init, const float InitialBiasVariance = DEFAULT_q_gyro_bias2_init,
//			const float MeasurementVariance = DEFAULT_r_acc2, const float MeasurementVarianceVariableGain = DEFAULT_r_a2);
  void DCM_IMU_uC_init(const float Gravity, const float *State, const float *Covariance,
		const float DCMVariance, const float BiasVariance,
		const float InitialDCMVariance, const float InitialBiasVariance,
		const float MeasurementVariance, const float MeasurementVarianceVariableGain);
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
	void updateIMU(const float *Gyroscope, const float *Accelerometer, const float SamplePeriod);

	//! A method to query State.
	/*!
	 *  @param State a 6 units long float array where the current state is stored.
	 */
	void getState(float *State);

	//! A method to query Covariance.
	/*!
	 *  @param Covariance a 36 units long float array where the current covariance is stored in row-major order.
	 */
	void getCovariance(float *Covariance);

	//! A method to query non-gravitational acceleration.
	/*!
	 *  @param a a 3 units long float array where the current non-gravitational acceleration is stored (x, y, and z axis).
	 */
	void getNGAcc(float *a);

	//! A method to return the yaw angle.
	/*!
	 *  @return The yaw angle.
	 */
	float getYaw(void);

	//! A method to return the pitch angle.
	/*!
	 *  @return The pitch angle.
	 */
	float getPitch(void);

	//! A method to return the roll angle.
	/*!
	 *  @return The roll angle.
	 */
	float getRoll(void);

extern float g0, inv_g0, inv_g0_2;
extern float x0, x1, x2, x3, x4, x5;
extern float q_dcm2;
extern float q_gyro_bias2;
extern float r_acc2;
extern float r_a2;
extern float a0, a1, a2;
extern float yaw, pitch, roll;
extern float P00, P01, P02, P03, P04, P05;
extern float P11, P12, P13, P14, P15;
extern float P22, P23, P24, P25;
extern float P33, P34, P35;
extern float P44, P45;
extern float P55;
extern float fr0, fr1, fr2;

#endif /* __DCM_H */
