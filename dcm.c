#include "dcm.h"

float g0, inv_g0, inv_g0_2;
float x0, x1, x2, x3, x4, x5;
float q_dcm2;
float q_gyro_bias2;
float r_acc2;
float r_a2;
float a0, a1, a2;
float yaw, pitch, roll;
float P00, P01, P02, P03, P04, P05;
float P11, P12, P13, P14, P15;
float P22, P23, P24, P25;
float P33, P34, P35;
float P44, P45;
float P55;
float fr0, fr1, fr2;


void DCM_IMU_uC_init(const float Gravity, const float *State, const float *Covariance,
		const float DCMVariance, const float BiasVariance,
		const float InitialDCMVariance, const float InitialBiasVariance,
		const float MeasurementVariance, const float MeasurementVarianceVariableGain){
	float temp[] = DEFAULT_state;
	int i;
	g0 = Gravity;
	q_dcm2 = DCMVariance;
	q_gyro_bias2 = BiasVariance;
	r_acc2 = MeasurementVariance;
	r_a2 = MeasurementVarianceVariableGain;
	
	if (State != NULL) {
		for (i = 0; i < 6; ++i) {
			temp[i] = State[i];
		}
	}
	x0 = temp[0];
	x1 = temp[1];
	x2 = temp[2];
	x3 = temp[3];
	x4 = temp[4];
	x5 = temp[5];

	if (Covariance == NULL) {
		P00 = InitialDCMVariance;
		P01 = 0;
		P02 = 0;
		P03 = 0;
		P04 = 0;
		P05 = 0;
		P11 = InitialDCMVariance;
		P12 = 0;
		P13 = 0;
		P14 = 0;
		P15 = 0;
		P22 = InitialDCMVariance;
		P23 = 0;
		P24 = 0;
		P25 = 0;
		P33 = InitialBiasVariance;
		P34 = 0;
		P35 = 0;
		P44 = InitialBiasVariance;
		P45 = 0;
		P55 = InitialBiasVariance;
	}
	else {
		P00 = Covariance[0];
		P01 = Covariance[1];
		P02 = Covariance[2];
		P03 = Covariance[3];
		P04 = Covariance[4];
		P05 = Covariance[5];
		P11 = Covariance[7];
		P12 = Covariance[8];
		P13 = Covariance[9];
		P14 = Covariance[10];
		P15 = Covariance[11];
		P22 = Covariance[14];
		P23 = Covariance[15];
		P24 = Covariance[16];
		P25 = Covariance[17];
		P33 = Covariance[21];
		P34 = Covariance[22];
		P35 = Covariance[23];
		P44 = Covariance[28];
		P45 = Covariance[29];
		P55 = Covariance[35];
	}

	inv_g0 = 1.0f / g0;
	inv_g0_2 = inv_g0 * inv_g0;
	yaw = 0.0f;
	pitch = 0.0f;
	roll = 0.0f;
	fr0 = 1.0f; //first row for alternative rotation computation
	fr1 = 0.0f; //default is yaw = 0 which happens when fr = [1, 0, 0]
	fr2 = 0.0f;
}


void updateIMU(const float *Gyroscope, const float *Accelerometer, const float h) {
	float x_last[3], u0, u1, u2, x_0, x_1, x_2, x_3, x_4, x_5, hh, P_00, P_01, P_02, P_03, P_04, P_05, P_11, P_12, P_13, P_14, P_15, P_22, P_23, P_24, P_25, P_33, P_34, P_35;
	float P_44, P_45, P_55, z0, z1, z2, y0, y1, y2, a_len, r_adab, S00, S01, S02, S11, S12, S22, invPart, K00, K01, K02, K10, K11, K12, K20, K21, K22, K30, K31, K32;
	float K40, K41, K42, K50, K51, K52, K00_1, K11_1, K22_1, common1, common2, common3, common4, common5, common6, common7, common8, common9, commonA, commonB, commonC;
	float P__00, P__01, P__02, P__03, P__04, P__05, P__11, P__12, P__13, P__14, P__15, P__22, P__23, P__24, P__25, P__33, P__34, P__35, P__44, P__45, P__55; 	
	float len, invlen3, invlen32, x1x1_x2x2, x0x0_x2x2, x0x0_x1x1, u_nb0, u_nb1, u_nb2;
	float cy, sy, d, d_inv, R11, R12, R13, R21, R22, R23, R11_new, R21_new, sr0, sr1, sr2, invlen, det_S;

	// save last state to memory for rotation estimation
	x_last[0] = x0;
	x_last[1] = x1;
	x_last[2] = x2;

	// control input (gyroscopes)
	u0 = Gyroscope[0];
	u1 = Gyroscope[1];
	u2 = Gyroscope[2];

	// state prediction
	x_0 = x0-h*(u1*x2-u2*x1+x1*x5-x2*x4);
	x_1 = x1+h*(u0*x2-u2*x0+x0*x5-x2*x3);
	x_2 = x2-h*(u0*x1-u1*x0+x0*x4-x1*x3);
	x_3 = x3;
	x_4 = x4;
	x_5 = x5;

	// covariance prediction
	hh = h*h;
	P_00 = P00-h*(P05*x1*2.0f-P04*x2*2.0f+P02*(u1-x4)*2.0f-P01*(u2-x5)*2.0f)-hh*(-q_dcm2+x2*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5))+x1*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5))+(u2-x5)*(P15*x1-P14*x2+P12*(u1-x4)-P11*(u2-x5))-(u1-x4)*(P25*x1-P24*x2+P22*(u1-x4)-P12*(u2-x5)));
	P_01 = P01+h*(P05*x0-P03*x2-P15*x1+P14*x2+P02*(u0-x3)-P12*(u1-x4)-P00*(u2-x5)+P11*(u2-x5))+hh*(x2*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5))+x0*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5))+(u2-x5)*(P05*x1-P04*x2+P02*(u1-x4)-P01*(u2-x5))-(u0-x3)*(P25*x1-P24*x2+P22*(u1-x4)-P12*(u2-x5)));
	P_02 = P02-h*(P04*x0-P03*x1+P25*x1-P24*x2+P01*(u0-x3)-P00*(u1-x4)+P22*(u1-x4)-P12*(u2-x5))-hh*(x1*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5))-x0*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5))+(u1-x4)*(P05*x1-P04*x2+P02*(u1-x4)-P01*(u2-x5))-(u0-x3)*(P15*x1-P14*x2+P12*(u1-x4)-P11*(u2-x5)));
	P_03 = P03-h*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5));
	P_04 = P04-h*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5));
	P_05 = P05+h*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5));
	P_11 = P11+h*(P15*x0*2.0f-P13*x2*2.0f+P12*(u0-x3)*2.0f-P01*(u2-x5)*2.0f)-hh*(-q_dcm2+x2*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5))+x0*(P35*x2-P55*x0-P25*(u0-x3)+P05*(u2-x5))+(u2-x5)*(P05*x0-P03*x2+P02*(u0-x3)-P00*(u2-x5))-(u0-x3)*(P25*x0-P23*x2+P22*(u0-x3)-P02*(u2-x5)));
	P_12 = P12-h*(P14*x0-P13*x1-P25*x0+P23*x2+P11*(u0-x3)-P01*(u1-x4)-P22*(u0-x3)+P02*(u2-x5))+hh*(x1*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5))-x0*(P45*x0-P34*x2+P24*(u0-x3)-P04*(u2-x5))+(u1-x4)*(P05*x0-P03*x2+P02*(u0-x3)-P00*(u2-x5))-(u0-x3)*(P15*x0-P13*x2+P12*(u0-x3)-P01*(u2-x5)));
	P_13 = P13+h*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5));
	P_14 = P14+h*(P45*x0-P34*x2+P24*(u0-x3)-P04*(u2-x5));
	P_15 = P15-h*(P35*x2-P55*x0-P25*(u0-x3)+P05*(u2-x5));
	P_22 = P22-h*(P24*x0*2.0f-P23*x1*2.0f+P12*(u0-x3)*2.0f-P02*(u1-x4)*2.0f)-hh*(-q_dcm2+x1*(P34*x0-P33*x1+P13*(u0-x3)-P03*(u1-x4))+x0*(P34*x1-P44*x0-P14*(u0-x3)+P04*(u1-x4))+(u1-x4)*(P04*x0-P03*x1+P01*(u0-x3)-P00*(u1-x4))-(u0-x3)*(P14*x0-P13*x1+P11*(u0-x3)-P01*(u1-x4)));
	P_23 = P23-h*(P34*x0-P33*x1+P13*(u0-x3)-P03*(u1-x4));
	P_24 = P24+h*(P34*x1-P44*x0-P14*(u0-x3)+P04*(u1-x4));
	P_25 = P25+h*(P35*x1-P45*x0-P15*(u0-x3)+P05*(u1-x4));
	P_33 = P33+hh*q_gyro_bias2;
	P_34 = P34;
	P_35 = P35;
	P_44 = P44+hh*q_gyro_bias2;
	P_45 = P45;
	P_55 = P55+hh*q_gyro_bias2;

	// measurements (accelerometers)
	z0 = Accelerometer[0] * inv_g0;
	z1 = Accelerometer[1] * inv_g0;
	z2 = Accelerometer[2] * inv_g0;

	// Kalman innovation
	y0 = z0-x_0;
	y1 = z1-x_1;
	y2 = z2-x_2;

	a_len = sqrtf(y0*y0+y1*y1+y2*y2) * g0;
	r_adab = (r_acc2 + a_len*r_a2) * inv_g0_2;

	// innovation covariance
	S00 = P_00 + r_adab;
	S01 = P_01;
	S02 = P_02;
	S11 = P_11 + r_adab;
	S12 = P_12;
	S22 = P_22 + r_adab;

        // verify that the innovation covariance is large enough
	if (S00 < VARIANCE_MIN_LIMIT) S00 = VARIANCE_MIN_LIMIT;
	if (S11 < VARIANCE_MIN_LIMIT) S11 = VARIANCE_MIN_LIMIT;
	if (S22 < VARIANCE_MIN_LIMIT) S22 = VARIANCE_MIN_LIMIT;

	// determinant of S
	det_S = -S00*(S12*S12) - (S02*S02)*S11 - (S01*S01)*S22 + S01*S02*S12*2.0f + S00*S11*S22;

	// Kalman gain
	invPart = 1.0f / det_S;
	K00 = -(S02*(P_02*S11-P_01*S12)-S01*(P_02*S12-P_01*S22)+P_00*(S12*S12)-P_00*S11*S22)*invPart;
	K01 = -(S12*(P_02*S00-P_00*S02)-S01*(P_02*S02-P_00*S22)+P_01*(S02*S02)-P_01*S00*S22)*invPart;
	K02 = -(S12*(P_01*S00-P_00*S01)-S02*(P_01*S01-P_00*S11)+P_02*(S01*S01)-P_02*S00*S11)*invPart;
	K10 = -(S02*(P_12*S11-P_11*S12)-S01*(P_12*S12-P_11*S22)+P_01*(S12*S12)-P_01*S11*S22)*invPart;
	K11 = -(S12*(P_12*S00-P_01*S02)-S01*(P_12*S02-P_01*S22)+P_11*(S02*S02)-P_11*S00*S22)*invPart;
	K12 = (S12*(P_01*S01-P_11*S00)+S02*(P_11*S01-P_01*S11)-P_12*(S01*S01)+P_12*S00*S11)*invPart;
	K20 = (S02*(P_12*S12-P_22*S11)+S01*(P_22*S12-P_12*S22)-P_02*(S12*S12)+P_02*S11*S22)*invPart;
	K21 = (S12*(P_02*S02-P_22*S00)+S01*(P_22*S02-P_02*S22)-P_12*(S02*S02)+P_12*S00*S22)*invPart;
	K22 = (S12*(P_02*S01-P_12*S00)+S02*(P_12*S01-P_02*S11)-P_22*(S01*S01)+P_22*S00*S11)*invPart;
	K30 = (S02*(P_13*S12-P_23*S11)+S01*(P_23*S12-P_13*S22)-P_03*(S12*S12)+P_03*S11*S22)*invPart;
	K31 = (S12*(P_03*S02-P_23*S00)+S01*(P_23*S02-P_03*S22)-P_13*(S02*S02)+P_13*S00*S22)*invPart;
	K32 = (S12*(P_03*S01-P_13*S00)+S02*(P_13*S01-P_03*S11)-P_23*(S01*S01)+P_23*S00*S11)*invPart;
	K40 = (S02*(P_14*S12-P_24*S11)+S01*(P_24*S12-P_14*S22)-P_04*(S12*S12)+P_04*S11*S22)*invPart;
	K41 = (S12*(P_04*S02-P_24*S00)+S01*(P_24*S02-P_04*S22)-P_14*(S02*S02)+P_14*S00*S22)*invPart;
	K42 = (S12*(P_04*S01-P_14*S00)+S02*(P_14*S01-P_04*S11)-P_24*(S01*S01)+P_24*S00*S11)*invPart;
	K50 = (S02*(P_15*S12-P_25*S11)+S01*(P_25*S12-P_15*S22)-P_05*(S12*S12)+P_05*S11*S22)*invPart;
	K51 = (S12*(P_05*S02-P_25*S00)+S01*(P_25*S02-P_05*S22)-P_15*(S02*S02)+P_15*S00*S22)*invPart;
	K52 = (S12*(P_05*S01-P_15*S00)+S02*(P_15*S01-P_05*S11)-P_25*(S01*S01)+P_25*S00*S11)*invPart;

//  printf("%f  ", x1);
//	printf("%f  \n", x2);
	// update a posteriori
	x0 = x_0 + K00*y0 + K01*y1 + K02*y2;
	x1 = x_1 + K10*y0 + K11*y1 + K12*y2;
	x2 = x_2 + K20*y0 + K21*y1 + K22*y2;
	x3 = x_3 + K30*y0 + K31*y1 + K32*y2;
	x4 = x_4 + K40*y0 + K41*y1 + K42*y2;
	x5 = x_5 + K50*y0 + K51*y1 + K52*y2;

	// update a posteriori covariance
	K00_1 = K00 - 1.0f;
	K11_1 = K11 - 1.0f;
	K22_1 = K22 - 1.0f;

	common1 = P_01*K00_1 + K01*P_11 + K02*P_12;
	common2 = P_02*K00_1 + K01*P_12 + K02*P_22;
	common3 = P_00*K00_1 + K01*P_01 + K02*P_02;
	common4 = P_01*K11_1 + K10*P_00 + K12*P_02;
	common5 = P_12*K11_1 + K10*P_02 + K12*P_22;
	common6 = P_11*K11_1 + K10*P_01 + K12*P_12;
	common7 = P_02*K22_1 + K20*P_00 + K21*P_01;
	common8 = P_12*K22_1 + K20*P_01 + K21*P_11;
	common9 = P_22*K22_1 + K20*P_02 + K21*P_12;
	commonA = -P_03 + K30*P_00 + K31*P_01 + K32*P_02;
	commonB = -P_13 + K30*P_01 + K31*P_11 + K32*P_12;
	commonC = -P_23 + K30*P_02 + K31*P_12 + K32*P_22;

	P__00 = K01*common1+K02*common2+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+K00_1*common3;
	P__01 = K10*common3+K12*common2+K11_1*common1+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
	P__02 = K20*common3+K21*common1+K22_1*common2+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
	P__03 = -P_03*K00_1+K30*common3+K31*common1+K32*common2-K01*P_13-K02*P_23+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
	P__04 = -P_04*K00_1+K40*common3+K41*common1+K42*common2-K01*P_14-K02*P_24+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
	P__05 = -P_05*K00_1+K50*common3+K51*common1+K52*common2-K01*P_15-K02*P_25+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
	P__11 = K10*common4+K12*common5+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+K11_1*common6;
	P__12 = K20*common4+K21*common6+K22_1*common5+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
	P__13 = -P_13*K11_1+K30*common4+K31*common6+K32*common5-K10*P_03-K12*P_23+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
	P__14 = -P_14*K11_1+K40*common4+K41*common6+K42*common5-K10*P_04-K12*P_24+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
	P__15 = -P_15*K11_1+K50*common4+K51*common6+K52*common5-K10*P_05-K12*P_25+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
	P__22 = K20*common7+K21*common8+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+K22_1*common9;
	P__23 = -P_23*K22_1+K30*common7+K31*common8+K32*common9-K20*P_03-K21*P_13+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
	P__24 = -P_24*K22_1+K40*common7+K41*common8+K42*common9-K20*P_04-K21*P_14+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
	P__25 = -P_25*K22_1+K50*common7+K51*common8+K52*common9-K20*P_05-K21*P_15+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
	P__33 = P_33+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+K30*commonA+K31*commonB+K32*commonC-K30*P_03-K31*P_13-K32*P_23;
	P__34 = P_34+K40*commonA+K41*commonB+K42*commonC-K30*P_04-K31*P_14-K32*P_24+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
	P__35 = P_35+K50*commonA+K51*commonB+K52*commonC-K30*P_05-K31*P_15-K32*P_25+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
	P__44 = P_44+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+K40*(-P_04+K40*P_00+K41*P_01+K42*P_02)+K41*(-P_14+K40*P_01+K41*P_11+K42*P_12)+K42*(-P_24+K40*P_02+K41*P_12+K42*P_22)-K40*P_04-K41*P_14-K42*P_24;
	P__45 = P_45+K50*(-P_04+K40*P_00+K41*P_01+K42*P_02)+K51*(-P_14+K40*P_01+K41*P_11+K42*P_12)+K52*(-P_24+K40*P_02+K41*P_12+K42*P_22)-K40*P_05-K41*P_15-K42*P_25+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
	P__55 = P_55+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+K50*(-P_05+K50*P_00+K51*P_01+K52*P_02)+K51*(-P_15+K50*P_01+K51*P_11+K52*P_12)+K52*(-P_25+K50*P_02+K51*P_12+K52*P_22)-K50*P_05-K51*P_15-K52*P_25;

	// Normalization of covariance
	len = sqrtf(x0*x0 + x1*x1 + x2*x2);
	invlen3 = 1.0f / (len*len*len);
	invlen32 = (invlen3*invlen3);

	x1x1_x2x2 = (x1*x1 + x2*x2);
	x0x0_x2x2 = (x0*x0 + x2*x2);
	x0x0_x1x1 = (x0*x0 + x1*x1);

	P00 = invlen32*(-x1x1_x2x2*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x0*x1*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2)+x0*x2*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2));
	P01 = invlen32*(-x0x0_x2x2*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2)+x0*x1*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x1*x2*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2));
	P02 = invlen32*(-x0x0_x1x1*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2)+x0*x2*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x1*x2*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2));
	P03 = -invlen3*(-P__03*x1x1_x2x2+P__13*x0*x1+P__23*x0*x2);
	P04 = -invlen3*(-P__04*x1x1_x2x2+P__14*x0*x1+P__24*x0*x2);
	P05 = -invlen3*(-P__05*x1x1_x2x2+P__15*x0*x1+P__25*x0*x2);
	P11 = invlen32*(-x0x0_x2x2*(-P__11*x0x0_x2x2+P__01*x0*x1+P__12*x1*x2)+x0*x1*(-P__01*x0x0_x2x2+P__00*x0*x1+P__02*x1*x2)+x1*x2*(-P__12*x0x0_x2x2+P__02*x0*x1+P__22*x1*x2));
	P12 = invlen32*(-x0x0_x1x1*(-P__12*x0x0_x2x2+P__02*x0*x1+P__22*x1*x2)+x0*x2*(-P__01*x0x0_x2x2+P__00*x0*x1+P__02*x1*x2)+x1*x2*(-P__11*x0x0_x2x2+P__01*x0*x1+P__12*x1*x2));
	P13 = -invlen3*(-P__13*x0x0_x2x2+P__03*x0*x1+P__23*x1*x2);
	P14 = -invlen3*(-P__14*x0x0_x2x2+P__04*x0*x1+P__24*x1*x2);
	P15 = -invlen3*(-P__15*x0x0_x2x2+P__05*x0*x1+P__25*x1*x2);
	P22 = invlen32*(-x0x0_x1x1*(-P__22*x0x0_x1x1+P__02*x0*x2+P__12*x1*x2)+x0*x2*(-P__02*x0x0_x1x1+P__00*x0*x2+P__01*x1*x2)+x1*x2*(-P__12*x0x0_x1x1+P__01*x0*x2+P__11*x1*x2));
	P23 = -invlen3*(-P__23*x0x0_x1x1+P__03*x0*x2+P__13*x1*x2);
	P24 = -invlen3*(-P__24*x0x0_x1x1+P__04*x0*x2+P__14*x1*x2);
	P25 = -invlen3*(-P__25*x0x0_x1x1+P__05*x0*x2+P__15*x1*x2);
	P33 = P__33;
	P34 = P__34;
	P35 = P__35;
	P44 = P__44;
	P45 = P__45;
	P55 = P__55;

        // increment covariance slightly at each iteration (nonoptimal but keeps the filter stable against rounding errors in 32bit float computation)
	P00 += VARIANCE_SAFETY_INCREMENT;
	P11 += VARIANCE_SAFETY_INCREMENT;
	P22 += VARIANCE_SAFETY_INCREMENT;
	P33 += VARIANCE_SAFETY_INCREMENT;
	P44 += VARIANCE_SAFETY_INCREMENT;
	P55 += VARIANCE_SAFETY_INCREMENT;

	// variance is required to be always at least the minimum value
	if (P00 < VARIANCE_MIN_LIMIT) P00 = VARIANCE_MIN_LIMIT;
	if (P11 < VARIANCE_MIN_LIMIT) P11 = VARIANCE_MIN_LIMIT;
	if (P22 < VARIANCE_MIN_LIMIT) P22 = VARIANCE_MIN_LIMIT;
	if (P33 < VARIANCE_MIN_LIMIT) P33 = VARIANCE_MIN_LIMIT;
	if (P44 < VARIANCE_MIN_LIMIT) P44 = VARIANCE_MIN_LIMIT;
	if (P55 < VARIANCE_MIN_LIMIT) P55 = VARIANCE_MIN_LIMIT;

	// normalized a posteriori state
	x0 = x0/len;
	x1 = x1/len;
	x2 = x2/len;


	// compute Euler angles (not exactly a part of the extended Kalman filter)
	// yaw integration through full rotation matrix
	u_nb0 = u0 - x3;
	u_nb1 = u1 - x4;
	u_nb2 = u2 - x5;

	if (0) {
		cy = cosf(yaw); //old angles (last state before integration)
		sy = sinf(yaw);
		d = sqrtf(x_last[1]*x_last[1] + x_last[2]*x_last[2]);
		d_inv = 1.0f / d;

		// compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
		R11 = cy * d;
		R12 = -(x_last[2]*sy + x_last[0]*x_last[1]*cy) * d_inv;
		R13 = (x_last[1]*sy - x_last[0]*x_last[2]*cy) * d_inv;
		R21 = sy * d;
		R22 = (x_last[2]*cy - x_last[0]*x_last[1]*sy) * d_inv;
		R23 = -(x_last[1]*cy + x_last[0]*x_last[2]*sy) * d_inv;

		// update needed parts of R for yaw computation
		R11_new = R11 + h*(u_nb2*R12 - u_nb1*R13);
		R21_new = R21 + h*(u_nb2*R22 - u_nb1*R23);
		yaw = atan2f(R21_new,R11_new);
	}
	else { //alternative method estimating the whole rotation matrix
		//integrate full rotation matrix (using first row estimate in memory)

		// calculate the second row (sr) from a rotated first row (rotation with bias corrected gyroscope measurement)
		sr0 = -fr1*x_last[2]+fr2*x_last[1]-h*(x_last[1]*(fr1*u_nb0-fr0*u_nb1)+x_last[2]*(fr2*u_nb0-fr0*u_nb2));
		sr1 = fr0*x_last[2]-fr2*x_last[0]+h*(x_last[0]*(fr1*u_nb0-fr0*u_nb1)-x_last[2]*(fr2*u_nb1-fr1*u_nb2));
		sr2 = -fr0*x_last[1]+fr1*x_last[0]+h*(x_last[0]*(fr2*u_nb0-fr0*u_nb2)+x_last[1]*(fr2*u_nb1-fr1*u_nb2));

		// normalize the second row
		invlen = 1.0f / sqrtf(sr0*sr0 + sr1*sr1 + sr2*sr2);
		sr0 *= invlen;
		sr1 *= invlen;
		sr2 *= invlen;

		// recompute the first row (ensure perpendicularity)
		fr0 = sr1*x_last[2] - sr2*x_last[1];
		fr1 = -sr0*x_last[2] + sr2*x_last[0];
		fr2 = sr0*x_last[1] - sr1*x_last[0];

		// normalize the first row
		invlen = 1.0f / sqrtf(fr0*fr0 + fr1*fr1 + fr2*fr2);
		fr0 *= invlen;
		fr1 *= invlen;
		fr2 *= invlen;

		// calculate yaw from first and second row
		yaw = atan2f(sr0,fr0);
	}

	// compute new pitch and roll angles from a posteriori states
	pitch = asinf(-x0);
	roll = atan2f(x1,x2);
//	printf("%f  ", x1);
//	printf("%f  \n", x2);
	// save the estimated non-gravitational acceleration
	a0 = (z0-x0)*g0;
	a1 = (z1-x1)*g0;
	a2 = (z2-x2)*g0;
}

void getState(float *State) {
	State[0] = x0;
	State[1] = x1;
	State[2] = x2;
	State[3] = x3;
	State[4] = x4;
	State[5] = x5;
}

void getCovariance(float *Covariance) {
	Covariance[0] = P00;
	Covariance[1] = P01;
	Covariance[2] = P02;
	Covariance[3] = P03;
	Covariance[4] = P04;
	Covariance[5] = P05;
	Covariance[6] = P01;
	Covariance[7] = P11;
	Covariance[8] = P12;
	Covariance[9] = P13;
	Covariance[10] = P14;
	Covariance[11] = P15;
	Covariance[12] = P02;
	Covariance[13] = P12;
	Covariance[14] = P22;
	Covariance[15] = P23;
	Covariance[16] = P24;
	Covariance[17] = P25;
	Covariance[18] = P03;
	Covariance[19] = P13;
	Covariance[20] = P23;
	Covariance[21] = P33;
	Covariance[22] = P34;
	Covariance[23] = P35;
	Covariance[24] = P04;
	Covariance[25] = P14;
	Covariance[26] = P24;
	Covariance[27] = P34;
	Covariance[28] = P44;
	Covariance[29] = P45;
	Covariance[30] = P05;
	Covariance[31] = P15;
	Covariance[32] = P25;
	Covariance[33] = P35;
	Covariance[34] = P45;
	Covariance[35] = P55;
}

void getNGAcc(float *ngacc) {
	ngacc[0] = a0;
	ngacc[1] = a1;
	ngacc[2] = a2;
}

//! A method to return the yaw angle.
	/*!
	 *  @return The yaw angle.
	 */
	float getYaw() { return yaw; }

	//! A method to return the pitch angle.
	/*!
	 *  @return The pitch angle.
	 */
	float getPitch() { return pitch; }

	//! A method to return the roll angle.
	/*!
	 *  @return The roll angle.
	 */
	float getRoll() { return roll; }
