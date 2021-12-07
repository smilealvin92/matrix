#include "attitude.h"

/*******************************************************************************
* Function Name  : init_quaternion
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(quaternion_struct *quaternion)
{ 
//  unsigned long sensor_timestamp;
	unsigned long timestamp;
//  unsigned char more;
//  long quat[4];
	short gy[3], ac[3], ma[3];
  float init_Yaw, init_Pitch, init_Roll;
	float init_ax, init_ay, init_az, init_mx, init_my, init_mz;
  int i;
//ѭ��20������Ϊ��һ��ִ��dmp_read_fifoʱ���޷�����if(sensors & INV_XYZ_ACCEL)����Ҫ��ѭ������
	//�����ʼ��Ԫ��ʱ��û���õ������ǵ�����
  for(i=0;i<20;i++)
  {  
//		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
		//���Բ���dmp��ȡ��ֱ�Ӵ�MPU6050�ļĴ�����ȡ
		mpu_get_gyro_reg(gy, 0);
		mpu_get_accel_reg(ac, 0);  
		//Accel_4_Scale_Factor��8192��
		//8192��2��13�η�������Ϊ���������ֵΪshort���ͣ��Ӹ�������2^16�η��ķ�Χ�������ٶȵĲ�����Χ������4g�����2^16����8����8192��
		init_ax=(float)(ac[0] / Accel_4_Scale_Factor);	   //��λת�����������ٶȵĵ�λ��g
		init_ay=(float)(ac[1] / Accel_4_Scale_Factor);
	  init_az=(float)((ac[2]+600) / Accel_4_Scale_Factor);
//		    printf("\r\n    ax=%f,   ay=%f,   az=%f\r\n", init_ax, init_ay, init_az);
    mpu_set_bypass(1);                     //����bypass��������������
    mpu_get_compass_reg(ma, &timestamp);  //��ȡcompass����
//����x y���У׼��δ��z�����У׼���ο�MEMSense��У׼���� 

//		init_mx =(float)mag[1]-8;						
//    init_my =(float)1.046632f*mag[0]-1.569948f;
//    init_mz =(float)-mag[2];
		init_mx =(float)1.046632f*ma[0]-1.569948f;						
		init_my =(float)ma[1]-8;
		init_mz =(float)ma[2];
    mpu_set_bypass(0);						//�ر�bypass��������������					   //�ر�bypass��������������
//    printf("    mx=%f,   my=%f,   mz=%f \n\r", init_mx, init_my, init_mz);
  }//end of for   

#if defined ZXY
	//������y��Ϊǰ��������Ϊ��Y����ת�Ľ���roll����ǰ����������ת��Ȼ�Ǻ�����ˡ�   
	init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.3ת��Ϊ�Ƕ�
//	init_Pitch = asin(init_ay/(init_ay*init_ay+init_az*init_az));              //init_Pitch = asin(ay / 1);  
	init_Pitch = asin(init_ay);
	init_Yaw = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
	
	if(init_Yaw < 0){init_Yaw = init_Yaw + 2*M_PI;}
	if(init_Yaw > 2*M_PI){init_Yaw = init_Yaw - 2*M_PI;}				            
	//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������zyx˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
	quaternion->w = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //w
	quaternion->xi = cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //x   ��x����ת��pitch
	quaternion->yj = sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //y   ��y����ת��roll
	quaternion->zk = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw) + sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw);  //z   ��z����ת��Yaw
#elif defined ZYX
	//������x��Ϊǰ������
  init_Roll  =  atan2(init_ay, init_az);
	//init_ax�п��ܴ���1������һ�����������������ˣ���ע����Ĵ�ӡֵ
	//���������һ��ʵ��
  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
	if(init_Yaw < 0){init_Yaw = init_Yaw + 2*M_PI;}
	if(init_Yaw > 2*M_PI){init_Yaw = init_Yaw - 2*M_PI;}		
	//��X����roll��fai����Y����pitch��theta��yaw��psi
  quaternion->w = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //w
  quaternion->xi = sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //x   ��x����ת��roll
  quaternion->yj = cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //y   ��y����ת��pitch
  quaternion->zk = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw) - sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw);  //z   ��z����ת��Yaw
#endif
  printf("��ʼ����Ԫ����Yaw=%f, Pitch=%f, Roll=%f, q0=%f, q1=%f, q2=%f, q3=%f", 
               init_Yaw, init_Pitch, init_Roll, quaternion->w, quaternion->xi, quaternion->yj, quaternion->zk);
}
/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��
     Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
//AHRS�㷨ĿǰЧ������ã����ȶ�����Ҳ������ȱ�㣬2���ھ��Ҹı�Yawʱ����ʱRoll����ų��ֲ�����ĸı䣬�����ı�Yaw��û��
***************************************************************************************************************************************/
void AHRSupdate(matrix *gyro, matrix *accel, matrix *mag, quaternion_struct *quaternion) 
{
	      float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
        float norm, halfT;
        float hx, hy, hz, by, bz;
//					float bx;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
//	      float qa, qb, qc;

/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
        //auxiliary variables to reduce number of repeated operations��
        float q0q0 = (quaternion->w)*(quaternion->w);
        float q0q1 = (quaternion->w)*(quaternion->xi);
        float q0q2 = (quaternion->w)*(quaternion->yj);
        float q0q3 = (quaternion->w)*(quaternion->zk);
        float q1q1 = (quaternion->xi)*(quaternion->xi);
        float q1q2 = (quaternion->xi)*(quaternion->yj);
        float q1q3 = (quaternion->xi)*(quaternion->zk);
        float q2q2 = (quaternion->yj)*(quaternion->yj);   
        float q2q3 = (quaternion->yj)*(quaternion->zk);
        float q3q3 = (quaternion->zk)*(quaternion->zk);          
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
        //normalise the measurements
        vector_normalize(accel);
				vector_normalize(mag);        
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
        //compute reference direction of flux
        hx = 2*(mag->data[0])*(0.5f - q2q2 - q3q3) + 2*(mag->data[1])*(q1q2 - q0q3) + 2*(mag->data[2])*(q1q3 + q0q2);
        hy = 2*(mag->data[0])*(q1q2 + q0q3) + 2*(mag->data[1])*(0.5f - q1q1 - q3q3) + 2*(mag->data[2])*(q2q3 - q0q1);
        hz = 2*(mag->data[0])*(q1q3 - q0q2) + 2*(mag->data[1])*(q2q3 + q0q1) + 2*(mag->data[2])*(0.5f - q1q1 - q2q2);

/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣�������by=0��bx=ĳֵ
������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊby=0�����Ծͼ򻯳�(bx*bx)  = ((hx*hx) + (hy*hy))�������bx��*/
        by = sqrtf((hx*hx) + (hy*hy));
//				bx = sqrtf((hx*hx) + (hy*hy));
        bz = hz;        
    
        // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
��Ϊby=0�����������漰��by�Ĳ��ֶ���ʡ���ˡ�
������������vxyz�����㣬��Ϊ����g��gz=1��gx=gy=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��wxyz�Ĺ�ʽ����bx����gx��0������bz����gz��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//          wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//          wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//          wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);
//TODO���˴���������һ��ʵ�飬�����ǲ���������ˣ���Ϊԭ������������Y����roll������������pitch��
				wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
				wy = 2*by*(0.5f - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
				wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5f - q1q1 - q2q2);
           
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
        // error is sum of cross product between reference direction of fields and direction measured by sensors
				
        ex = ((accel->data[1])*vz - (accel->data[2])*vy) + ((mag->data[1])*wz - (mag->data[2])*wy);
        ey = ((accel->data[2])*vx - (accel->data[0])*vz) + ((mag->data[2])*wx - (mag->data[0])*wz);
        ez = ((accel->data[0])*vy - (accel->data[1])*vx) + ((mag->data[0])*wy - (mag->data[1])*wx);
		          
//        // integral error scaled integral gain
//        exInt = exInt + ex*Ki;
//        eyInt = eyInt + ey*Ki;
//        ezInt = ezInt + ez*Ki;
//        
//        // adjusted gyroscope measurements
//        gx = gx + Kp*ex + exInt;
//        gy = gy + Kp*ey + eyInt;
//        gz = gz + Kp*ez + ezInt;

				halfT=GET_NOWTIME();
				if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
				{
					// integral error scaled integral gain
					exInt = exInt + ex*Ki * halfT;			   //���Բ������ڵ�һ��
					eyInt = eyInt + ey*Ki * halfT;
					ezInt = ezInt + ez*Ki * halfT;
					// adjusted gyroscope measurements
					gyro->data[0] += Kp*ex + exInt;
					gyro->data[1] += Kp*ey + eyInt;
					gyro->data[2] += Kp*ez + ezInt;
				}    

        // integrate quaternion rate and normalise����Ԫ�������㷨
				//�˴������д��󣬺����õ�q0�Ͳ���֮ǰ��q0�ˣ����˴��д���ȶ��
//				qa = q0;
//	      qb = q1;
//      	qc = q2;
//        q0 = q0 + (-qb*gx - qc*gy - q3*gz)*halfT;
//        q1 = q1 + (qa*gx + qc*gz - q3*gy)*halfT;
//        q2 = q2 + (qa*gy - qb*gz + q3*gx)*halfT;
//        q3 = q3 + (qa*gz + qb*gy - qc*gx)*halfT;  
        quaternion->w += (-(quaternion->xi)*(gyro->data[0]) - (quaternion->yj)*(gyro->data[1]) - (quaternion->zk)*(gyro->data[2]))*halfT;
        quaternion->xi += ((quaternion->w)*(gyro->data[0]) + (quaternion->yj)*(gyro->data[2]) - (quaternion->zk)*(gyro->data[1]))*halfT;
        quaternion->yj += ((quaternion->w)*(gyro->data[1]) - (quaternion->xi)*(gyro->data[2]) + (quaternion->zk)*(gyro->data[0]))*halfT;
        quaternion->zk += ((quaternion->w)*(gyro->data[2]) + (quaternion->xi)*(gyro->data[1]) - (quaternion->yj)*(gyro->data[0]))*halfT;  
        
        // normalise quaternion
        norm = invSqrt((quaternion->w)*(quaternion->w) + (quaternion->xi)*(quaternion->xi) + (quaternion->yj)*(quaternion->yj) + (quaternion->zk)*(quaternion->zk));
        (quaternion->w) = (quaternion->w) * norm;       //w
        (quaternion->yj) = (quaternion->yj) * norm;       //x
        (quaternion->xi) = (quaternion->xi) * norm;       //y
        (quaternion->zk) = (quaternion->zk) * norm;       //z
				quater_to_euler(quaternion);
}

void quater_to_euler(quaternion_struct *quaternion)
{
	///*����Ԫ�������Pitch  Roll  Yaw
//����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	float roll, pitch, yaw = 0.0f;
	float q0 = (quaternion->w);
	float q1 = (quaternion->xi);
	float q2 = (quaternion->yj);
	float q3 = (quaternion->zk);
#if defined ZXY
				//ĳ������ϵ�£���X����pitch��theta����Y����roll��fai��yaw��psi
			yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * rad_to_deg;  //ƫ���ǣ���z��ת��
			if(yaw < 0 ){Yaw = Yaw + 360;}
			if(yaw > 360 ){Yaw = Yaw - 360;}
			pitch = asin(2*q2*q3 + 2*q0*q1) * rad_to_deg; //�����ǣ���x��ת��	 
			roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * rad_to_deg; //�����ǣ���y��ת��
#elif defined ZYX
			//ĳ������ϵ�£�����X����roll��theta��Y����pitch��fai����yaw��psi
			pitch = -asin(-2*q0*q2 + 2*q1*q3) * rad_to_deg; //�����ǣ���y��ת��	 
			roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * rad_to_deg; //�����ǣ���x��ת��
			yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * (float)rad_to_deg;
			if(yaw < 0 ){yaw = yaw + 360;}
			if(yaw > 360 ){yaw = yaw - 360;}
#endif
/*���������Ԫ�������Pitch  Roll  Yaw
Roll=-arctan2(-2wy+2xz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=-arctan2(-2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
  printf("\r\nPitch = %f��    Roll = %f��     Yaw = %f��    ", pitch, roll, yaw);
}


///*******************************************************************************
//* Function Name  : IMUupdate
//* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick.
//* Input          : None
//* Output         : None
//* Return         : None

//// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
//// 1.����У׼accle gyro mag��
//// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
////   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
//// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
//// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��
//		 Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
//// 5.��ֱ��ʹ����Ԫ����
//// 6.�ظ�4�����ɸ�����̬;

////�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
////���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
////ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
////���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

////������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
////Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

////ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�
//*******************************************************************************/
void IMUupdate(matrix *gyro, matrix *accel, matrix *mag, quaternion_struct *quaternion, float *yaw, float halfT) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float q0, q1, q2, q3;
//	float q0, q1, q2, q3, qa, qb, qc, qd;
	float roll, pitch = 0.0f;
	float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
  //auxiliary variables to reduce number of repeated operations��
	float q0q0 = (quaternion->w)*(quaternion->w);
	float q0q1 = (quaternion->w)*(quaternion->xi);
	float q0q2 = (quaternion->w)*(quaternion->yj);
	float q1q1 = (quaternion->xi)*(quaternion->xi);
	float q1q3 = (quaternion->xi)*(quaternion->zk);
	float q2q2 = (quaternion->yj)*(quaternion->yj);   
	float q2q3 = (quaternion->yj)*(quaternion->zk);
	float q3q3 = (quaternion->zk)*(quaternion->zk); 
  /*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
  //normalise the measurements
	//axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������
	//����
	vector_normalize(accel);
	
	//vx��vy��vz��ʵ������һ�ε�ŷ���ǣ���Ԫ�����Ļ�������ο�ϵ
	//��������������ĵ�λ����
	//����ˣ�ԭ���㷨�У����õ���halfvx, halfvy, halfvz�������㷨�����������
	vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

      
  //���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	//axyz��vxyz���ǻ����������ϵ�ϵ�����������������֮�������������
	//���ֺ����̬�ͼ��ٶȼƲ��������̬֮������
	//��������������������������ʾ��exyz�����������������Ĳ��
	// vx\vy\vz������һ�ֹ�����̬�ǲ������
	ex = ((accel->data[1])*vz - (accel->data[2])*vy);
	ey = ((accel->data[2])*vx - (accel->data[0])*vz);
	ez = ((accel->data[0])*vy - (accel->data[1])*vx);
 
  //���µļ��������ò���������PI����������ƫ
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gyro->data[0] += Kp*ex + exInt;
	gyro->data[1] += Kp*ey + eyInt;
	gyro->data[2] += Kp*ez + ezInt;
        
//	halfT=GET_NOWTIME();
//	halfT = (0xFFFFFF-get_systick_val())/21000000.0f;
//	start_systick(0xFFFFFF);
//	printf("halfT:  %f", halfT);

	// integrate quaternion rate and normalise����Ԫ�������㷨��һ������-������
	//����������������Ϊ׼�����ٶȼƵ��������������������ǵ�����
	//��ѭ���в��ϸ�����Ԫ����Ȼ��͵ó��˲��ϸ��µ�
//	qa = quaternion->w;
//	qb = quaternion->xi;
//	qc = quaternion->yj;
//	qd = quaternion->zk;
//	quaternion->w += (-qb*gyro->data[0] - qc*gyro->data[1] - qd*gyro->data[2])*halfT;
//	quaternion->xi += (qa*gyro->data[0] - qd*gyro->data[1] + qc*gyro->data[2])*halfT;
//	quaternion->yj += (qa*gyro->data[1] + qd*gyro->data[0] - qb*gyro->data[2])*halfT;
//	quaternion->zk += (qa*gyro->data[2] - qc*gyro->data[0] + qb*gyro->data[1])*halfT;  
	halfT = halfT/2.0f;
	quaternion->w += (-(quaternion->xi)*(gyro->data[0]) - (quaternion->yj)*(gyro->data[1]) - (quaternion->zk)*(gyro->data[2]))*halfT;
  quaternion->xi += ((quaternion->w)*(gyro->data[0]) + (quaternion->yj)*(gyro->data[2]) - (quaternion->zk)*(gyro->data[1]))*halfT;
  quaternion->yj += ((quaternion->w)*(gyro->data[1]) - (quaternion->xi)*(gyro->data[2]) + (quaternion->zk)*(gyro->data[0]))*halfT;
  quaternion->zk += ((quaternion->w)*(gyro->data[2]) + (quaternion->xi)*(gyro->data[1]) - (quaternion->yj)*(gyro->data[0]))*halfT;  
	
	// normalise quaternion
	norm = invSqrt((quaternion->w)*(quaternion->w) + (quaternion->xi)*(quaternion->xi) + (quaternion->yj)*(quaternion->yj) + (quaternion->zk)*(quaternion->zk));
	(quaternion->w) = (quaternion->w) * norm;       //w
	(quaternion->yj) = (quaternion->yj) * norm;       //x
	(quaternion->xi) = (quaternion->xi) * norm;       //y
	(quaternion->zk) = (quaternion->zk) * norm;       //z
	

//
////0.9��0.1������ϵ��������5.73=0.1*rad_to_deg������57.3��Ϊ�˽�����ת��Ϊ�Ƕȣ��ù�ʽ��˼�ǽ������Ƶĳ���׼ȷ�Ⱥ�
////�����ǵĸ������Ƚ��л����˲������������ǵ����ݽ��и�ͨ�˲����Դ����Ƶ����ݽ��е�ͨ�˲��������  


/*����Ԫ�������Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;*/
//����Ԫ�������Pitch  Roll  Yaw,����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�
  q0 = quaternion->w;
  q1 = quaternion->xi;
  q2 = quaternion->yj;
  q3 = quaternion->zk;	
#if defined ZXY
  pitch = asin(2 * q2 * q3 + 2 * q0 * q1); //�����ǣ���x��ת��	 
  roll  = -atan2(2 * q1 * q3 - 2 * q0 * q2, -2 * q1 * q1 - 2 * q2* q2 + 1); //�����ǣ���y��ת��
//����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�
//q0��w��q1��x��q2��y��q3��z
#elif defined ZYX
	pitch = asin(-2 * q1 * q3 + 2 * q0 * q2); //�����ǣ���y��ת��	 
  roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); //�����ǣ���x��ת��
#endif
//�������ڶ�ʱ�������ã���ʱ��Ļ��ֻ����������ٶȼƸպ÷���������ʱ���ڲ��ɿ�
//�����ǳ�ʱ��Ļ��ֽ���ͱȽ�׼ȷ
//0.9��0.1������ϵ��������5.73=0.1*57.3������57.3��Ϊ�˽�����ת��Ϊ�Ƕȣ��ù�ʽ��˼�ǽ������Ƶĳ���׼ȷ�Ⱥ�
//�����ǵĸ������Ƚ��л����˲������������ǵ����ݽ��и�ͨ�˲����Դ����Ƶ����ݽ��е�ͨ�˲��������
//�����ǣ���y��ת��
//	Yaw   = (0.9f * (Yaw + init_gz*2*halfT) + 5.73f * atan2(init_mx*cos(Roll) + init_my*sin(Roll)*sin(Pitch) + init_mz*sin(Roll)*cos(Pitch), init_my*cos(Pitch) - init_mz*sin(Pitch)));
//�����ǣ���x��ת��,�����ںϰ취��Ȼ����Ư�ƣ����ǵ�+-170֮�󣬾ͻ���Ҳ��������AHRS��Զ��
//֮���Ժ�����һ�д�����һ���ģ�����Ϊinit_mx��init_my�Ѿ��任����
	*yaw   = (0.9f * (*yaw + (gyro->data[2])*2*halfT*57.3f) + 5.73f * atan2((mag->data[0])*cos(roll) + (mag->data[1])*sin(roll)*sin(pitch) + (mag->data[2])*sin(roll)*cos(pitch), (mag->data[1])*cos(pitch) - (mag->data[2])*sin(pitch)));
	pitch = pitch * (float)rad_to_deg;
	roll = roll * (float)rad_to_deg;
	//��һ����һ������������ʹ�ô����Ƶ�����
	//�Թ�֮�������Yaw��Ư�ơ�����
//	Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * (float)rad_to_deg;
	
//	printf("q0=%f, q1=%f, q2=%f, q3=%f, Yaw=%f, Pitch=%f, Roll=%f \n\r", q0, q1, q2, q3, Yaw, Pitch, Roll);
  printf("\r\nPitch=%f��    Roll=%f��     Yaw=%f��    ", pitch, roll, *yaw);
}

void transform_data(short *gyro, short* accel, matrix *gyro_mat, matrix *accel_mat, float *bias){
	//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s����Gyro_500_Scale_Factor�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
	 //Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)
	 //�����ʼ����Ԫ��ʱ��û���õ������ǵ�����
	
		gyro_mat->data[0] = (float)(((float)gyro[0]-bias[0]) / (Gyro_500_rad_Scale_Factor));	  	     //��λת���ɣ�����/s
		gyro_mat->data[1] = (float)(((float)gyro[1]-bias[1]) / (Gyro_500_rad_Scale_Factor));
		gyro_mat->data[2] = (float)(((float)gyro[2]-bias[2]) / (Gyro_500_rad_Scale_Factor));
			 //�˴���Щ���ٶȼƵ�����û��������ʼ����Ԫ����ʱ������������Ϊ����Ҫ�����涼��һ����
//	 	 accel_mat->data[0] = (float)accel[0];	  
//		 accel_mat->data[1] = (float)accel[1];
//		 accel_mat->data[2] = (float)accel[2];
		accel_mat->data[0] = (float)(accel[0]/Accel_4_M_Scale_Factor); //��λת�����������ٶȵĵ�λ��m/s2
		accel_mat->data[1] = (float)(accel[1]/Accel_4_M_Scale_Factor);
		accel_mat->data[2] = (float)((accel[2])/Accel_4_M_Scale_Factor);
}

void transform_data_better(short *gyro, short* accel, matrix *gyro_mat, matrix *accel_mat, float *gyro_bias, float *acc_bias){
	//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s����Gyro_500_Scale_Factor�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
	 //Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)
	 //�����ʼ����Ԫ��ʱ��û���õ������ǵ�����
	
		gyro_mat->data[0] = (((float)gyro[0]-gyro_bias[0]) / (Gyro_500_rad_Scale_Factor));	  	     //��λת���ɣ�����/s
		gyro_mat->data[1] = (((float)gyro[1]-gyro_bias[1]) / (Gyro_500_rad_Scale_Factor));
		gyro_mat->data[2] = (((float)gyro[2]-gyro_bias[2]) / (Gyro_500_rad_Scale_Factor));
			 //�˴���Щ���ٶȼƵ�����û��������ʼ����Ԫ����ʱ������������Ϊ����Ҫ�����涼��һ����
//	 	 accel_mat->data[0] = (float)accel[0];	  
//		 accel_mat->data[1] = (float)accel[1];
//		 accel_mat->data[2] = (float)accel[2];
		accel_mat->data[0] = (((float)accel[0]-acc_bias[0])/Accel_4_M_Scale_Factor); //��λת�����������ٶȵĵ�λ��m/s2
		accel_mat->data[1] = (((float)accel[1]-acc_bias[1])/Accel_4_M_Scale_Factor);
		accel_mat->data[2] = (((float)accel[2]-acc_bias[2])/Accel_4_M_Scale_Factor);
}

void gyro_calibrate(float *gyro_bias){
	short gyro[3]; 
	int	last_gyro[3], change_gyro[3];
	short flag_calib_failed = 1;
	int i = 0;
	int j = 0;
	for(j=0;j<200;j++){
		mpu_get_gyro_reg(gyro, 0);
	}
	for(j=0;j<3;j++){
		gyro_bias[j]=0.0f;
	}
	while(flag_calib_failed){
//		printf("i: %d", i);
//		printf("gyro 0: %f", gyro_bias[0]);
//		printf("gyro 1: %f", gyro_bias[1]);
//		printf("gyro 2: %f\n\n", gyro_bias[2]);
		if (i<200){
			mpu_get_gyro_reg(gyro, 0);
			if(i==0){
				for(j=0;j<3;j++){
					last_gyro[j]=gyro[j];
				}
			}
			for(j=0;j<3;j++){
				change_gyro[j]+=abs(gyro[j]-last_gyro[j]);
			}
			for(j=0;j<3;j++){
				last_gyro[j]=gyro[j];
			}
			for(j=0;j<3;j++){
//				printf("gyro 0: %f\n", gyro_bias[j]);
//				printf("gyro 0: %d\n", gyro[j]);
				gyro_bias[j]+=(float)gyro[j];
//				printf("gyro 0: %f\n", gyro_bias[j]);
			}
			i++;
		}else{
			if(change_gyro[0]>500||change_gyro[1]>500||change_gyro[2]>500){
				flag_calib_failed = 1;
				i=0;
				for(j=0;j<3;j++){
					gyro_bias[j]=0.0f;
					change_gyro[j]=0;
				}
			}else{
				flag_calib_failed=0;
			}
		}
	}
	for(j=0;j<3;j++){
//		printf("gyro_bias: %f\n", gyro_bias[j]);
		gyro_bias[j]*=0.005f;
//		printf("gyro_bias small: %f\n", gyro_bias[j]);
	}
}

// ȷ��ÿ�����ٶȼƵĳ�ֵƫ�ƹ��̣���ʹ��U3�����ݽ�����̬���㣬�����̬�󣬾Ϳ���ͨ���������������̣����ÿ�����ϵĳ�ֵƫ��
void acc_calibrate(float *acc_bias, float *gyro_bias){
	short gyro[3], accel[3];
	int accel_zero[3];
	float halfT;
	short ax_g, ay_g, az_g;
	short j;
	int last_acc[3], change_acc[3];
	short flag_calib_failed = 1;
	short i = 0;
	matrix *gyro_mat = matrix_init(3, 1);
	matrix *accel_mat = matrix_init(3, 1);
	// �ȶ�ȡ���ݣ�����һ�£���������ݵ�ƽ����
	for(j=0;j<200;j++){
		mpu_get_gyro_reg(gyro, 0);
		mpu_get_accel_reg(accel, 0);
	}
	for(j=0;j<3;j++){
		acc_bias[j]=0.0f;
	}
	DCM_IMU_uC_init_m(DEFAULT_g0, NULL, NULL, DEFAULT_q_dcm2, DEFAULT_q_gyro_bias2, DEFAULT_q_dcm2_init, DEFAULT_q_gyro_bias2_init, DEFAULT_r_acc2, DEFAULT_r_a2);
	while(flag_calib_failed){
//		printf("i: %d", i);
//		printf("gyro 0: %f", gyro_bias[0]);
//		printf("gyro 1: %f", gyro_bias[1]);
//		printf("gyro 2: %f\n\n", gyro_bias[2]);
		if (i<200){
			mpu_get_accel_reg(accel, 0);
			mpu_get_gyro_reg(gyro, 0);
			transform_data(gyro, accel, gyro_mat, accel_mat, gyro_bias);
			halfT = GET_NOWTIME();
	    updateIMU_m(gyro_mat->data, accel_mat->data, halfT*4.0f);
	    pitch = getPitch_m();
	    roll = getRoll_m();
			// ����ADCshort���͵���
	    ax_g = (short)((-1)*sin(pitch)*DEFAULT_g0*Accel_4_M_Scale_Factor);
	    ay_g = (short)(cos(pitch)*sin(roll)*DEFAULT_g0*Accel_4_M_Scale_Factor);
			az_g = (short)(cos(pitch)*cos(roll)*DEFAULT_g0*Accel_4_M_Scale_Factor);
			accel_zero[0]=accel[0]-ax_g;
			accel_zero[1]=accel[1]-ay_g;
			accel_zero[2]=accel[2]-az_g;
			if(i==0){
				for(j=0;j<3;j++){
					last_acc[j]=accel_zero[j];
				}
			}
			for(j=0;j<3;j++){
				change_acc[j]+=abs(accel_zero[j]-last_acc[j]);
			}
			for(j=0;j<3;j++){
				last_acc[j]=accel_zero[j];
			}
			for(j=0;j<3;j++){
//				printf("accel_zero: %d\n", accel_zero[j]);
//				printf("gyro 0: %d\n", gyro[j]);
				acc_bias[j]+=(float)accel_zero[j];
//				printf("gyro 0: %f\n", gyro_bias[j]);
			}
			i++;
		}else{
//			printf("change_acc: %d\n", change_acc[0]);
//			printf("change_acc: %d\n", change_acc[1]);
//			printf("change_acc: %d\n", change_acc[2]);
			if(change_acc[0]>1000||change_acc[1]>1000||change_acc[2]>1000){
				flag_calib_failed = 1;
				i=0;
				for(j=0;j<3;j++){
					acc_bias[j]=0.0f;
					change_acc[j]=0;
				}
			}else{
				flag_calib_failed=0;
//				i = 0;
			}
		}
	}
	for(j=0;j<3;j++){
//		printf("gyro_bias: %f\n", gyro_bias[j]);
		acc_bias[j]*=0.005f;
		printf("acc_bias: %f\n",acc_bias[j]);
//		printf("gyro_bias small: %f\n", gyro_bias[j]);
	}
//	printf("Pitch=%f��    Roll=%f��     Yaw=%f��\n", getPitch_m()*rad_to_deg, getRoll_m()*rad_to_deg, getYaw_m()*rad_to_deg);
}

void acc_calibrate_horizon(float *acc_bias){
	short acc[3]; 
	int	last_acc[3], change_acc[3];
	short flag_calib_failed = 1;
	int i = 0;
	int j = 0;
	for(j=0;j<200;j++){
		mpu_get_accel_reg(acc, 0);
	}
	for(j=0;j<3;j++){
		acc_bias[j]=0.0f;
	}
	while(flag_calib_failed){
//		printf("i: %d", i);
//		printf("gyro 0: %f", gyro_bias[0]);
//		printf("gyro 1: %f", gyro_bias[1]);
//		printf("gyro 2: %f\n\n", gyro_bias[2]);
		if (i<200){
			mpu_get_accel_reg(acc, 0);
			acc[2] = acc[2]-Accel_4_Scale_Factor;
			if(i==0){
				for(j=0;j<3;j++){
					last_acc[j]=acc[j];
				}
			}
			for(j=0;j<3;j++){
				change_acc[j]+=abs(acc[j]-last_acc[j]);
			}
			for(j=0;j<3;j++){
				last_acc[j]=acc[j];
			}
			for(j=0;j<3;j++){
//				printf("gyro 0: %f\n", gyro_bias[j]);
//				printf("gyro 0: %d\n", gyro[j]);
				acc_bias[j]+=(float)acc[j];
//				printf("gyro 0: %f\n", gyro_bias[j]);
			}
			i++;
		}else{
			if(change_acc[0]>1000||change_acc[1]>1000||change_acc[2]>1000){
				flag_calib_failed = 1;
				i=0;
				for(j=0;j<3;j++){
					acc_bias[j]=0.0f;
					change_acc[j]=0;
				}
			}else{
				flag_calib_failed=0;
			}
		}
	}
	for(j=0;j<3;j++){
//		printf("gyro_bias: %f\n", gyro_bias[j]);
		acc_bias[j]*=0.005f;
//		printf("gyro_bias small: %f\n", gyro_bias[j]);
	}
}

void gyro_fusion(matrix *gyro_3, matrix *gyro_2, matrix *gyro_4, matrix *gyro_5, matrix *gyro_6){
	// U3��X��Ľ��ٶȺ�U4��X��Ľ��ٶȡ�U6��X��Ľ��ٶ�Ӧ����һ���ģ�ͬ���ģ�U3��Y��Ľ��ٶȺ�U2��Y��Ľ��ٶȡ�U5��Y��Ľ��ٶ���һ����
	// �ֽ������ƽ�������Լ�������
	gyro_3->data[0] = (gyro_4->data[0]+gyro_6->data[0]+gyro_3->data[0])/3.0f;
	gyro_3->data[1] = (gyro_2->data[1]+gyro_5->data[1]+gyro_3->data[1])/3.0f;
}
// U3��U4��U6��roll��һ���ģ�U3��U2��U5��pitch��һ���ģ�
// ��������Χ�ĸ����ٶȼƵ�����ȥ�ں��м��������z����ٶ����ݣ��ĸ����ٶȼƵ��᲻һ������Ҫ������������
		// ����u2����y������������ݣ�����u4����x������������ݣ�����u5����y�Ḻ��������ݣ�����u6����x�Ḻ���������
void get_centri_pitch(matrix *acc, float pitch, float *acc_centri){
	//		ax_g = (-1)*sin(pitch)*DEFAULT_g0;
//		ay_g = cos(pitch)*sin(roll)*DEFAULT_g0;
	float az_no_bias, cosroll;
	float roll;
	az_no_bias = acc->data[2];
	cosroll = az_no_bias/(DEFAULT_g0*cos(pitch));
	cosroll = 1<cosroll?1:cosroll;
//	printf("acos: %f\n", az_no_bias/(DEFAULT_g0*cos(pitch)));
	roll = acos(cosroll);
//	printf("roll: %f\n", roll*rad_to_deg);
	*acc_centri = acc->data[1]-DEFAULT_g0*cos(pitch)*sin(roll);
}

// U3��U4��U6��roll��һ���ģ�U3��U2��U5��pitch��һ���ģ�
void get_centri_roll(matrix *acc, float roll, float *acc_centri){
	float az_no_bias;
	float cospitch;
	float pitch;
	az_no_bias = acc->data[2];
	cospitch = az_no_bias/(DEFAULT_g0*cos(roll));
	cospitch = 1<cospitch?1:cospitch;
//	printf("acos: %f\n", az_no_bias/(DEFAULT_g0*cos(roll)));
	pitch = acos(cospitch);
//	printf("pitch: %f\n", pitch*rad_to_deg);
	*acc_centri = acc->data[0]+DEFAULT_g0*sin(pitch);
}

void sensor_fusion(matrix *gyro, matrix *acc_2, matrix *acc_4, matrix *acc_5, matrix *acc_6){
	float acc_centri_2, acc_centri_4, acc_centri_5, acc_centri_6, acc_centri;
//	if(fabs(gyro->data[2])<0.003f){
//			gyro->data[2]=0.0f;
//	}
	acc_centri_2 = (-1)*acc_2->data[1];
	acc_centri_4 = (-1)*acc_4->data[0];
	acc_centri_5 = acc_5->data[1];
	acc_centri_6 = acc_6->data[0];
	acc_centri = (acc_centri_2+acc_centri_4+acc_centri_5+acc_centri_6)/4.0f;
//	if(acc_centri<0){
//		gyro->data[2] = (-1)*0.9f*sqrtf(fabs(acc_centri)/0.07)+0.1*gyro->data[2];
//	}
//	else{
//		gyro->data[2] = 0.9f*sqrtf(fabs(acc_centri)/0.07)+0.1*gyro->data[2];
//	}
//	printf("acc_centri: %f\n", acc_centri/0.07);
	
}

void sensor_fusion_kf(matrix *gyro_3, matrix *gyro_2, matrix *gyro_4, matrix *gyro_5, matrix *gyro_6, 
	matrix *acc_2, matrix *acc_4, matrix *acc_5, matrix *acc_6, float time, double *s, double *P){
	double acc_angle_2, acc_angle_4, acc_angle_5, acc_angle_6, acc_angle;
	double kalman_gain;
	double gyro_mean;
//	if(fabs(gyro->data[2])<0.003f){
//			gyro->data[2]=0.0f;
//	}
//		printf("P: %f\n", *P);
	acc_angle_2 = (-1)*acc_2->data[0];
	acc_angle_4 = acc_4->data[1];
	acc_angle_5 = acc_5->data[0];
	acc_angle_6 = (-1)*acc_6->data[1];
  acc_angle = (acc_angle_2+acc_angle_4+acc_angle_5+acc_angle_6)/(4.0*0.07);
	gyro_mean = (gyro_3->data[2]+gyro_2->data[2]+gyro_4->data[2]+gyro_5->data[2]+gyro_6->data[2])/5.0;
//	gyro_mean = (gyro_3->data[2]+gyro_2->data[2]+gyro_4->data[2]+gyro_6->data[2])/4.0;	
	*s = *s + acc_angle*time;
	*P = *P + ((double)time)*time*0.0005/(4.0*0.0049);
//		*P = *P + ((double)time)*time*(0.0004377+0.0004504+0.0004979+0.0003862)/(16.0*0.0049);
//		printf("P-: %f\n", *P);
	kalman_gain = *P/(*P+0.000003/5.0);
//	kalman_gain = *P/(*P+(0.0000004736+0.0000004065+0.0000005359+0.000002815+0.0000006885)/25.0);
//		kalman_gain = *P/(*P+(0.0000004736+0.0000004065+0.0000005359+0.0000006885)/16.0);
//		printf("kalman_gain: %f\n", kalman_gain);
		
//	kalman_gain = *P/(*P+0.0000007f/4.0f);
	*s = *s + kalman_gain*(gyro_mean-(*s));
	*P = *P - kalman_gain*(*P);
	
//	if(acc_centri<0){
//		gyro->data[2] = (-1)*0.9f*sqrtf(fabs(acc_centri)/0.07)+0.1*gyro->data[2];
//	}
//	else{
//		gyro->data[2] = 0.9f*sqrtf(fabs(acc_centri)/0.07)+0.1*gyro->data[2];
//	}
//	printf("acc_centri: %f\n", acc_centri/0.07);
}

void sensor_fusion_with_angle(matrix *gyro, matrix *acc_2, matrix *acc_4, matrix *acc_5, matrix *acc_6, float pitch, float roll){
	float acc_centri_2, acc_centri_4, acc_centri_5, acc_centri_6, acc_centri;
	if(fabs(gyro->data[2])<0.003f){
			gyro->data[2]=0.0f;
	}
	get_centri_pitch(acc_2, pitch, &acc_centri_2);
	get_centri_roll(acc_4, roll, &acc_centri_4);
	get_centri_pitch(acc_5, pitch, &acc_centri_5);
	get_centri_roll(acc_6, roll, &acc_centri_6);
	acc_centri_2 = (-1)*acc_centri_2;
	acc_centri_4 = (-1)*acc_centri_4;
//	acc_centri_5 = acc_5->data[1];
//	acc_centri_6 = acc_6->data[0];
	acc_centri = (acc_centri_2+acc_centri_4+acc_centri_5+acc_centri_6)/4.0f;
//	if(acc_centri<0){
//		gyro->data[2] = (-1)*0.9f*sqrtf(fabs(acc_centri)/0.07)+0.1*gyro->data[2];
//	}
//	else{
//		gyro->data[2] = 0.9f*sqrtf(fabs(acc_centri)/0.07)+0.1*gyro->data[2];
//	}
	printf("acc_centri: %f\n", acc_centri/0.07);
	
}
////���㷨�������Ǹ��㷨����������ط��ǲ�һ����
//void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
//	float recipNorm;
//    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
//	float hx, hy, bx, bz;
//	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
//	float halfex, halfey, halfez;
//	float qa, qb, qc;
//	float halfT;
//	float Pitch, Roll, Yaw;

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//		ax *= recipNorm;
//		ay *= recipNorm;
//		az *= recipNorm;     

//		// Normalise magnetometer measurement
//		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
//		mx *= recipNorm;
//		my *= recipNorm;
//		mz *= recipNorm;   

//		// Auxiliary variables to avoid repeated arithmetic
//		q0q0 = q0 * q0;
//		q0q1 = q0 * q1;
//		q0q2 = q0 * q2;
//		q0q3 = q0 * q3;
//		q1q1 = q1 * q1;
//		q1q2 = q1 * q2;
//		q1q3 = q1 * q3;
//		q2q2 = q2 * q2;
//		q2q3 = q2 * q3;
//		q3q3 = q3 * q3;   

//		// Reference direction of Earth's magnetic field
//		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
//		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
//		bx = sqrt(hx * hx + hy * hy);
//		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

//		// Estimated direction of gravity and magnetic field
//		halfvx = q1q3 - q0q2;
//		halfvy = q0q1 + q2q3;
//		halfvz = q0q0 - 0.5f + q3q3;
//		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
//	
//		// Error is sum of cross product between estimated direction and measured direction of field vectors
//		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
//		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
//		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
//		
//		halfT=GET_NOWTIME();
//		// Compute and apply integral feedback if enabled
//		if(Ki > 0.0f) {
//			exInt += Ki * halfex * halfT * 2.0f;	// integral error scaled by Ki
//			eyInt += Ki * halfey * halfT * 2.0f;
//			ezInt += Ki * halfez * halfT * 2.0f;
//			gx += exInt;	// apply integral feedback
//			gy += eyInt;
//			gz += ezInt;
//		}
//		else {
//			exInt = 0.0f;	// prevent integral windup
//			eyInt = 0.0f;
//			ezInt = 0.0f;
//		}

//		// Apply proportional feedback
//		gx += Kp * halfex;
//		gy += Kp * halfey;
//		gz += Kp * halfez;
//	}
//	
//	// Integrate rate of change of quaternion
//	gx *= halfT;		// pre-multiply common factors
//	gy *= halfT;
//	gz *= halfT;
//	qa = q0;
//	qb = q1;
//	qc = q2;
//	q0 += (-qb * gx - qc * gy - q3 * gz);
//	q1 += (qa * gx + qc * gz - q3 * gy);
//	q2 += (qa * gy - qb * gz + q3 * gx);
//	q3 += (qa * gz + qb * gy - qc * gx); 
//	
//	// Normalise quaternion
//	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//	//ĳ������ϵ�£���Y����pitch��theta����X����roll��fai��yaw��psi
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * rad_to_deg; //�����ǣ���y��ת��	 
//	Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * rad_to_deg; //�����ǣ���x��ת��
//	Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * (float)rad_to_deg;
//	if(Yaw < 0 ){Yaw = Yaw + 360;}
//	if(Yaw > 360 ){Yaw = Yaw - 360;}
//	//����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
//  printf("\r\nPitch = %f��    Roll = %f��     Yaw = %f��    ", Pitch, Roll, Yaw);
//}
