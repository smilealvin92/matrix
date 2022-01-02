#include "attitude.h"

/*******************************************************************************
* Function Name  : init_quaternion
* Description    : 算出初始化四元数q0 q1 q2 q3.
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
//循环20次是因为第一次执行dmp_read_fifo时，无法进入if(sensors & INV_XYZ_ACCEL)，需要多循环几次
	//计算初始四元数时，没有用到陀螺仪的数据
  for(i=0;i<20;i++)
  {  
//		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
		//尝试不从dmp读取，直接从MPU6050的寄存器读取
		mpu_get_gyro_reg(gy, 0);
		mpu_get_accel_reg(ac, 0);  
		//Accel_4_Scale_Factor是8192。
		//8192即2的13次方，是因为传感器输出值为short类型，从负到正有2^16次方的范围，而加速度的测量范围是正负4g，因此2^16除以8等于8192。
		init_ax=(float)(ac[0] / Accel_4_Scale_Factor);	   //单位转化成重力加速度的单位：g
		init_ay=(float)(ac[1] / Accel_4_Scale_Factor);
	  init_az=(float)((ac[2]+600) / Accel_4_Scale_Factor);
//		    printf("\r\n    ax=%f,   ay=%f,   az=%f\r\n", init_ax, init_ay, init_az);
    mpu_set_bypass(1);                     //开启bypass，必须有这句代码
    mpu_get_compass_reg(ma, &timestamp);  //读取compass数据
//进行x y轴的校准，未对z轴进行校准，参考MEMSense的校准方法 

//		init_mx =(float)mag[1]-8;						
//    init_my =(float)1.046632f*mag[0]-1.569948f;
//    init_mz =(float)-mag[2];
		init_mx =(float)1.046632f*ma[0]-1.569948f;						
		init_my =(float)ma[1]-8;
		init_mz =(float)ma[2];
    mpu_set_bypass(0);						//关闭bypass，必须有这句代码					   //关闭bypass，必须有这句代码
//    printf("    mx=%f,   my=%f,   mz=%f \n\r", init_mx, init_my, init_mz);
  }//end of for   

#if defined ZXY
	//陀螺仪y轴为前进方向，因为绕Y轴旋转的角是roll，绕前进方向轴旋转当然是横滚角了。   
	init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.3转化为角度
//	init_Pitch = asin(init_ay/(init_ay*init_ay+init_az*init_az));              //init_Pitch = asin(ay / 1);  
	init_Pitch = asin(init_ay);
	init_Yaw = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度
	
	if(init_Yaw < 0){init_Yaw = init_Yaw + 2*M_PI;}
	if(init_Yaw > 2*M_PI){init_Yaw = init_Yaw - 2*M_PI;}				            
	//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照zyx顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
	quaternion->w = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //w
	quaternion->xi = cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //x   绕x轴旋转是pitch
	quaternion->yj = sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //y   绕y轴旋转是roll
	quaternion->zk = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw) + sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw);  //z   绕z轴旋转是Yaw
#elif defined ZYX
	//陀螺仪x轴为前进方向
  init_Roll  =  atan2(init_ay, init_az);
	//init_ax有可能大于1，这样一来，不就是有问题了？关注上面的打印值
	//明天这个做一下实验
  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
	if(init_Yaw < 0){init_Yaw = init_Yaw + 2*M_PI;}
	if(init_Yaw > 2*M_PI){init_Yaw = init_Yaw - 2*M_PI;}		
	//绕X轴是roll是fai，绕Y轴是pitch是theta，yaw是psi
  quaternion->w = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //w
  quaternion->xi = sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*cos(0.5f*init_Yaw) - cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //x   绕x轴旋转是roll
  quaternion->yj = cos(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw) + sin(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw);  //y   绕y轴旋转是pitch
  quaternion->zk = cos(0.5f*init_Roll)*cos(0.5f*init_Pitch)*sin(0.5f*init_Yaw) - sin(0.5f*init_Roll)*sin(0.5f*init_Pitch)*cos(0.5f*init_Yaw);  //z   绕z轴旋转是Yaw
#endif
  printf("初始化四元数：Yaw=%f, Pitch=%f, Roll=%f, q0=%f, q1=%f, q2=%f, q3=%f", 
               init_Yaw, init_Pitch, init_Roll, quaternion->w, quaternion->xi, quaternion->yj, quaternion->zk);
}
/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，
     然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
//AHRS算法目前效果是最好，最稳定，但也有两个缺点，2：在剧烈改变Yaw时，此时Roll会跟着出现不合理的改变，缓慢改变Yaw则没事
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

/*方便之后的程序使用，减少计算时间*/
        //auxiliary variables to reduce number of repeated operations，
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
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
        //normalise the measurements
        vector_normalize(accel);
				vector_normalize(mag);        
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
        //compute reference direction of flux
        hx = 2*(mag->data[0])*(0.5f - q2q2 - q3q3) + 2*(mag->data[1])*(q1q2 - q0q3) + 2*(mag->data[2])*(q1q3 + q0q2);
        hy = 2*(mag->data[0])*(q1q2 + q0q3) + 2*(mag->data[1])*(0.5f - q1q1 - q3q3) + 2*(mag->data[2])*(q2q3 - q0q1);
        hz = 2*(mag->data[0])*(q1q3 - q0q2) + 2*(mag->data[1])*(q2q3 + q0q1) + 2*(mag->data[2])*(0.5f - q1q1 - q2q2);

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。*/
        by = sqrtf((hx*hx) + (hy*hy));
//				bx = sqrtf((hx*hx) + (hy*hy));
        bz = hz;        
    
        // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为by=0，所以所有涉及到by的部分都被省略了。
类似上面重力vxyz的推算，因为重力g的gz=1，gx=gy=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把bx换成gx（0），把bz换成gz（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//          wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//          wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//          wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);
//TODO，此处还可以做一下实验，看看是不是这里错了，因为原作者坐标中绕Y轴是roll，而我现在是pitch。
				wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
				wy = 2*by*(0.5f - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
				wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5f - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
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
				if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
				{
					// integral error scaled integral gain
					exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
					eyInt = eyInt + ey*Ki * halfT;
					ezInt = ezInt + ez*Ki * halfT;
					// adjusted gyroscope measurements
					gyro->data[0] += Kp*ex + exInt;
					gyro->data[1] += Kp*ey + eyInt;
					gyro->data[2] += Kp*ez + ezInt;
				}    

        // integrate quaternion rate and normalise，四元数更新算法
				//此处明显有错误，后面用的q0就不是之前的q0了，（此处有待商榷）
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
	///*由四元数计算出Pitch  Roll  Yaw
//乘以57.3是为了将弧度转化为角度*/
	float roll, pitch, yaw = 0.0f;
	float q0 = (quaternion->w);
	float q1 = (quaternion->xi);
	float q2 = (quaternion->yj);
	float q3 = (quaternion->zk);
#if defined ZXY
				//某种坐标系下，绕X轴是pitch是theta，绕Y轴是roll是fai，yaw是psi
			yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * rad_to_deg;  //偏航角，绕z轴转动
			if(yaw < 0 ){Yaw = Yaw + 360;}
			if(yaw > 360 ){Yaw = Yaw - 360;}
			pitch = asin(2*q2*q3 + 2*q0*q1) * rad_to_deg; //俯仰角，绕x轴转动	 
			roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * rad_to_deg; //滚动角，绕y轴转动
#elif defined ZYX
			//某种坐标系下，绕绕X轴是roll是theta，Y轴是pitch是fai，，yaw是psi
			pitch = -asin(-2*q0*q2 + 2*q1*q3) * rad_to_deg; //俯仰角，绕y轴转动	 
			roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * rad_to_deg; //滚动角，绕x轴转动
			yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * (float)rad_to_deg;
			if(yaw < 0 ){yaw = yaw + 360;}
			if(yaw > 360 ){yaw = yaw - 360;}
#endif
/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=-arctan2(-2wy+2xz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=-arctan2(-2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.3是为了将弧度转化为角度*/
  printf("\r\nPitch = %f度    Roll = %f度     Yaw = %f度    ", pitch, roll, yaw);
}


///*******************************************************************************
//* Function Name  : IMUupdate
//* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick.
//* Input          : None
//* Output         : None
//* Return         : None

//// q0 q1 q2 q3需要初始化才能代入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
//// 1.首先校准accle gyro mag；
//// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
////   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
//// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
//// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，
//		 然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
//// 5.或直接使用四元数；
//// 6.重复4，即可更新姿态;

////总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
////以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
////前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
////以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

////在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
////然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

////欧拉角单位为弧度radian，乘以57.3以后转换为角度
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
  //auxiliary variables to reduce number of repeated operations，
	float q0q0 = (quaternion->w)*(quaternion->w);
	float q0q1 = (quaternion->w)*(quaternion->xi);
	float q0q2 = (quaternion->w)*(quaternion->yj);
	float q1q1 = (quaternion->xi)*(quaternion->xi);
	float q1q3 = (quaternion->xi)*(quaternion->zk);
	float q2q2 = (quaternion->yj)*(quaternion->yj);   
	float q2q3 = (quaternion->yj)*(quaternion->zk);
	float q3q3 = (quaternion->zk)*(quaternion->zk); 
  /*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
  //normalise the measurements
	//axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力
	//向量
	vector_normalize(accel);
	
	//vx，vy，vz其实就是上一次的欧拉角（四元数）的机体坐标参考系
	//换算出来的重力的单位向量
	//奇怪了，原版算法中，都用的是halfvx, halfvy, halfvz。两个算法都有这个区别
	vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

      
  //现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	//axyz和vxyz都是机体坐标参照系上的重力向量，那它们之间的误差，就是陀螺
	//积分后的姿态和加速度计测出来的姿态之间的误差
	//向量间的误差，可以用向量叉积来表示，exyz就是两个重力向量的叉积
	// vx\vy\vz就是上一轮惯性姿态角测量结果
	ex = ((accel->data[1])*vz - (accel->data[2])*vy);
	ey = ((accel->data[2])*vx - (accel->data[0])*vz);
	ez = ((accel->data[0])*vy - (accel->data[1])*vx);
 
  //以下的计算是在用叉积误差来做PI修正陀螺零偏
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

	// integrate quaternion rate and normalise，四元数更新算法，一阶龙格-库塔法
	//还是以陀螺仪数据为准，加速度计的数据是用来修正陀螺仪的数据
	//在循环中不断更新四元数，然后就得出了不断更新的
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
////0.9和0.1是修正系数，其中5.73=0.1*rad_to_deg，乘以57.3是为了将弧度转化为角度，该公式意思是将磁力计的长期准确度和
////陀螺仪的高灵敏度进行互补滤波，即对陀螺仪的数据进行高通滤波，对磁力计的数据进行低通滤波，再相加  


/*由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;*/
//由四元数计算出Pitch  Roll  Yaw,乘以57.3是为了将弧度转化为角度
  q0 = quaternion->w;
  q1 = quaternion->xi;
  q2 = quaternion->yj;
  q3 = quaternion->zk;	
#if defined ZXY
  pitch = asin(2 * q2 * q3 + 2 * q0 * q1); //俯仰角，绕x轴转动	 
  roll  = -atan2(2 * q1 * q3 - 2 * q0 * q2, -2 * q1 * q1 - 2 * q2* q2 + 1); //滚动角，绕y轴转动
//乘以57.3是为了将弧度转化为角度
//q0是w，q1是x，q2是y，q3是z
#elif defined ZYX
	pitch = asin(-2 * q1 * q3 + 2 * q0 * q2); //俯仰角，绕y轴转动	 
  roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); //滚动角，绕x轴转动
#endif
//陀螺仪在短时间内有用，长时间的积分会有误差，而加速度计刚好反过来，短时间内不可靠
//，但是长时间的积分结果就比较准确
//0.9和0.1是修正系数，其中5.73=0.1*57.3，乘以57.3是为了将弧度转化为角度，该公式意思是将磁力计的长期准确度和
//陀螺仪的高灵敏度进行互补滤波，即对陀螺仪的数据进行高通滤波，对磁力计的数据进行低通滤波，再相加
//滚动角，绕y轴转动
//	Yaw   = (0.9f * (Yaw + init_gz*2*halfT) + 5.73f * atan2(init_mx*cos(Roll) + init_my*sin(Roll)*sin(Pitch) + init_mz*sin(Roll)*cos(Pitch), init_my*cos(Pitch) - init_mz*sin(Pitch)));
//滚动角，绕x轴转动,这种融合办法虽然不会漂移，但是到+-170之后，就会剧烈波动，相比AHRS差远了
//之所以和上面一行代码是一样的，是因为init_mx和init_my已经变换过了
	*yaw   = (0.9f * (*yaw + (gyro->data[2])*2*halfT*57.3f) + 5.73f * atan2((mag->data[0])*cos(roll) + (mag->data[1])*sin(roll)*sin(pitch) + (mag->data[2])*sin(roll)*cos(pitch), (mag->data[1])*cos(pitch) - (mag->data[2])*sin(pitch)));
	pitch = pitch * (float)rad_to_deg;
	roll = roll * (float)rad_to_deg;
	//试一下另一个方案，即不使用磁力计的数据
	//试过之后，这个的Yaw会漂移。。。
//	Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * (float)rad_to_deg;
	
//	printf("q0=%f, q1=%f, q2=%f, q3=%f, Yaw=%f, Pitch=%f, Roll=%f \n\r", q0, q1, q2, q3, Yaw, Pitch, Roll);
  printf("\r\nPitch=%f度    Roll=%f度     Yaw=%f度    ", pitch, roll, *yaw);
}

void transform_data(short *gyro, short* accel, matrix *gyro_mat, matrix *accel_mat, float *bias){
	//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，即Gyro_500_Scale_Factor，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
	 //然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)
	 //计算初始化四元数时，没有用到陀螺仪的数据
	
		gyro_mat->data[0] = (float)(((float)gyro[0]-bias[0]) / (Gyro_500_rad_Scale_Factor));	  	     //单位转化成：弧度/s
		gyro_mat->data[1] = (float)(((float)gyro[1]-bias[1]) / (Gyro_500_rad_Scale_Factor));
		gyro_mat->data[2] = (float)(((float)gyro[2]-bias[2]) / (Gyro_500_rad_Scale_Factor));
			 //此处这些加速度计的数据没有像计算初始化四元数的时候那样处理，因为不需要，后面都归一化了
//	 	 accel_mat->data[0] = (float)accel[0];	  
//		 accel_mat->data[1] = (float)accel[1];
//		 accel_mat->data[2] = (float)accel[2];
		accel_mat->data[0] = (float)(accel[0]/Accel_4_M_Scale_Factor); //单位转化成重力加速度的单位：m/s2
		accel_mat->data[1] = (float)(accel[1]/Accel_4_M_Scale_Factor);
		accel_mat->data[2] = (float)((accel[2])/Accel_4_M_Scale_Factor);
}

void transform_data_better(short *gyro, short* accel, matrix *gyro_mat, matrix *accel_mat, float *gyro_bias, float *acc_bias){
	//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，即Gyro_500_Scale_Factor，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
	 //然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)
	 //计算初始化四元数时，没有用到陀螺仪的数据
	
		gyro_mat->data[0] = (((float)gyro[0]-gyro_bias[0]) / (Gyro_500_rad_Scale_Factor));	  	     //单位转化成：弧度/s
		gyro_mat->data[1] = (((float)gyro[1]-gyro_bias[1]) / (Gyro_500_rad_Scale_Factor));
		gyro_mat->data[2] = (((float)gyro[2]-gyro_bias[2]) / (Gyro_500_rad_Scale_Factor));
			 //此处这些加速度计的数据没有像计算初始化四元数的时候那样处理，因为不需要，后面都归一化了
//	 	 accel_mat->data[0] = (float)accel[0];	  
//		 accel_mat->data[1] = (float)accel[1];
//		 accel_mat->data[2] = (float)accel[2];
		accel_mat->data[0] = (((float)accel[0]-acc_bias[0])/Accel_4_M_Scale_Factor); //单位转化成重力加速度的单位：m/s2
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

// 确定每个加速度计的常值偏移过程：先使用U3的数据进行姿态解算，算出姿态后，就可以通过解重力分量方程，解出每个轴上的常值偏移
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
	// 先读取数据，缓冲一下，以提高数据的平稳性
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
			// 化成ADCshort类型的数
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
//	printf("Pitch=%f度    Roll=%f度     Yaw=%f度\n", getPitch_m()*rad_to_deg, getRoll_m()*rad_to_deg, getYaw_m()*rad_to_deg);
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
	// U3绕X轴的角速度和U4绕X轴的角速度、U6绕X轴的角速度应该是一样的，同样的，U3绕Y轴的角速度和U2绕Y轴的角速度、U5绕Y轴的角速度是一样的
	// 现将其进行平均处理，以减少噪声
	gyro_3->data[0] = (gyro_4->data[0]+gyro_6->data[0]+gyro_3->data[0])/3.0f;
	gyro_3->data[1] = (gyro_2->data[1]+gyro_5->data[1]+gyro_3->data[1])/3.0f;
}
// U3和U4和U6的roll是一样的，U3和U2和U5的pitch是一样的，
// 下面用周围四个加速度计的数据去融合中间的陀螺仪z轴角速度数据，四个加速度计的轴不一样，主要是利用离心力
		// 对于u2，是y轴正方向的数据，对于u4，是x轴正方向的数据，对于u5，是y轴负方向的数据，对于u6，是x轴负方向的数据
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

// U3和U4和U6的roll是一样的，U3和U2和U5的pitch是一样的，
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


////此算法和上面那个算法大概有三个地方是不一样的
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
//	//某种坐标系下，绕Y轴是pitch是theta，绕X轴是roll是fai，yaw是psi
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * rad_to_deg; //俯仰角，绕y轴转动	 
//	Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * rad_to_deg; //滚动角，绕x轴转动
//	Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * (float)rad_to_deg;
//	if(Yaw < 0 ){Yaw = Yaw + 360;}
//	if(Yaw > 360 ){Yaw = Yaw - 360;}
//	//乘以57.3是为了将弧度转化为角度*/
//  printf("\r\nPitch = %f度    Roll = %f度     Yaw = %f度    ", Pitch, Roll, Yaw);
//}
