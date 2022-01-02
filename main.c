#include "string.h"
#include "sys.h"
#include "delay.h"  
//#include "usart.h"
#include "usart1.h"
#include "usart2.h"
#include "led.h"
#include "i2c.h"
#include "oled.h"
#include "key.h"
#include "lora_app.h"
#include "stm32f4xx.h"
#include "stm324x7i_eval.h"
#include <stdio.h>
#include "delay.h"
#include "sys.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "i2c.h"
#include "Time.h"
#include "common.h"
#include "malloc.h"
#include "mpu6050_1.h"
#include "mpu6050_2.h"
#include "mpu6050_3.h"
#include "mpu6050_4.h"
#include "mpu6050_5.h"
#include "attitude.h"
#include "ekf.h"
#include "dcm.h"
/*******************************************************************************************************
//本程序的读写mpu9150部分是在http://www.amobbs.com/thread-5538389-1-1.html，作者_spetrel的基础上修改而来
//本程序Yaw的互补滤波出自Hom19910422@gmail.com
//本程序采用的gyro accel融合算法出自下面的作者Madgwick
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
*******************************************************************************************************/

//----------------------------------------------------------------------------------------------------


/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/*神州III号LED灯相关定义*/
#define RCC_GPIO_LED                    RCC_AHB1Periph_GPIOF    /*LED使用的GPIO时钟*/
//#define LEDn                            2                       /*神舟III号LED数量*/
#define GPIO_LED                        GPIOF                   /*神舟III号LED灯使用的GPIO组*/

#define DS1_PIN                         GPIO_Pin_10              /*DS1使用的GPIO管脚*/
#define DS0_PIN                         GPIO_Pin_9              /*DS0使用的GPIO管脚*/

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
//#define DEFAULT_MPU_HZ  (100)
#define DEFAULT_MPU_DMP_HZ (200)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)


/* Private macro -------------------------------------------------------------*/


//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
  
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
void Clock_Enable(void);
void GPIO_Configuration(void);
void systickInit (uint16_t frequency);

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 * 陀螺仪的旋转矩阵，用于旋转陀螺仪的轴向，注意，轴向要符合右手定理，
 * 即4个手指指向x轴并向y轴握拳，此时竖起的拇指方向为z轴方向
 */
//static signed char gyro_orientation[9] = { 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1};
//static signed char gyro_orientation[9] = { 0, 1, 0,
//                                          -1, 0, 0,
//                                           0, 0, 1};
//static signed char gyro_orientation[9] = { -1,  0, 0,
//                                            0, -1, 0,
//                                            0,  0, 1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
	int result;
//    char test_packet[4] = {0};
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	printf("result: %d", result);
	if (result == 0x3  ) 
		{
			/* Test passed. We can trust the gyro data here, so let's push it down
			 * to the DMP.
			 */
			float sens;
			unsigned short accel_sens;
			mpu_get_gyro_sens(&sens);
			gyro[0] = (long)(gyro[0] * sens);
			gyro[1] = (long)(gyro[1] * sens);
			gyro[2] = (long)(gyro[2] * sens);
			dmp_set_gyro_bias(gyro);
			mpu_get_accel_sens(&accel_sens);
			accel[0] *= accel_sens;
			accel[1] *= accel_sens;
			accel[2] *= accel_sens;
			dmp_set_accel_bias(accel);
			printf("\r\n setting bias succesfully ......\n\r");
		}
	else
		{
			printf("\r\n bias has not been modified ......\n\r");
			GPIO_SetBits(GPIOB, GPIO_Pin_9);
			while(1);
		}
}

#define Accel_Zout_Offset		600
#define Gyro_Xout_Offset	    -70
#define Gyro_Yout_Offset		25
#define Gyro_Zout_Offset		-10
//#define halfT 0.5f  //half the sample period,halfT 0.5f需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度

//#define q30  1073741824.0f
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
/*******************************************************************************
* Function Name  : main
* Description    : 读取MPU-9150's accel gyro mag数据.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
	int count = 100000;
	int mod = 0;
	int i = 0;
	float gyro_bias_3[3], gyro_bias_2[3], gyro_bias_4[3], gyro_bias_5[3], gyro_bias_6[3];
	float acc_bias_3[3], acc_bias_2[3], acc_bias_4[3], acc_bias_5[3], acc_bias_6[3];
	unsigned long timestamp;
  short gyro[3], accel[3];
	short gyro_2[3], accel_2[3];
	short gyro_4[3], accel_4[3];
	short gyro_5[3], accel_5[3];
	short gyro_6[3], accel_6[3];
	float yaw, pitch, roll;
	float halfT;
	float acc_centri_2, acc_centri_4, acc_centri_5, acc_centri_6, acc_centri;
	float ax_g, ay_g;
	float gyro_new_z;
	float last_yaw;
	float last_gyro_z;
	// 采集温度曲线的数据
	float last_temp, current_temp;
	short count_temp = 100;
	// 尝试卡尔曼滤波融合多传感器数据
	double s, P;
	// 试试滑动均值滤波
	float gyro_3_z[10];
	long temp;
//	float time_stamp = 0.0f;
//	u32 current_val = 0;
	matrix *gyro_mat = matrix_init(3, 1);
	matrix *accel_mat = matrix_init(3, 1);
	
	matrix *gyro_mat_3 = matrix_init(3, 1);
	matrix *accel_mat_3 = matrix_init(3, 1);
	
	matrix *gyro_mat_2 = matrix_init(3, 1);
	matrix *accel_mat_2 = matrix_init(3, 1);

	matrix *gyro_mat_4 = matrix_init(3, 1);
	matrix *accel_mat_4 = matrix_init(3, 1);

	matrix *gyro_mat_5 = matrix_init(3, 1);
	matrix *accel_mat_5 = matrix_init(3, 1);

	matrix *gyro_mat_6 = matrix_init(3, 1);
	matrix *accel_mat_6 = matrix_init(3, 1);

	quaternion_struct quaternion;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */      
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
	delay_init(168);
  SYSTICK_INIT(); 
	usart1_init(921600);
	usart2_init(921600);
	I2C_Configuration();
//	printf("hahaah\n");
	//开启wifi模块，设置为STA模式，连接已有的热点，
	//然后设置为TCP客户端，开启透传模式
//	atk_8266_init();

 
	//整个main的流程是：先初始化stm32的硬件，即MPU的外围硬件，如总线时钟、串口、I2C等，再初始化
	//MPU，再对MPU中的各个传感器进行设置，再就是得到初始四元数，然后就是不断的循环的进行姿态解算。
	//DMP是一个在MPU和I2C之间传输数据的组件
//	printf("gyro 3\n");
	choose_iic_3();
	mpu_conf();
	gyro_calibrate(gyro_bias_3);
//	acc_calibrate(acc_bias_3, gyro_bias_3);
	acc_calibrate_horizon(acc_bias_3);
//	i2cReadBuffer(0x68, 0x19, 1, &data);
//	printf("\r\ndata: %d", data);
//  printf("gyro 2\n");
	choose_iic_2();
	mpu_conf();
	gyro_calibrate(gyro_bias_2);
//	acc_calibrate(acc_bias_2, gyro_bias_2);
	acc_calibrate_horizon(acc_bias_2);
//	printf("gyro 4\n");
	choose_iic_4();
	mpu_conf();
	gyro_calibrate(gyro_bias_4);
//	acc_calibrate(acc_bias_4, gyro_bias_4);
	acc_calibrate_horizon(acc_bias_4);
//	printf("gyro 5\n");
	choose_iic_5();
	mpu_conf();
	gyro_calibrate(gyro_bias_5);
//	acc_calibrate(acc_bias_5, gyro_bias_5);
	acc_calibrate_horizon(acc_bias_5);
//	printf("gyro 6\n");
	choose_iic_6();
	mpu_conf();
	gyro_calibrate(gyro_bias_6);
//	acc_calibrate(acc_bias_6, gyro_bias_6);
	acc_calibrate_horizon(acc_bias_6);
//	
  init_quaternion(&quaternion);   //得到初始化四元数
	start_systick(0xFFFFFF);
//	DCM_IMU_uC_init(DEFAULT_g0, NULL, NULL, DEFAULT_q_dcm2, DEFAULT_q_gyro_bias2, DEFAULT_q_dcm2_init, DEFAULT_q_gyro_bias2_init, DEFAULT_r_acc2, DEFAULT_r_a2);
  DCM_IMU_uC_init_m(DEFAULT_g0, NULL, NULL, DEFAULT_q_dcm2, DEFAULT_q_gyro_bias2, DEFAULT_q_dcm2_init, DEFAULT_q_gyro_bias2_init, DEFAULT_r_acc2, DEFAULT_r_a2);
//	choose_iic_3();
//	for(i=0;i<10;i++){
//		mpu_get_gyro_reg(gyro, 0);
//		transform_data_better(gyro, accel, gyro_mat_3, accel_mat_3, gyro_bias_3, acc_bias_3);
//		gyro_3_z[i] = gyro_mat_3->data[2];
//	}
  last_yaw = 0;
	s = 0;
	P = 0.000003;
//	P = (0.0000004736+0.0000004065+0.0000005359+0.000002815+0.0000006885)/25.0;
//	P = (0.0000004736+0.0000004065+0.0000005359+0.0000006885)/16.0;
//	P = 0.0000007;
  last_temp = 29.50;
	current_temp = 0.0;
	
  while(1)
  {
		choose_iic_3();
		mpu_get_gyro_reg(gyro, 0);
		mpu_get_accel_reg(accel, 0);
//		mpu_get_temperature(&temp, 0);
		choose_iic_2();
		mpu_get_gyro_reg(gyro_2, 0);
		mpu_get_accel_reg(accel_2, 0);
//		mpu_get_temperature(&temp, 0);
		choose_iic_4();
		mpu_get_gyro_reg(gyro_4, 0);
		mpu_get_accel_reg(accel_4, 0);
//		mpu_get_temperature(&temp, 0);
		choose_iic_5();
		mpu_get_gyro_reg(gyro_5, 0);
		mpu_get_accel_reg(accel_5, 0);
//		mpu_get_temperature(&temp, 0);
		choose_iic_6();
		mpu_get_gyro_reg(gyro_6, 0);
		mpu_get_accel_reg(accel_6, 0);
		mpu_get_temperature(&temp, 0);
//		transform_data(gyro, accel, gyro_mat, accel_mat, gyro_bias_3);
//		transform_data(gyro_2, accel_2, gyro_mat_2, accel_mat_2, gyro_bias_2);
//		transform_data(gyro_4, accel_4, gyro_mat_4, accel_mat_4, gyro_bias_4);
//		transform_data(gyro_5, accel_5, gyro_mat_5, accel_mat_5, gyro_bias_5);
//		transform_data(gyro_6, accel_6, gyro_mat_6, accel_mat_6, gyro_bias_6);
//		gyro_fusion(gyro_mat, gyro_mat_2, gyro_mat_4, gyro_mat_5, gyro_mat_6);
		transform_data_better(gyro_2, accel_2, gyro_mat_2, accel_mat_2, gyro_bias_2, acc_bias_2);
		transform_data_better(gyro_4, accel_4, gyro_mat_4, accel_mat_4, gyro_bias_4, acc_bias_4);
		transform_data_better(gyro_5, accel_5, gyro_mat_5, accel_mat_5, gyro_bias_5, acc_bias_5);
		transform_data_better(gyro_6, accel_6, gyro_mat_6, accel_mat_6, gyro_bias_6, acc_bias_6);
		transform_data_better(gyro, accel, gyro_mat_3, accel_mat_3, gyro_bias_3, acc_bias_3);
//		sensor_fusion(gyro_mat_3, accel_mat_2, accel_mat_4, accel_mat_5, accel_mat_6);
//		printf("ax3: %d\n", accel[0]);
//		printf("ax2: %d\n", accel_2[0]);
//		printf("az4: %d\n", accel_4[2]);
//		printf("ax5: %d\n", accel_5[0]);
//		printf("ax6: %d\n", accel_6[0]);
//		gyro_new_z = gyro_mat_3->data[2];
//		for(i=1;i<10;i++){
//			gyro_mat_3->data[2]+=gyro_3_z[i];
//	  }
//		for(i=0;i<9;i++){
//			gyro_3_z[i]=gyro_3_z[i+1];
//	  }
//		gyro_3_z[i] = gyro_new_z;
//		choose_iic_3();
//		i=10;
//		while(i)
//		{
//			mpu_get_gyro_reg(gyro, 0);
//		  mpu_get_accel_reg(accel, 0);
//		  transform_data_better(gyro, accel, gyro_mat_3, accel_mat_3, gyro_bias_3, acc_bias_3);
//			gyro_mat->data[0] += gyro_mat_3->data[0];
//			gyro_mat->data[1] += gyro_mat_3->data[1];
//			gyro_mat->data[2] += gyro_mat_3->data[2];
//			accel_mat->data[0] += accel_mat_3->data[0];
//			accel_mat->data[1] += accel_mat_3->data[1];
//			accel_mat->data[2] += accel_mat_3->data[2];
////			for(i=0;i<3;i++){
////				printf("gyro: %d\n", gyro[i]);
////			}
////			for(i=0;i<3;i++){
////				printf("accel: %d\n", accel[i]);
////			}
//			i--;
//		}
//		gyro_mat->data[0] /= 10.0f;
//		gyro_mat->data[1] /= 10.0f;
//		gyro_mat->data[2] /= 10.0f;
//		accel_mat->data[0] /= 10.0f;
//		accel_mat->data[1] /= 10.0f;
//		accel_mat->data[2] /= 10.0f;
//		mpu_get_gyro_reg(gyro, 0);
//		mpu_get_accel_reg(accel, 0);
//		
//		printf("acc_centri: %f\n", acc_centri);
//		acc_centri = acc_centri>0?acc_centri:0;
//		printf("Z轴角速度: %f\n", gyro_mat_3->data[2]);
//		printf("离心力转换成的角速度: %f\n", acc_centri);
//		gyro_mat->data[2] = 0.9*sqrtf(acc_centri/0.07)+0.1*gyro_mat->data[2];

//		acc_centri = (acc_centri_2+acc_centri_4+acc_centri_5+acc_centri_6)/4.0f;
//		if(acc_centri<0){
//			acc_centri = 0;
//		}
//		printf("Z轴角速度: %f\n", gyro_mat_3->data[2]);
//		printf("离心力转换成的角速度: %f\n", sqrtf(acc_centri/0.07));
//    printf("acc_centri: %f\n", acc_centri);
//		printf("acc_centri_2: %f\n", acc_centri_2);
//		printf("acc_centri_4: %f\n", acc_centri_4);
//		printf("acc_centri_5: %f\n", acc_centri_5);
//		printf("acc_centri_6: %f\n", acc_centri_6);
//		acc_yaw = ax6<(acc_yaw=ay5<(acc_yaw=ay2<ax4?ay2:ax4)?ay5:acc_yaw)?ax6:acc_yaw;
//		printf("acc_yaw: %f\n", acc_yaw);
//		printf("w3z: %f\n", sqrt(acc_yaw/0.07)*rad_to_deg);
//		printf("w3z: %f\n", gyro_mat->data[2]*rad_to_deg);
//		 delay_ms(2);
//		 printf("   ax=%f,  ay=%f,  az=%f\n", accel[0]*DEFAULT_g0/Accel_4_Scale_Factor, accel[1]*DEFAULT_g0/Accel_4_Scale_Factor, accel[2]*DEFAULT_g0/Accel_4_Scale_Factor);
//		 printf("    gx=%f,   gy=%f,   gz=%f \n", gyro_mat->data[0], gyro_mat->data[1], gyro_mat->data[2]);
//     printf("aaaaa\n");
     get_sample_interval(&halfT);
//		 halfT = GET_NOWTIME();
//		 time_stamp += (0xFFFFFF-current_val)/21000000.0f;
//     printf(" t=%f,     ax=%d,  ay=%d,  az=%d,   gx=%d,   gy=%d,   gz=%d \n", time_stamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
		 start_systick(0xFFFFFF);
//		 printf("    gx=%d,   gy=%d,   gz=%d \n", gyro[0], gyro[1], gyro[2]);

//		 printf("   ax=%f,  ay=%f,  az=%f", init_ax,init_ay,init_az);
//     AHRSupdate(gyro_mat, accel_mat, mag_mat, &quaternion); 
//		 IMUupdate(gyro_mat, accel_mat, mag_mat, &quaternion, &yaw, halfT);
//		 printf("halfT: %f\n", halfT);
//		 updateIMU(gyro_mat->data, accel_mat->data, halfT);
//     if(fabs((gyro_mat_3->data[2]-last_gyro_z)/(halfT*4.0f))<1.0f && fabs(gyro_mat_3->data[2])<0.003f){
//			 gyro_mat_3->data[2] = 0.0f;
//		 }
//     printf("角加速度: %f\n", );
//     if((gyro_mat_3->data[2]-last_gyro_z)/(halfT*4.0f))
		 updateIMU_m(gyro_mat_3->data, accel_mat_3->data, halfT);
//		 printf("bbbbb\n");
//		 printf("\r\nPitch=%f度    Roll=%f度     Yaw=%f度    ", getPitch()*rad_to_deg, getRoll()*rad_to_deg, getYaw()*rad_to_deg);
     pitch = getPitch_m();
		 roll = getRoll_m();   
//     printf("gyro_z_rate: %f, Yaw=%f度\n", gyro_mat_3->data[2]*57.3, (getYaw_m()-last_yaw)*rad_to_deg);
//		 last_yaw += gyro_mat_3->data[2]*rad_to_deg*halfT;
		 last_yaw += s*rad_to_deg*halfT;
//		 last_gyro_z = gyro_mat_3->data[2];
//     printf("g6x=%f,  g6y=%f,  g6z=%f\n", gyro_mat_6->data[0], gyro_mat_6->data[1], gyro_mat_6->data[2]);
//		 printf("Yaw=%f  temp=%f\n", last_yaw, (float)(temp)/65536.0f);
//     current_temp = (float)(temp)/65536.0f;
//     if((current_temp-last_temp)>0.1&&(current_temp-last_temp)<0.2)
//		 {
//			 if(count_temp!=0)
//			 {
//				 printf("g3z=%f  temp=%f\n", gyro_mat_3->data[2], (float)(temp)/65536.0f);
//				 count_temp -= 1;
//			 }else{
//				 last_temp += 0.1;
//				 count_temp = 100;
//			 }
//		 }
//		 delay_ms(100);
//     printf("g3z=%f  temp=%f\n", gyro_mat_3->data[2], (float)(temp)/65536.0f);
     printf("g6z=%f  a6y=%f  temp=%f\n", gyro_mat_6->data[2], accel_mat_6->data[1], (float)(temp)/65536.0f);
//		 printf("g3z=%f  g2z=%f  g4z=%f  g5z=%f  g6z=%f  temp=%f\n", gyro_mat_3->data[2], gyro_mat_2->data[2], gyro_mat_4->data[2], gyro_mat_5->data[2], gyro_mat_6->data[2], (float)(temp)/65536.0f);
//		 printf("U3:%f   U2:%f\n", gyro_mat_3->data[2], gyro_mat_2->data[2]);
//		 printf("Pitch=%f度    Roll=%f度     Yaw=%f度\n", getPitch_m()*rad_to_deg, getRoll_m()*rad_to_deg, last_yaw);
//     printf("halfT: %f\n", halfT);
  }
}

/*******************************************************************************
* Function Name  : GPIO 
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

//  Led1=1;
//  Led2=1; 
//  Led3=1; 
//  Led4=1;   	
}
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/**
  * @brief  systickInit program
  * @param  None
  * @retval None
  */
void systickInit (uint16_t frequency)
{
   RCC_ClocksTypeDef RCC_Clocks;
   RCC_GetClocksFreq (&RCC_Clocks);
   (void) SysTick_Config (RCC_Clocks.HCLK_Frequency / frequency);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//  USART_SendData(EVAL_COM2, (uint8_t) ch); /*发送一个字符函数*/ 

//  /* Loop until the end of transmission */
//  while (USART_GetFlagStatus(EVAL_COM2, USART_FLAG_TC) == RESET)/*等待发送完成*/
//  {
//  
//  }
//  return ch;
//}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

