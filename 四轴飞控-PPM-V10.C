/*------------------------------------------------------------------*/
/* --- STC MCU Limited ---------------------------------------------*/
/* --- STC89-90xx Series MCU UART (8-bit/9-bit)Demo ----------------*/
/* --- Mobile: (86)13922805190 -------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ---------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966----------------------*/
/* --- Web: www.STCMCU.com -----------------------------------------*/
/* --- Web: www.GXWMCU.com -----------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/*------------------------------------------------------------------*/

/* 本程序经过测试完全正常, 不提供电话技术支持, 如不能理解, 请自行补充相关基础.  */

/***  特别注意: 下载时选择内部时钟24MHZ, 设置用户EEPROM大小为2K或以上.  ****/

/*********************************************
  四轴飞控-V10.C

使用遥控接收器型号: MC6B六通道2.4G 100mW.

四轴上电待机：上电后，航灯不亮，接收机LED闪烁，此时打开遥控器，将左右油门下拉到最小，接收机收到信号LED常亮，
              表示RF通讯已连接。此时蜂鸣器"哔"一声，航灯闪烁，表示待机模式。

四轴启动：将遥控器左右操纵杆掰成下内八，启动四轴，四轴"哔"一声，4个螺旋桨开始低速旋转，航灯常亮。
          此后提升油门，就可以加速螺旋桨，直到起飞。

四轴飞行：起飞后，可以操纵右手的俯仰、横滚操纵杆，实现前后左右或任意方向的飞行。
          左手油门杆左掰是航向逆时针转，右掰是航向顺时钟转。

四轴下降停止：收油门，四轴逐渐下降到地面，然后两操纵杆掰成下外八，停止四轴，重新处于待机模式。

四轴水平校准：将四轴放置于水平地面，处于待机模式，然后两操纵杆掰成上内八，四轴"哔"一声进入校准，完成后"哔哔"两声完成校准。

四轴取消水平校准：将四轴放置于水平地面，处于待机模式，然后两操纵杆掰成上外八，四轴"哔"一声取消校准。取消水平校准或未进行水平校准过的四轴，起飞时即使无风也可能会有明显漂移。

电池低压报警：当电池低压时，蜂鸣器"哔哔"报警，同时航灯闪烁，此时请尽快回航降落。

无遥控信号异常：当四轴在空中突然收不到遥控信号时，四轴蜂鸣器发出"哔哔哔"报警，同时航灯闪烁，四轴保持水平，逐渐自动减小油门降落。


***********************************************/

#define		Baudrate1			115200UL
#define		TX1_LENGTH	128
#define		RX1_LENGTH	128


#include "config.h"
#include "STC8xxx_PWM.H"
#include "MPU6050.H"
#include "AD.H"	
#include "EEPROM.H"
#include "PCA.h"
#include <math.H>

sbit	P_Light  = P5^4;	//航灯
sbit	P_BUZZER = P5^5;	//蜂鸣器


int		xdata g_x=0,g_y=0,g_z=0;					//陀螺仪矫正参数
float	xdata a_x=0,a_y=0;							//角度矫正参数
float	data  AngleX=0,AngleY=0;					//四元数解算出的欧拉角
float	xdata Angle_gx=0,Angle_gy=0,Angle_gz=0;		//由角速度计算的角速率(角度制)
float	xdata Angle_ax=0,Angle_ay=0,Angle_az=0;		//由加速度计算的加速度(弧度制)
float	xdata Ax=0,Ay=0,Az=0;						//加入遥控器控制量后的角度    
float	data PID_x=0,PID_y=0,PID_z=0;				//PID最终输出量
int		data  speed0=0,speed1=0,speed2=0,speed3=0;	//电机速度参数
int		data  PWM0=0,PWM1=0,PWM2=0,PWM3=0;//,PWM4=0,PWM5=0;			//加载至PWM模块的参数

int		int_tmp;
u8		YM=0,FRX=128,FRY=128,FRZ=128;				//4通道遥控信号.
u8		xdata	tp[16];		//读MP6050缓冲


//****************姿态处理和PID*********************************************

float xdata Out_PID_X=0,Last_Angle_gx=0;					//外环PI输出量  上一次陀螺仪数据
float xdata ERRORX_Out=0,ERRORX_In=0;			//外环P  外环I  外环误差积分
float xdata Out_PID_Y=0,Last_Angle_gy=0;
float xdata ERRORY_Out=0,ERRORY_In=0;            //规则1:内外环P乘积等于10.5

float xdata Last_Ax=0,Last_Ay=0,Last_Az=0;


/******************************************************************************/
#define	Out_XP	6.65f	//ADC0	外环P	V1 / 10
#define	Out_XI	0.0074f	//ADC4	外环I	V2 / 10000
#define	Out_XD	6.0f	//ADC5	外环D	V3 / 10

#define	In_XP	0.8275f	//ADC6	内环P	V4 / 100
#define	In_XI	0.0074f	//ADC4	内环I	V2 / 10000
#define	In_XD	6.0f	//ADC5	内环D	V3 / 10


#define	Out_YP	Out_XP
#define	Out_YI	Out_XI
#define	Out_YD	Out_XD

#define	In_YP	In_XP
#define	In_YI	In_XI
#define	In_YD	In_XD


#define	ZP	5.0f
#define	ZI	0.1f
#define	ZD	4.0f	//自旋控制的P D
float Z_integral=0;//Z轴积分

#define	ERR_MAX	500
//======================================================================


u8	data YM_LostCnt=0, Lost16S; //上一次RxBuf[0]数据(RxBuf[0]数据在不断变动的)   状态标识
u8	SW2_tmp;


//======================================================================
bit	B_8ms;	//8ms标志

bit	B_rtn_ADC0;	//请求返回信息
bit	B_BAT_LOW;	//低电压标志
u8	xdata cnt_ms;		//时间计数


u8		xdata UART1_cmd=0;	//串口命令
u8		xdata TX1_Read=0;	//发送读指针
u8		xdata TX1_Write=0;	//发送写指针
u8		xdata TX1_cnt=0;	//发送计数
u8 		xdata TX1_Buffer[TX1_LENGTH];	//发送缓冲
bit		B_TX1_Busy;			//发送忙标志
u8 		xdata RX1_Cnt,RX1_Timer;
u8 		xdata RX1_Buffer[RX1_LENGTH];
bit 	B_RX1_OK;


u8		xdata Cal_Setp=0;			//校准步骤
u8		xdata Cal_cnt=0;			//校准平均值计数
int		xdata x_sum,y_sum,z_sum;	//校准累加和
float	xdata float_x_sum,float_y_sum;	//校准累加和

u8	xdata BuzzerOnTime,BuzzerOffTime,BuzzerRepeat,BuzzerOnCnt,BuzzerOffCnt;
u8	xdata cnt_100ms;


/* =================== PPM接收相关变量 ========================== */
u16	xdata CCAP0_RiseTime;		//捕捉到的上升沿时刻
u8	xdata PPM1_Rise_TimeOut;	//高电平限时
u8	xdata PPM1_Rx_TimerOut;		//接收超时计数
u8	xdata PPM1_RxCnt;			//接收次数计数
u16	xdata PPM1_Cap;				//捕捉到的PPM脉冲宽度
bit	B_PPM1_OK;					//接收到一个PPM脉冲宽度

u16	xdata CCAP1_RiseTime;
u8	xdata PPM2_Rise_TimeOut;	//高电平限时
u8	xdata PPM2_Rx_TimerOut;
u8	xdata PPM2_RxCnt;
u16	xdata PPM2_Cap;
bit	B_PPM2_OK;

u16	xdata CCAP2_RiseTime;
u8	xdata PPM3_Rise_TimeOut;	//高电平限时
u8	xdata PPM3_Rx_TimerOut;
u8	xdata PPM3_RxCnt;
u16	xdata PPM3_Cap;
bit	B_PPM3_OK;

u16	xdata CCAP3_RiseTime;
u8	xdata PPM4_Rise_TimeOut;	//高电平限时
u8	xdata PPM4_Rx_TimerOut;
u8	xdata PPM4_RxCnt;
u16	xdata PPM4_Cap;
bit	B_PPM4_OK;

u16	xdata CCAP_FallTime;

u8	PPM1,PPM2,PPM3,PPM4;
bit	B_Start;
u8	cnt_start;

/* ============================================= */


void	UART1_config(void);
void 	PrintString1(u8 *puts);	//发送一个字符串
void	TX1_write2buff(u8 dat);	//写入发送缓冲，指针+1
void	TX1_int_value(int i);
void	delay_ms(u8 ms);
void	Return_Message(void);
u16 	MODBUS_CRC16(u8 *p,u8 n);	//input:	*p--->First Data Address,n----->Data Number,	return:	CRC16
void	PCA_config(void);
void 	Timer0_Config(void);
void 	Timer1_Config(void);
void	return_TTMx(u8 id,PPMx);
void 	Timer0_Config(void);
u16 	MODBUS_CRC16(u8 *p,u8 n);	//input:	*p--->First Data Address,n----->Data Number,	return:	CRC16

extern xdata u16	adc0;
extern xdata int	Battery;


//*********************************************************************
//****************角度计算*********************************************
//*********************************************************************
#define	pi		3.14159265f                           
#define	Kp		0.8f                        
#define	Ki		0.001f                         
#define	halfT	0.004f           

float idata q0=1,q1=0,q2=0,q3=0;   
float idata exInt=0,eyInt=0,ezInt=0;  


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float data norm;
	float idata vx, vy, vz;
	float idata ex, ey, ez;

	norm = sqrt(ax*ax + ay*ay + az*az);	//把加速度计的三维向量转成单维向量   
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

		//	下面是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。 
		//	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素
		//	所以这里的vx vy vz，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的
		//	重力单位向量。
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;

	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;

	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy) * halfT;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx) * halfT;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx) * halfT;

	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	AngleX = asin(2*(q0*q2 - q1*q3 )) * 57.2957795f; // 俯仰   换算成度
	AngleY = asin(2*(q0*q1 + q2*q3 )) * 57.2957795f; // 横滚
}



//****************姿态计算*********************************************
void PWM_int (void) interrupt 	22	//PWM中断函数
{
	PWMCFG = 0;	//CBIF;	//清除中断标志

	B_8ms = 1;

//======================== 超时溢出处理 ==============================================
	PPM1_Rise_TimeOut++;	//高电平限时
	PPM2_Rise_TimeOut++;	//高电平限时
	PPM3_Rise_TimeOut++;	//高电平限时
	PPM4_Rise_TimeOut++;	//高电平限时

	if(--PPM1_Rx_TimerOut == 0)		//超过100ms收不到信号
	{
		PPM1_RxCnt = 0;			//一旦出现溢出, 则开始的n个脉冲无效
		PPM1 = 128;;			//默认中点
	}
	if(--PPM2_Rx_TimerOut == 0)		//超过100ms收不到信号
	{
		PPM2_RxCnt = 0;			//一旦出现溢出, 则开始的n个脉冲无效
		PPM2 = 128;;			//默认中点
	}
	if(--PPM3_Rx_TimerOut == 0)		//超过200ms收不到信号
	{
		PPM3_RxCnt = 0;			//一旦出现溢出, 则开始的n个脉冲无效
	}
	if(--PPM4_Rx_TimerOut == 0)		//超过100ms收不到信号
	{
		PPM4_RxCnt = 0;			//一旦出现溢出, 则开始的n个脉冲无效
		PPM4 = 128;				//默认中点
	}
//======================================================================

	if(++YM_LostCnt >= 250)		//失联2秒后
	{
		YM_LostCnt = 200;		//重复0.4秒，失控保护
		if(PPM3 > 80)	PPM3--;
		else if(++Lost16S >= 40)
		{
			Lost16S = 250;
			PPM3 = 0;
			B_Start = 0;
		}
	}
	if(YM_LostCnt  >= 25)	//失联200ms
	{
		PPM1 = 128;
		PPM2 = 128;		//俯仰 横滚 航向均归0
		PPM4 = 128;
	}

	FRX = PPM1;
	FRY = PPM2;
	YM  = PPM3;	//油门
	FRZ = PPM4;
	

//********************************************************************************************
	Read_MPU6050(tp);	//680us

	Angle_ax = ((float)(((int *)&tp)[0])) / 8192.0;	//加速度处理	结果单位是 +- g
	Angle_ay = ((float)(((int *)&tp)[1])) / 8192.0;	//转换关系	8192 LSB/g, 1g对应读数8192
	Angle_az = ((float)(((int *)&tp)[2])) / 8192.0;	//加速度量程 +-4g/S
	Last_Angle_gx = Angle_gx;		//储存上一次角速度数据
	Last_Angle_gy = Angle_gy;
	Angle_gx = ((float)(((int *)&tp)[4] - g_x)) / 65.5;	//陀螺仪处理	结果单位是 +-度
	Angle_gy = ((float)(((int *)&tp)[5] - g_y)) / 65.5;	//陀螺仪量程 +-500度/S, 1度/秒 对应读数 65.536
	Angle_gz = ((float)(((int *)&tp)[6] - g_z)) / 65.5;	//转换关系65.5 LSB/度

	IMUupdate(Angle_gx*0.0174533f, Angle_gy*0.0174533f, Angle_gz*0.0174533f, Angle_ax,Angle_ay,Angle_az);

//**********************************X轴指向************************************************
	Ax  = AngleX - a_x - ((float)FRX - 128) / 4.0;		//角度控制量加载至角度

	if(YM > 35)	ERRORX_Out += Ax,	ERRORX_Out += Ax,	ERRORX_Out += Ax;	//外环积分(油门小于某个值时不积分)
	else		ERRORX_Out = 0; //油门小于定值时清除积分值
		 if(ERRORX_Out >  1500)	ERRORX_Out =  1500;
	else if(ERRORX_Out < -1500)	ERRORX_Out = -1500;	//积分限幅

	Out_PID_X = Ax*Out_XP + ERRORX_Out*Out_XI + (Ax-Last_Ax)*Out_XD;	//外环PI
	Last_Ax = Ax;
	
	if(YM > 35)	ERRORX_In += (Angle_gy - Out_PID_X);	//内环积分(油门小于某个值时不积分)
	else		ERRORX_In = 0;	//油门小于定值时清除积分值
		 if(ERRORX_In >  500)	ERRORX_In =  500;
	else if(ERRORX_In < -500)	ERRORX_In = -500;	//积分限幅

	PID_x = (Angle_gy + Out_PID_X) * In_XP + ERRORX_In * In_XI + (Angle_gy - Last_Angle_gy) * In_XD;	//内环PID
	if(PID_x >  500)	PID_x =  500;	//输出量限幅
	if(PID_x < -500)	PID_x = -500;

//**************Y轴指向**************************************************
	Ay  = AngleY - a_y + ((float)FRY - 128) / 4.0;		//角度控制量加载至角度
	
	if(YM > 35)	ERRORY_Out += Ay,	ERRORY_Out += Ay,	ERRORY_Out += Ay;	//外环积分(油门小于某个值时不积分)
	else		ERRORY_Out = 0; //油门小于定值时清除积分值
		 if(ERRORY_Out >  1500)	ERRORY_Out =  1500;
	else if(ERRORY_Out < -1500)	ERRORY_Out = -1500;	//积分限幅
	
	Out_PID_Y = Ay * Out_YP + ERRORY_Out * Out_YI + (Ay-Last_Ay)*Out_YD;	//外环PID
	Last_Ay = Ay;

	if(YM > 35)	ERRORY_In += (Angle_gx - Out_PID_Y);  //内环积分(油门小于某个值时不积分)
	else		ERRORY_In = 0; //油门小于定值时清除积分值
		 if(ERRORY_In >  500)	ERRORY_In =  500;
	else if(ERRORY_In < -500)	ERRORY_In = -500;	//积分限幅
	
	PID_y = (Angle_gx + Out_PID_Y) * In_YP + ERRORY_In * In_YI + (Angle_gx - Last_Angle_gx) * In_YD;	//内环PID
	
	if(PID_y > 500)	PID_y =  500;	//输出量限幅
	if(PID_y <-500)	PID_y = -500;

//**************Z轴指向(Z轴随便啦，自旋控制没必要上串级PID)*****************************	
	Az = Angle_gz - ((float)FRZ - 128);
	
	if(YM > 35)	Z_integral += Az;	//Z轴积分
	else		Z_integral = 0;		//油门小于40积分清零
		 if(Z_integral >  500.0f)	Z_integral =  500.0f;	//积分限幅
	else if(Z_integral < -500.0f)	Z_integral = -500.0f;	//积分限幅

	PID_z = Az * ZP + Z_integral * ZI + (Az - Last_Az) * ZD;
	Last_Az = Az;
	if(PID_z >  200)	PID_z =  200;	//输出量限幅
	if(PID_z < -200)	PID_z = -200;

	speed0 = (int)(  PID_x + PID_y + PID_z);	//M1改为逆时针
	speed1 = (int)(  PID_x - PID_y - PID_z);
	speed2 = (int)( -PID_x - PID_y + PID_z);
	speed3 = (int)( -PID_x + PID_y - PID_z);

//**************将速度参数加载至PWM模块*************************************************	
	
	if(YM < 10)	PWM0 = 1000, PWM1 = 1000, PWM2 = 1000, PWM3 = 1000;
	else if(YM < 35)	PWM0 = 860, PWM1 = 860, PWM2 = 860, PWM3 = 860;
	else
	{
		int_tmp = 1000 - (int)YM * 4;

		PWM0 = int_tmp - speed0;

			 if(PWM0 > 1000)	PWM0 = 1000;    //速度参数控制，防止超过PWM参数范围0-1000
		else if(PWM0 < 10)		PWM0 = 10;

		PWM1 = int_tmp - speed1;

			 if(PWM1 > 1000)	PWM1 = 1000;
		else if(PWM1 < 10)		PWM1 = 10;

		PWM2 = int_tmp - speed2;

			 if(PWM2 > 1000)	PWM2 = 1000;
		else if(PWM2 < 10)		PWM2 = 10;

		PWM3 = int_tmp - speed3;

			 if(PWM3 > 1000)	PWM3 = 1000;
		else if(PWM3 < 10)		PWM3 = 10;
	}

	SW2_tmp = P_SW2;	//保存SW2设置
	EAXSFR();	//访问XFR
	PWM0T2 = (u16)(PWM0 * 2);
	PWM1T2 = (u16)(PWM1 * 2);
	PWM2T2 = (u16)(PWM2 * 2);
	PWM3T2 = (u16)(PWM3 * 2);	
	P_SW2  = SW2_tmp;	//恢复SW2设置

}


/********************** 蜂鸣函数 ************************/
void	beep(void)	//100ms调用
{
	if(BuzzerRepeat > 0)	//蜂鸣器处理, 重复次数不为0，则蜂鸣器要发声
	{
		if((BuzzerOnCnt == 0) && (BuzzerOffCnt == 0))	//On和OFF都为0，则开始装载On和Off的时间
		{
			P_BUZZER = 1;			//允许蜂鸣
			BuzzerOnCnt  = BuzzerOnTime;	//装载on计数
			BuzzerOffCnt = BuzzerOffTime;	//装载off计数
		}
		else if(BuzzerOnCnt  > 0)	{if(--BuzzerOnCnt == 0)	P_BUZZER = 0;}	//On的时间
		else if(BuzzerOffCnt > 0)	//Off的时间
		{
			if(--BuzzerOffCnt == 0)	BuzzerRepeat--;
		}
	}
	else	P_BUZZER = 0;
}

void	SetBuzzer(u8 on,u8 off,u8 rep)	// rep: 重复次数, on: on的时间, off: off的时间
{
	BuzzerRepeat = rep;
	BuzzerOnTime  = on;
	BuzzerOffTime = off;
	if(BuzzerOnTime  == 0)	BuzzerOnTime  = 1;
	if(BuzzerOffTime == 0)	BuzzerOffTime = 1;
	if(BuzzerRepeat == 1)	BuzzerOffTime = 1;
	BuzzerOnCnt = 0,	BuzzerOffCnt = 0;
}

// ===================== 自动校准序列 =====================
void	AutoCal(void)
{
	if(PPM3 < 40)	//停止时才允许校准
	{
		if(Cal_Setp == 1)	//进入校准序列
		{
			x_sum = 0;	y_sum = 0;	z_sum = 0;
			Cal_cnt  = 0;
			Cal_Setp = 2;
		}
		else if(Cal_Setp == 2)	//对陀螺仪累加
		{
			x_sum += ((int *)&tp)[4];  //读取陀螺仪数据
			y_sum += ((int *)&tp)[5];
			z_sum += ((int *)&tp)[6];
			if(++Cal_cnt >= 64)
			{
				g_x = x_sum / 64;
				g_y = y_sum / 64;
				g_z = z_sum / 64;
				float_x_sum = 0;	float_y_sum = 0;
				Cal_cnt  = 0;
				Cal_Setp = 3;
			}
		}
		else if(Cal_Setp == 3)	//对X Y角度累加
		{
			float_x_sum += AngleX;
			float_y_sum += AngleY;
			if(++Cal_cnt >= 64)
			{
				Cal_cnt  = 0;
				Cal_Setp = 0;
				a_x = float_x_sum / 64.0;
				a_y = float_y_sum / 64.0;
				IAP_Gyro();
				SetBuzzer(5,1,1);
			}
		}
	}
	else
	{
		Cal_Setp = 0;
		Cal_cnt  = 0;
	}
}

// ===================== 主函数 =====================
void main(void)
{

	//所有I/O口全设为准双向，弱上拉模式
	P0M0=0x00;	P0M1=0x00;
	P1M0=0x00;	P1M1=0x00;
	P2M0=0x00;	P2M1=0x00;
	P3M0=0x00;	P3M1=0x00;
	P4M0=0x00;	P4M1=0x00;
	P5M0=0x00;	P5M1=0x00;
	P6M0=0x00;	P6M1=0x00;
	P7M0=0x00;	P7M1=0x00;

	PPM1 = 128;
	PPM2 = 128;
	PPM3 = 0;
	PPM4 = 128;

	PWMGO();

	P_Light  = 0;
	P_BUZZER = 0;
	P5n_push_pull(0x30);

	adc_init();    //启动A/D
	
	PCA_config();

	delay_ms(100);
	IAPRead();		//读取陀螺仪静差
	InitMPU6050();	//初始化MPU-6050
	delay_ms(100);

	PWMCR =  0xc0;//ECBI;	//允许PWM计数器归零中断
	EA = 1;	//允许总中断
	
	cnt_start = 0;
	while(cnt_start < 25)	//等待油门最小	20ms * 25 = 500ms
	{
		if(B_PPM3_OK)	//油门
		{
			B_PPM3_OK = 0;
			if(PPM3_Cap <= 1200)	cnt_start++;
		}
		delay_ms(1);
	}
	P_Light  = 0;
	
	cnt_start = 0;

	SetBuzzer(5,1,1);
	

//==============================================
	UART1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
	PrintString1("STC15W4K系列大四轴飞控程序!\r\n");	//SUART1发送一个字符串
//==============================================

	B_Start = 0;	//上电禁止运行

	while(1)
	{
		if(B_PPM1_OK)	//左右(横滚)
		{
			B_PPM1_OK = 0;
				 if(PPM1_Cap < 1120)	PPM1_Cap = 1120;
			else if(PPM1_Cap > 1880)	PPM1_Cap = 1880;
			PPM2 = (u8)((PPM1_Cap-1116)/3);	//转为0~255, 中间值为128
		}
		
		if(B_PPM2_OK)	//前后(俯仰)
		{
			B_PPM2_OK = 0;
				 if(PPM2_Cap < 1120)	PPM2_Cap = 1120;
			else if(PPM2_Cap > 1880)	PPM2_Cap = 1880;
			PPM1 = (u8)((PPM2_Cap-1116)/3);	//转为0~255, 中间值为128
		}
		
		if(B_PPM4_OK)	//航向
		{
			B_PPM4_OK = 0;
				 if(PPM4_Cap < 1056)	PPM4_Cap = 1056;
				 if(PPM4_Cap > 1940)	PPM4_Cap = 1940;
				 if(PPM4_Cap < 1440)	PPM4_Cap = PPM4_Cap + 60;
			else if(PPM4_Cap > 1560)	PPM4_Cap = PPM4_Cap - 60;
			else	PPM4_Cap = 1500;
			PPM4 = (u8)((PPM4_Cap-1116)/3);	//转为0~255, 中间值为128
		}
		
		if(B_PPM3_OK)	//油门
		{
			B_PPM3_OK = 0;
			if(PPM3_Cap < 1000)	PPM3_Cap = 1000;
			if(PPM3_Cap > 1900)	PPM3_Cap = 1900;

			if(B_Start)		//正在运行时,
			{
				PPM3 = (u8)((PPM3_Cap-1000)/4);	//转为0~255,	实际8~225
				if(PPM3 < 32)	PPM3 = 32;
				
				if((PPM1 < 50) && (PPM2 < 50) && (PPM3_Cap < 1120) && (PPM4 > 200))	//下外八, 禁止
				{
					if(++cnt_start >= 50)	//1秒
					{
						cnt_start = 0;
						B_Start = 0;
						SetBuzzer(1,1,2);
					}
				}
				else	cnt_start = 0;
			}
			else	//禁止运行时, 等待内八开启
			{
				PPM3 = 0;
				if((PPM1 < 50) && (PPM2 > 200) && (PPM3_Cap < 1120) && (PPM4 < 50))	//下内八, 启动
				{
					if(++cnt_start >= 50)	//1秒
					{
						cnt_start = 0;
						B_Start = 1;
						SetBuzzer(5,1,1);
					}
				}
				else if((PPM1 > 200) && (PPM2 > 200) && (PPM3_Cap > 1850) && (PPM4 < 50))	//上内八, 水平校准
				{
					if(++cnt_start >= 50)	//1秒
					{
						cnt_start = 0;
						SetBuzzer(2,1,1);
						Cal_Setp = 1;
					}
				}
				else if((PPM1 > 200) && (PPM2 < 50) && (PPM3_Cap > 1850) && (PPM4 > 200))	//上外八, 取消水平校准
				{
					if(++cnt_start >= 50)	//1秒
					{
						cnt_start = 0;
						g_x = 0;
						g_y = 0;
						g_z = 0;
						a_x = 0;
						a_y = 0;
						IAP_Gyro();
						SetBuzzer(1,1,2);
					}
				}
				else	cnt_start = 0;
			}
		}


		if(B_8ms)		//8ms到
		{
			B_8ms = 0;
			
			if(Cal_Setp != 0)	AutoCal();	//是否执行自动校准序列
			AD();		// 读ADC计算电压

			if(++cnt_100ms >= 12)	cnt_100ms = 0,	beep();	//100ms处理一次蜂鸣器

			B = cnt_ms;
			++cnt_ms;
			B = (B ^ cnt_ms) & cnt_ms;

			if(B2)		//64ms
			{
				if(!B_BAT_LOW && (YM_LostCnt < 120))	//电压足, 信号正常
				{
					if(!B_Start)	P_Light = 0;	// 空闲时, 则慢闪(每2048ms亮64ms)
					else 			P_Light = 1;	// 启动后, 灯常亮
				}
			}
			else if(B4)		//256ms
			{
				if(B_BAT_LOW || (YM_LostCnt >= 120))	P_Light = ~P_Light;		//电压低, 或无信号, 航灯闪烁 2HZ
			}
			else if(B6)		//1024ms
			{
				if(Battery < 1090)	B_BAT_LOW = 1;	else if(Battery > 1110)	B_BAT_LOW = 0;	//<10.90V电压低, >11.10V电压够
				
				if(B_BAT_LOW)	SetBuzzer(1,1,2);	//电压低
				
				if(B_rtn_ADC0)	Return_Message();	//请求返回ADC0数据

				if(!B_BAT_LOW && (YM_LostCnt < 120))	P_Light = 1;	//遥控信号正常,	电压正常时
			}
			else if(B7)		//2048ms
			{
				if(!B_BAT_LOW && (YM_LostCnt >= 120))	SetBuzzer(1,1,3);	//电压正常时 遥控信号丢失, 每两秒短鸣3次,
			}
		}


		if(UART1_cmd != 0)
		{
			if(UART1_cmd == 'a')		//PC发送a，飞控返回一些参数
			{
				B_rtn_ADC0 = ~B_rtn_ADC0;
			}
			UART1_cmd = 0;
		}
		
		
		if((TX1_Read != TX1_Write) && (!B_TX1_Busy))	//有数据要发送, 并且发送空闲
		{
			SBUF = TX1_Buffer[TX1_Read];
			B_TX1_Busy = 1;
			if(++TX1_Read >= TX1_LENGTH)	TX1_Read = 0;
		}

	}
}

//=========================================================

void	Return_Message(void)
{
	TX1_write2buff('V');
	TX1_write2buff('=');
	TX1_write2buff(Battery/1000 + '0');
	TX1_write2buff((Battery%1000)/100 + '0');
	TX1_write2buff('.');
	TX1_write2buff((Battery%100)/10 + '0');
	TX1_write2buff(Battery%10 + '0');
	TX1_write2buff(' ');
	TX1_write2buff(' ');

	PrintString1("AngleX=");
	TX1_int_value((int)(AngleX * 10));

	PrintString1("AngleY=");
	TX1_int_value((int)(AngleY * 10));

	PrintString1("AngleZ=");
	TX1_int_value((int)(Angle_gz * 10));

	PrintString1("a_x=");
	TX1_int_value(a_x * 10);
	PrintString1("a_y=");
	TX1_int_value(a_y * 10);
	PrintString1("g_z=");
	TX1_int_value(g_z * 10);

	TX1_cnt = 0;
	TX1_write2buff(0x0d);
	TX1_write2buff(0x0a);
}


void  delay_ms(u8 ms)
{
     u16 i;
	 do
	 {
	 	i = MAIN_Fosc / 13000;
		while(--i)	;   //13T per loop
     }while(--ms);
}


void	TX1_int_value(int i)
{
	if(i < 0)	TX1_write2buff('-'),	i = 0 - i;
	else		TX1_write2buff(' ');
	TX1_write2buff(i / 1000 + '0');
	TX1_write2buff((i % 1000) / 100 + '0');
	TX1_write2buff((i % 100) / 10 + '0');
	TX1_write2buff('.');
	TX1_write2buff(i % 10 + '0');
	TX1_write2buff(' ');
	TX1_write2buff(' ');
}

/*************** 装载串口1发送缓冲 *******************************/
void TX1_write2buff(u8 dat)	//写入发送缓冲，指针+1
{
	TX1_Buffer[TX1_Write] = dat;
	if(++TX1_Write >= TX1_LENGTH)	TX1_Write = 0;
}


//========================================================================
// 函数: void PrintString1(u8 *puts)
// 描述: 串口1发送字符串函数。
// 参数: puts:  字符串指针.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void PrintString1(u8 *puts)	//发送一个字符串
{
	for (; *puts != 0;	puts++)   TX1_write2buff(*puts);	//遇到停止符0结束
}

//========================================================================
// 函数: SetTimer2Baudrate(u16 dat)
// 描述: 设置Timer2做波特率发生器。
// 参数: dat: Timer2的重装值.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================

void	SetTimer2Baudrate(u16 dat)	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
{
	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	TH2 = dat / 256;
	TL2 = dat % 256;
	IE2  &= ~(1<<2);	//禁止中断
	AUXR |=  (1<<4);	//Timer run enable
}


//========================================================================
// 函数: void	UART1_config(u8 brt)
// 描述: UART1初始化函数。
// 参数: brt: 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void	UART1_config(void)
{
	/*********** 波特率使用定时器2 *****************/
	AUXR |= 0x01;		//S1 BRT Use Timer2;
	SetTimer2Baudrate(65536UL - (MAIN_Fosc / 4) / Baudrate1);

	/*********** 波特率使用定时器1 *****************/
/*	TR1 = 0;
	AUXR &= ~0x01;		//S1 BRT Use Timer1;
	AUXR |=  (1<<6);	//Timer1 set as 1T mode
	TMOD &= ~(1<<6);	//Timer1 set As Timer
	TMOD &= ~0x30;		//Timer1_16bitAutoReload;
	TH1 = (u8)((65536UL - (MAIN_Fosc / 4) / Baudrate1) / 256);
	TL1 = (u8)((65536UL - (MAIN_Fosc / 4) / Baudrate1) % 256);
	ET1 = 0;	//禁止中断
	INT_CLKO &= ~0x02;	//不输出时钟
	TR1  = 1;
*/	//========================================================================

	SCON = (SCON & 0x3f) | 0x40;	//UART1模式, 0x00: 同步移位输出, 0x40: 8位数据,可变波特率, 0x80: 9位数据,固定波特率, 0xc0: 9位数据,可变波特率
	PS  = 1;	//高优先级中断
	ES  = 1;	//允许中断
	REN = 1;	//允许接收
	P_SW1 &= 0x3f;
	P_SW1 |= 0x00;		//UART1 switch to, 0x00: P3.0 P3.1, 0x40: P3.6 P3.7, 0x80: P1.6 P1.7 (必须使用内部时钟)
//	PCON2 |=  (1<<4);	//内部短路RXD与TXD, 做中继, ENABLE,DISABLE

	B_TX1_Busy = 0;
	TX1_Read   = 0;
	TX1_Write  = 0;
	UART1_cmd  = 0;
	TX1_cnt    = 0;
}


//========================================================================
// 函数: void UART1_int (void) interrupt UART1_VECTOR
// 描述: UART1中断函数。
// 参数: nine.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void UART1_int (void) interrupt 4
{
	if(RI)
	{
		RI = 0;
		UART1_cmd = SBUF;
	}

	if(TI)
	{
		TI = 0;
		B_TX1_Busy = 0;
	}
}



void	PCA_config(void)
{
	PPM1_Rise_TimeOut = 0;
	PPM2_Rise_TimeOut = 0;
	PPM3_Rise_TimeOut = 0;
	PPM4_Rise_TimeOut = 0;

	CR = 0;
	CH = 0;
	CL = 0;
	AUXR1 = (AUXR1 & ~(3<<4)) | PCA_P12_P17_P16_P15_P14;	//切换IO口
	CMOD  = (CMOD  & ~(7<<1)) | PCA_Clock_12T;				//选择时钟源  STC8F8K D版本
//	CMOD  = (CMOD  & ~1) | 1;								//ECF
	PPCA = 1;	//高优先级中断

	CCAPM0     = PCA_Mode_Capture | PCA_Rise_Active | PCA_Fall_Active | ENABLE;	//工作模式, 中断模式
	PCA_PWM0   = PCA_PWM_8bit;	//PWM宽度
//	CCAP0L = (u8)CCAP0_tmp;			//将影射寄存器写入捕获寄存器，先写CCAPnL
//	CCAP0H = (u8)(CCAP0_tmp >> 8);	//后写CCAPnH

	CCAPM1     = PCA_Mode_Capture | PCA_Rise_Active | PCA_Fall_Active | ENABLE;	//工作模式, 中断模式
	PCA_PWM1   = PCA_PWM_8bit;	//PWM宽度
//	CCAP1L = (u8)CCAP1_tmp;			//将影射寄存器写入捕获寄存器，先写CCAPnL
//	CCAP1H = (u8)(CCAP1_tmp >> 8);	//后写CCAPnH

	CCAPM2     = PCA_Mode_Capture | PCA_Rise_Active | PCA_Fall_Active | ENABLE;	//工作模式, 中断模式
	PCA_PWM2   = PCA_PWM_8bit;	//PWM宽度
//	CCAP2L = (u8)CCAP2_tmp;			//将影射寄存器写入捕获寄存器，先写CCAPnL
//	CCAP2H = (u8)(CCAP2_tmp >> 8);	//后写CCAPnH

	CCAPM3     = PCA_Mode_Capture | PCA_Rise_Active | PCA_Fall_Active | ENABLE;	//工作模式, 中断模式
	PCA_PWM3   = PCA_PWM_8bit;	//PWM宽度
//	CCAP3L = (u8)CCAP3_tmp;			//将影射寄存器写入捕获寄存器，先写CCAPnL
//	CCAP3H = (u8)(CCAP3_tmp >> 8);	//后写CCAPnH

	CR = 1;
}


//========================================================================
// 函数: void	PCA_Handler (void) interrupt PCA_VECTOR
// 描述: PCA中断处理程序.
// 参数: None
// 返回: none.
// 版本: V1.0, 2012-11-22
//========================================================================
void	PCA_Handler (void) interrupt PCA_VECTOR
{
	if(CCF0)		//PCA模块0中断
	{
		CCF0 = 0;		//清PCA模块0中断标志
		if(P17)	//上升沿
		{
			CCAP0_RiseTime = ((u16)CCAP0H << 8) + CCAP0L;	//读CCAP0
			PPM1_Rise_TimeOut = 1;	//收到上升沿, 高电平限时
		}
		else	//下降沿
		{
			CCAP_FallTime = ((u16)CCAP0H << 8) + CCAP0L;	//读CCAP0
			if((PPM1_Rise_TimeOut != 0) && (PPM1_Rise_TimeOut < 3))	//收到过上升沿, 高电平也没有溢出
			{
				CCAP_FallTime = (CCAP_FallTime - CCAP0_RiseTime) >> 1;	//为了好处理, 转成单位为us
				if((CCAP_FallTime >= 800) && (CCAP_FallTime <= 2500))
				{
					if(++PPM1_RxCnt >= 5)	PPM1_RxCnt = 5;		//连续接收到5个脉冲
					if(PPM1_RxCnt == 5)
					{
						if(!B_PPM1_OK)
						{
							PPM1_Cap = CCAP_FallTime;
							B_PPM1_OK = 1;		//标志收到一个脉冲
							PPM1_Rx_TimerOut = 12;	//限时收不到脉冲
						}
					}
				}
			}
			PPM1_Rise_TimeOut = 0;
		}
	}

	if(CCF1)	//PCA模块1中断
	{
		CCF1 = 0;		//清PCA模块1中断标志
		if(P16)	//上升沿
		{
			CCAP1_RiseTime = ((u16)CCAP1H << 8) + CCAP1L;	//读CCAP1
			PPM2_Rise_TimeOut = 1;	//收到上升沿, 高电平限时
		}
		else	//下降沿
		{
			CCAP_FallTime = ((u16)CCAP1H << 8) + CCAP1L;	//读CCAP1
			if((PPM2_Rise_TimeOut != 0) && (PPM2_Rise_TimeOut < 3))	//收到过上升沿, 高电平也没有溢出
			{
				CCAP_FallTime = (CCAP_FallTime - CCAP1_RiseTime) >> 1;	//为了好处理, 转成单位为us
				if((CCAP_FallTime >= 800) && (CCAP_FallTime <= 2500))
				{
					if(++PPM2_RxCnt >= 5)	PPM2_RxCnt = 5;
					if(PPM2_RxCnt == 5)
					{
						if(!B_PPM2_OK)
						{
							PPM2_Cap = CCAP_FallTime;
							B_PPM2_OK = 1;		//标志收到一个脉冲
							PPM2_Rx_TimerOut = 12;	//限时收不到脉冲
						}
					}
				}
			}
			PPM2_Rise_TimeOut = 0;
		}
	}

	if(CCF2)	//PCA模块2中断
	{
		CCF2 = 0;		//清PCA模块1中断标志
		if(P15)	//上升沿
		{
			CCAP2_RiseTime = ((u16)CCAP2H << 8) + CCAP2L;	//读CCAP2
			PPM3_Rise_TimeOut = 1;	//收到上升沿, 高电平限时
		}
		else	//下降沿
		{
			CCAP_FallTime = ((u16)CCAP2H << 8) + CCAP2L;	//读CCAP2
			if((PPM3_Rise_TimeOut != 0) && (PPM3_Rise_TimeOut < 3))	//收到过上升沿, 高电平也没有溢出
			{
				CCAP_FallTime = (CCAP_FallTime - CCAP2_RiseTime) >> 1;	//为了好处理, 转成单位为us
				if((CCAP_FallTime >= 800) && (CCAP_FallTime <= 2500))
				{
					if(++PPM3_RxCnt >= 5)	PPM3_RxCnt = 5;
					if(PPM3_RxCnt == 5)
					{
						if(!B_PPM3_OK)
						{
							PPM3_Cap = CCAP_FallTime;
							B_PPM3_OK = 1;		//标志收到一个脉冲
							PPM3_Rx_TimerOut = 25;	//限时收不到脉冲
							YM_LostCnt = 0;
							Lost16S    = 0;
						}
					}
				}
			}
			PPM3_Rise_TimeOut = 0;
		}
	}

	if(CCF3)	//PCA模块3中断
	{
		CCF3 = 0;		//清PCA模块1中断标志
		if(P14)	//上升沿
		{
			CCAP3_RiseTime = ((u16)CCAP3H << 8) + CCAP3L;	//读CCAP3
			PPM4_Rise_TimeOut = 1;	//收到上升沿, 高电平限时
		}
		else	//下降沿
		{
			CCAP_FallTime = ((u16)CCAP3H << 8) + CCAP3L;	//读CCAP3
			if((PPM4_Rise_TimeOut != 0) && (PPM4_Rise_TimeOut < 3))	//收到过上升沿, 高电平也没有溢出
			{
				CCAP_FallTime = (CCAP_FallTime - CCAP3_RiseTime) >> 1;	//为了好处理, 转成单位为us
				if((CCAP_FallTime >= 800) && (CCAP_FallTime <= 2500))
				{
					if(++PPM4_RxCnt >= 5)	PPM4_RxCnt = 5;
					if(PPM4_RxCnt == 5)
					{
						if(!B_PPM4_OK)
						{
							PPM4_Cap = CCAP_FallTime;
							B_PPM4_OK = 1;		//标志收到一个脉冲
							PPM4_Rx_TimerOut = 12;	//限时收不到脉冲
						}
					}
				}
			}
			PPM4_Rise_TimeOut = 0;
		}
	}

//	if(CF)	//PCA溢出中断
//	{
//		CF = 0;			//清PCA溢出中断标志
//	}

}


