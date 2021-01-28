// Self-balancing robot with Line-following function //

#include <stdio.h>
#include <stdio.h>
#include "terasic_includes.h"
#include "Car.h"
#include "priv/alt_legacy_irq.h"
#include "system.h"
#include <math.h>
#include "MPU.h"
#include "kalman.h"
#include "IrRx.h"
#include "Command.h"

float encoder_lqr;
float left;
float right;

const float A_kalman[4][4] = { 0.62306648, 0.85921357, 0.00441865, 0.00553713,
 0.08679139, 0.55889584, 0.00194059, 0.00808854,
 -0.56002930, -0.45173857, 0.27036566, 0.71947306,
 0.05872239, -0.57039578, 0.25410188, 0.75356152 };
const float B_kalman[4][4] = { 0.00542480, 0.00542480, 0.37693352, -0.86937486,
 -0.00188616, -0.00188616, -0.08679139, 0.44876757,
 0.70916911, 0.70916911, 0.56002930, -0.66609073,
 -0.24697468, -0.24697468, -0.05872239, 1.78653831 };
const float C_kalman[4][4] = { 0.66520288, 0.73866618, 0.00000000, 0.00000000,
 0.07386662, 0.61037187, 0.00000000, 0.00000000,
 -16.13045327, 53.45637964, 1.00000000, 0.00000000,
 5.39792503, -19.76754552, 0.00000000, 1.00000000 };
const float D_kalman[4][4] = { 0.00000000, 0.00000000, 0.33479712, -0.73866618,
 0.00000000, 0.00000000, -0.07386662, 0.38962813,
 0.00000000, 0.00000000, 16.13045327, -53.45637964,
 0.00000000, 0.00000000, -5.39792503, 19.76754552 };


const float L_theta = -0.00000056;
const float L_psi = -32.23874313;
const float L_theta_dot = -1.00138809*0.3;
const float L_psi_dot = -2.96306334;

float intergral = 0;
float derivative = 0;
float error, turn_line, last_error=0;
float kp_line = 8;//6
float ki_line = 0.0;
float kd_line = 50;//40



float psi = 0; // Body yaw
float theta = 0; // Wheel angle
float x_est[4] = { 0,0,0,0 }; // Kalman filter state estimates
float x_prev[4] = { 0,0,0,0 }; // Kalman filter state estimates from previous time step
float u_kalman[4] = { 0,0,0,0 }; // Input signal to Kalman filter
float x_new[4] = { 0,0,0,0 }; // Updated states



float  Angle_Balance,Gyro_Balance,Gyro_Turn,distance,vol,x_angle,PI=3.1415926;
int    Movement,cnt_ng,mode=0,flag=0,ng_count=0;
bool   stop_flag=false, demo=false,env=true,cmd_ut=false,pick_up=false,flag1=true;
alt_u8 led,led0,led1,led2,led3;
alt_32 balance_pwm,velocity_pwm,turn_pwm;
alt_u32 data32,temp32=0xdeadbeef;
MPU mpu(MPU_I2C_BASE, 0x68);
Kalman 	kalman;
CCar    Car(DC_MOTOR_LEFT_BASE, DC_MOTOR_RIGHT_BASE,MOTOR_MEASURE_LEFT_BASE,MOTOR_MEASURE_RIGHT_BASE);
CIrRx	IR(IR_RX_BASE, IR_RX_IRQ, IR_RX_IRQ_INTERRUPT_CONTROLLER_ID);
void Get_Angle();
void read_power(alt_u8 ch , alt_u16 *pRegValue);
bool CommandParsing(char *pCommand, int *pCommandID, int *pParam);
void uart_write(char *string);
bool Restart_Check();
void MPU_INT_INIT();
void mpu_init_enable(bool enable);
void set_para();
int  balance(float Angle,float Gyro);
int  velocity();
void MPU_INT_ISR(void * contex, alt_u32 id);

float controller(float Angle);

void linefollow(int array)
{
	array = ~array;
	char str[8];
	int i,j;
	char led[8];
	sprintf(str, "%d",array);
	for(i=1;i<9;i++)
		led[i-1]=array>>(i-1)&1;
	float output;
	error = 0.0;
	for (i=0;i<8;i++)
		if (i >3) {
			error += int(led[7-i])*(i-3);
		}else{
			error += int(led[7-i])*(i-4);
		}

	intergral = intergral + error;
	derivative = error - last_error;
	last_error = error;
	output = (kp_line * error + ki_line * intergral + kd_line * derivative);
	turn_line = output *1;
	left = -turn_line;
	right = turn_line;
}

/**************************************************************************
Function     : Get Angle value (Kalman filter)
parameter    :
return value :
**************************************************************************/
void Get_Angle(void)
 {
 		int16_t ax, ay, az, gx, gy, gz;
 		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 		Gyro_Balance=-gy;
 		x_angle=atan2(ax,az)*180/PI;
 		gy=gy/16.4;
 		Angle_Balance=kalman.getAngle(x_angle,-gy);
 		//printf("Angle_Balance=%f\r\n",Angle_Balance);
 		Gyro_Turn=gz;
 }

 /**************************************************************************
 Function     : ADC_LTC2308 Read power
 parameter    : Channel
 return value : Power data
 **************************************************************************/
void read_power(alt_u8 ch , alt_u16 *pRegValue){
 	int  Value;
 	// set measure number for ADC convert
 	IOWR(ADC_LTC2308_0_BASE, 0x01, 1);
 	// start measure
 	IOWR(ADC_LTC2308_0_BASE, 0x00, (ch << 1) | 0x00);
 	IOWR(ADC_LTC2308_0_BASE, 0x00, (ch << 1) | 0x01);
 	IOWR(ADC_LTC2308_0_BASE, 0x00, (ch << 1) | 0x00);
 	usleep(1);
 	while ((IORD(ADC_LTC2308_0_BASE,0x00) & 0x01) == 0x00);
 	Value = IORD(ADC_LTC2308_0_BASE, 0x01);
 	//printf("Value=%d\n",Value);
 	*pRegValue= Value;
 }

 /**************************************************************************
 Function     : Bluetooth Command Parsing
 parameter    : Command ID
 return value : Command Parsing data
 **************************************************************************/
bool CommandParsing(char *pCommand, int *pCommandID, int *pParam){
 	bool bFind = false;
 	int nNum, i, j , x=0;
 	bool find_equal = false;
 	char Data[10]={0};
 	nNum = sizeof(gCommandList)/sizeof(gCommandList[0]);

 	for(i=0;i<nNum && !bFind;i++){
 		if (strncmp(pCommand, gCommandList[i].szCommand, strlen(gCommandList[i].szCommand)) == 0){
 			*pCommandID = gCommandList[i].CommandId;
 			if (gCommandList[i].bParameter){
 				//*pParam = 10; //??
 				//for(j=0;pCommand[j]!=0x0a;j++){
 				for(j=0;pCommand[j]!=0x0d;j++){
 					if(find_equal==true){
 						Data[x] = pCommand[j];
 						x++;
 					}
 					else if(pCommand[j]=='=')
 						find_equal=true;
 				}
 				*pParam=atoi(Data);
 			}
 			bFind = true;
 		} // if
 	} // for
 	return bFind;
 }

 /**************************************************************************
 Function     : Bluetooth Data Send
 parameter    : data
 return value :
 **************************************************************************/
void uart_write(char *string)
 {
 	int i;
 	alt_32 temp;
 	int len=strlen(string);
 	for(i=0;i<len;i++)
 	{
 		temp=IORD(UART_BT_BASE,0x01);
 		temp=temp>>16&0xff;
 		if(temp>0)IOWR(UART_BT_BASE,0x0,*string++);
 		else i--;
 	}
 }

 /**************************************************************************
 Function     : Normal State Check and Restart Self-Balance
 parameter    :
 return value : 1:normal  0:UnNormal
 **************************************************************************/
 bool  Restart_Check()
  {
 	 if(x_angle<10&&x_angle>-10){
 		 return true;}
 	 else
 		 return false;
  }

 /**************************************************************************
 Function     : MPU6500 interrupt (MPU6500 INT pin trigger)
 parameter    :
 return value :
 **************************************************************************/
 void MPU_INT_INIT(void)
  {
  	 IOWR_ALTERA_AVALON_PIO_EDGE_CAP(MPU_INT_BASE,0x00);
  #ifdef ALT_ENHANCED_INTERRUPT_API_PRESENT
       if ((alt_ic_isr_register(MPU_INT_IRQ_INTERRUPT_CONTROLLER_ID,
                                     MPU_INT_IRQ,
                                        MPU_INT_ISR,
                                 NULL,
                                 NULL
                                 )!= 0))
  #else
      if((alt_irq_register(MPU_INT_IRQ,NULL, MPU_INT_ISR  )!= 0))
  #endif
      	{
  			  printf("register irt failed\r\n");
  		  }
  		IOWR_ALTERA_AVALON_PIO_IRQ_MASK(MPU_INT_BASE,0x01);
  }

 /**************************************************************************
  Function     : MPU6500 Interrupt initialization
  parameter    :
  return value :
 **************************************************************************/
 void mpu_init_enable(bool enable)
  {
  	if(enable==TRUE)
  		IOWR_ALTERA_AVALON_PIO_IRQ_MASK(MPU_INT_BASE,0x01);
  	else
  		IOWR_ALTERA_AVALON_PIO_IRQ_MASK(MPU_INT_BASE,0x00);
  }

/**************************************************************************
Function     : Run the specified action (Demo Mode)
parameter    :
return value :
**************************************************************************/
void set_para(){
	static int count;
	static int count_times=0;
	//set Movement
 	if(Car.driver_direction&CAR_DIRECTION_FORWARD){
 		if((distance<25.0)&(mode==0x02)&(flag==0x02)) Movement=-10;
 		else if((distance<25.0)&(cmd_ut)&(flag==0x01)) Movement=-10;
 		else if(Angle_Balance<-10.0) Movement=-5;
 		else Movement=-10;}//-48
 	else if(Car.driver_direction&CAR_DIRECTION_BACKWARD){
 		if(Angle_Balance>12.0) Movement=5;
 		else Movement=45;}//45
 	else
 		 Movement=0;//0
}

/**************************************************************************
Function     : Upright Closed-loop Control (PD)
parameter    : The angular velocity value
return value : Upright Closed-loop Control PWM
**************************************************************************/
int balance(float Angle,float Gyro)
 {
	 float Bias,kp=11.0,kd=0.02;//0.0 0.02
 	 int balance;
 	 Bias=Angle-1;
 	 balance=kp*Bias+Gyro*kd;
 	// printf("balance=%d\n",balance);
 	 return balance;
 }
float controller(float Angle) {
	psi = Angle;
	theta = encoder_lqr;
	u_kalman[0] = balance_pwm;
	u_kalman[1] = balance_pwm;
	u_kalman[2] = theta;
	u_kalman[3] = psi;
	for (int i = 0; i < 4; i++) {
		x_new[i] = 0;
		x_est[i] = 0;
	}
		for (int r = 0; r < 4; r++)
			for (int k = 0; k < 2; k++)
				x_est[r] += x_prev[k] * C_kalman[r][k];
	x_est[2] += x_prev[2];
	x_est[3] += x_prev[3];
	for (int r = 0; r < 4; r++)
		for (int k = 2; k < 4; k++)
			x_est[r] += u_kalman[k] * D_kalman[r][k];
	for (int r = 0; r < 4; r++)
		for (int k = 0; k < 4; k++)
			x_new[r] += x_prev[k] * A_kalman[r][k] + u_kalman[k] * B_kalman[r][k];
	for (int i = 0; i < 4; i++)
		x_prev[i] = x_new[i];

	return -
		(L_theta*x_est[0] + L_psi * x_est[1] + L_theta_dot * x_est[2] + L_psi_dot * x_est[3]);
}
/**************************************************************************
Function     : Speed Closed-loop Control (PI)
parameter    : The encoder values of the wheelthe
return value : Speed Closed-loop Control PWM
**************************************************************************/
int velocity(void)
 {
     static float Velocity,Encoder_Least,Encoder;
 	 static float Encoder_Integral;
 	 float kp=2.2,ki=0.05;
  	 Encoder_Least =(-Car.Measure_L+Car.Measure_R)-0;
  	 Encoder *= 0.8;
  	 Encoder += Encoder_Least*0.2;
  	 Encoder_Integral +=Encoder;
  	 Encoder_Integral=Encoder_Integral-Movement;
  	 if(Encoder_Integral>8000)  	Encoder_Integral=8000;
  	 if(Encoder_Integral<-8000)	Encoder_Integral=-8000;
  	 if(stop_flag)Encoder_Integral=0;
  	 Velocity=Encoder*kp+Encoder_Integral*ki;
	 encoder_lqr = Encoder_Integral;
 	 return Velocity;
 }

/**************************************************************************
Function     : The interrupt function for 10ms
parameter    :
return value :
**************************************************************************/
#ifdef ALT_ENHANCED_INTERRUPT_API_PRESENT //nios2 91 edition or later
void MPU_INT_ISR(void *contex)
#else //before nios2 91 edition
void MPU_INT_ISR(void * contex, alt_u32 id)
#endif
{
	alt_32 static count;
  	int cnt;
	if(!IORD_ALTERA_AVALON_PIO_EDGE_CAP(MPU_INT_BASE)){
		return;
	}else
	{
		IOWR_ALTERA_AVALON_PIO_EDGE_CAP(MPU_INT_BASE,0x00);
		IOWR_ALTERA_AVALON_PIO_IRQ_MASK(MPU_INT_BASE,0x00);
		Get_Angle();
		Car.meaure_speed();
		Car.Set_TurnFORWARD();
		int array=~IORD(PIO_0_BASE,0X00);
		//printf("%d\n",array);
		IOWR_ALTERA_AVALON_PIO_DATA(LED_BASE,array);
		linefollow(array);
		balance_pwm=0.5*balance(Angle_Balance,Gyro_Balance) + 0.5*controller(Angle_Balance);
		set_para();
		velocity_pwm=velocity();
		Car.SetSpeed(-balance_pwm-velocity_pwm+left,-balance_pwm-velocity_pwm+right);//update PWM

	}
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(MPU_INT_BASE,0x01);
}


/**************************************************************************
Function     : The main function
parameter    : data
return value :
**************************************************************************/
int main()
{
	alt_u32 data;
	alt_u32 temp;
  	alt_u8  number;
  	alt_u16 Data16;
  	char szADC_Data[10]={0};
  	int i=0;
  	int Command_EPS32,Command_IR,Param;
  	char szData[10];
  	printf("Hello BAL-Car\r\n");
  	mpu.initialize();
  	IR.Enable();
  	Car.Stop();
  	Car.Start();
  	Car.SetSpeed(0,0);
  	led1=0x00;
  	MPU_INT_INIT();


	while(1){
		read_power(0x01 , &Data16);
        vol=(float)Data16*18.0/4.7/1000.0;
	    if(vol<10.5)
			led2=0x01;
	    else
		    led2=0x00;
	led=((led0&0x01)<<7)|((led1&0x03)<<5)|((led2&0x01)<<4)|(led3&0x0f);
    }
   return 0;
}//end main
