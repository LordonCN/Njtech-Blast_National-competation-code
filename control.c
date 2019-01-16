/******************* (C) COPYRIGHT 2018.7.10  JF TEAM *****START OF FILE************/
/*!
* @file       main.c
* @brief      疾风KEA128主程序
* @author     南工电控疾风队
* @version    v8.0
* @date       2018-7-9

*/
#include "include.h"

int16 Speed_L,Speed_R;

uint8 Stop=true , ready = false, starting = false,over = false;
void vcan_sendware(uint8 *wareaddr, uint32 waresize);
int8 stop_sign_practice = 0;
/* ______________特殊元素更改参数________________*/
int16     SetSpeedMax_temp;
float    Speed_Kp_temp;
float    Turn_Kp_temp;
float    Turn_Kd_temp;
int16     Cir_threshold_temp;
unsigned long int  Angle_Kp_temp;
unsigned long int  Angle_Kd_temp;
/* ______________特殊元素更改参数________________*/
//page1
int16     SetSpeedMax=160;
float    Speed_Kp   =63;
float    Speed_Ki   = 0.195;
float    Turn_Kp=120;
float    Turn_Kd=11;
float    AngelSpeed_Kd = 18.2;
int16     Cir_threshold =5000;        //圆环特征值  5000
//page2
unsigned long int  Angle_Kp=230;
unsigned long int  Angle_Kd=9;
unsigned long int  Anglefloat = 8200;  //直立零点  8200
int16                Stop_times=3;
//page3
int16     Podao_threshold=800;    // 坡道上限   800
int16     Cir_LR_threshold=900;   // 圆环下限   900
int16 admin_stop_time=1019;
//模糊
float  Delta_P=0;
float  Delta_D=0;
float  Fuzzy_Kp=25;
float  Fuzzy_Kd=10;
float  Fuzzy_out=0;
extern float error_d;

/*注意：发车多准备5秒，应设置为19.5秒与13.5秒
针对干簧管乱触发状况，大致估摸下跑完时间，约为14.5秒，试跑赛道应该为8.5秒 */

int16 over_stop = false;
int16 Duikang_stop = false;

int16 over_stop_sign = 0;//在没有磁铁的情况下判断是否成功开启中断
/*--------------------------------控制函数变量定义--------------------------------*/
extern int startflag=0;
extern int StandMode=0;
extern int16 Big_Cir_Is;         //确认大圆环位置UI
extern int16 Big_Cir2_Is;
extern int16 Big_Cir3_Is;
extern int16 Mid_Cir_Is;
extern int16 Mid_Cir2_Is;
extern int16 Podao_Num;

//响应较好超调小
#define GyroYfloat        65535                              //直立（Y轴）陀螺仪零点
#define GyroXfloat        65535                               //转向（X轴）陀螺仪零点

#define Anglerate        -0.007f                              //加速度计（数字转度数）系数
#define Gyrorate         0.06098f                              //陀螺仪（数字转度数）系数

/*--------------------------------角度参数始--------------------------------*/
int   GyroY=0,GyroYo=0;                               //陀螺仪（Y轴）采集值,用于直立
float Acc_X=0;                                       //加速度计采集值
float AngleOut=0;                                    //直立控制输出
float AngleCom=0;                                     //角度补偿
float Angle=0;                                        //卡尔曼融合角度
float Gyropitch=0,GyropitchOld[5];                       //Y轴角速度
float AccZ_Angle=0,AccZ_AngleOld[5];                   //加速度计融合角度
/*--------------------------------角度参数终--------------------------------*/

/*--------------------------------速度参数始--------------------------------*/
uint16_t leftcount,rightcount;                      //编码器计数值
int16_t leftspeed,rightspeed;                       //实际速度值
int16_t leftspeed_old,rightspeed_old;                       //实际速度值记录
float g_fSpeedControlOut=0;
float g_fCarSpeed=0;
int16 g_nSpeedControlPeriod=0;

int16 run_start = false;
int16 run_normal = false;//速度稳定下来
int16 run_error = false;//疯转
int16 run_normal_eva = 0;
int16 chaosu = 0;

float SetSpeed=0;                                     //设定速度
unsigned long int SpeedCodeErrorCount=0;
int16 SpeedNow=0,SpeedOld[5]={0};                       //车模当前速度及存储数组

/*--不慢队速度参数--*/
//速度类变量
float SpeedControlOutNew;
float SpeedControlOutOld;
float SpeedControlIntegral=0;
float ControlSpeed=0;

extern float denominator;
/*--------------------------------速度参数终--------------------------------*/

/*--------------------------------转向参数始--------------------------------*/
int32 GyroX=0,GyroXo=0;//陀螺仪（X轴）采集值
float Gyroroll,GyrorollOld[20];//转向角速度

static float g_fDirectionControlOutOld, fvalueNew,fvalueOld,g_fDirectionControlOutNew,g_nDirectionControlPeriod;
float  g_fDirectionControlOut=0;

/*__南师大转向参数__*/
float Direction_Kp=0,SpeedRate;
float Turn_Offset=0;

#define SpeedStep1     0.3//0.5f                       //0.9
#define SpeedStep2     0.35//0.8f                       //0.8
#define SpeedStep3     0.45//1.05f                      //0.7
#define SpeedStep4     0.65//1.25f                      //0.6
#define SpeedStep5     0.7//1.3f                       //0.5
#define SpeedStep6     0.75//1.35f                      //0.4
/*--------------------------------转向参数终--------------------------------*/

/*--------------------------------OLED调参--------------------------------*/
uint8 Debug_Index=0;
uint8 DebugC_Index1=0;
uint8 DebugC_Index2=0;
uint8 DebugC_Index3=0;
uint8 DebugC_Index4=0;
uint8 DebugC_Index5=0;
uint8 DebugC_Index6=0;

//       Sum_Cir_Num
//       Mid_Cir_Is
//       Big_Cir_Is
//       Big_Cir2_Is
//       Cir_threshold  Podao_threshold  Cir_LR_threshold

char Para_Switch1[7][16]= {"SetSpeedM","Speed_Kp\0","Speed_Ki\0","Turn_Kp\0","Turn_Kd\0","AgSpd_Kd\0","what\0"};
char Car_Set_Name[7][12]= {"Angle_Kp\0","Angle_Kd\0","Anglefloat\0","Fuzz_Kp\0","Fuzz_Kd\0","stop_time\0","what\0"};
char Para_Switch2[7][16]= {"Sum_Cir","Mid_Cir\0","Mid_Cir2\0","Big_Cir\0","Cir_LRthre\0","Cir_thres\0","Podao_thre\0"};

//char Car_Set_Name[7][12]= {"Set_Speed\0","Set_Angle\0","Acc_Offset\0","Debug_Mode\0","Light\0","Protect\0","Threshold\0"};
char Debug_Mode[5][12]= {"Normal \0","Upstand\0","Test_ad\0","ErrorMo\0","Bianma\0"}; //调试模式：正常 直立 没有速度 ，没有方向
float what = 0;

char Circle_Num[7][12]= {"Circle1\0","Circle2\0","Circle3\0","Circle4\0","Circle5\0","Circle6\0","what \0"};//OLED显示第几个圆环
float Circle1,Circle2,Circle3,Circle4,Circle5,Circle6;
char Direct_Cir1[2][12]= {" left  \0"," right\0"};//调试圆环方向：圆环方向
char Direct_Cir2[2][12]= {" left  \0"," right\0"};//调试圆环方向：圆环方向
char Direct_Cir3[2][12]= {" left  \0"," right\0"};//调试圆环方向：圆环方向
char Direct_Cir4[2][12]= {" left  \0"," right\0"};//调试圆环方向：圆环方向
char Direct_Cir5[2][12]= {" left  \0"," right\0"};//调试圆环方向：圆环方向
char Direct_Cir6[2][12]= {" left  \0"," right\0"};//调试圆环方向：圆环方向

/*--------------------------------OLED调参以上--------------------------------*/

/*********************************************************控制函数变量定义终（不得随意更改位置）*************************/
int16  OutData[6];
static int flag1ms=0;
static int flag5ms=0;
static int flag10ms=0;
static int flag20ms=0;
static int flag100ms=0;
static int flag1000ms=0;
static int flag10000ms=0;
/**************************PIT0中断控制程序*************************************/
void pit0_irq(void)
{
  PIT_Flag_Clear(PIT0);       //清中断标志位

  static int count5ms=0;
  static int count10ms=0;
  static int count20ms=0;
  static int count50ms=0;
  static int count100ms=0;
  static int count1000ms=0;
  static int count10000ms=0;

  count5ms++;
  count10ms++;
  count20ms++;
  count50ms++;
  count100ms++;
  count1000ms++;
  count10000ms++;

  switch(count5ms)
  {
  case 1:
    //直立控制
    AngleControl();
    motor();
    break;
  case 2:
    DirectionControl();
    break;
  case 3:
    SpeedControlOutput();
    break;
  case 4:
    Speedcount();            //编码器计数更新
    break;
  case 5:
    count5ms=0;
    break;
  default:
    break;
  }

  DirectionControlOutput();
  motor();

  if(count10ms==10)//周期10ms的任务
  {
    count10ms=0;      /***清除统计***/
    flag10ms=1;

  }
  if(count20ms==20)//周期20ms的任务
  {
    count20ms=0;      /***清除统计***/
    flag20ms=1;
//    SetSpeedMax=180;

  }
  if(count50ms==50)//周期100ms的任务
  {
    count50ms=0;     /***清除统计***/
    Speedcontrol();

  }
  if(count100ms==100)//周期100ms的任务
  {
    count100ms=0;     /***清除统计***/
    flag100ms=1;

    //		Speedcontrol();
  }
  if(count1000ms==1000)//周期1000ms的任务
  {
    count1000ms=0;      /***清除统计***/
    flag1000ms=1;
    led_turn(LED3);       /*****一秒闪烁一次********/

  }
  if(count10000ms==(admin_stop_time*1000))//用来停车的中断
  {
    over_stop_sign = 10000;
    count10000ms=0;      /***清除统计***/
    flag10000ms=1;
  }
}

/******************************中断状态标识************************************/

int get_flag1ms()
{
  return flag1ms;
}
void clr_flag1ms()
{
  flag1ms=0;
}

int get_flag5ms()
{
  return flag5ms;
}
void clr_flag5ms()
{
  flag5ms=0;
}

int get_flag10ms()
{
  return flag10ms;
}

void clr_flag10ms()
{
  flag10ms=0;
}

int get_flag20ms()
{
  return flag20ms;
}

void clr_flag20ms()
{
  flag20ms=0;
}

int get_flag100ms()
{
  return flag100ms;
}

void clr_flag100ms()
{
  flag100ms=0;
}

int get_flag1000ms()
{
  return flag1000ms;
}

void clr_flag1000ms()
{
  flag1000ms=0;
}

//停车专用
void clr_flag10000ms()
{
  flag10000ms=0;
}
int get_flag10000ms()
{
  return flag10000ms;
}



/*******************************************************************************/

void motor()
{


/*
500    1200   930
1000   3600   3300
1500   6100   5800
2000   8500   8300
*/

  Speed_L=(int)(AngleOut-g_fSpeedControlOut+g_fDirectionControlOut);
  Speed_R=(int)(AngleOut-g_fSpeedControlOut-g_fDirectionControlOut);

  if(2==StandMode)                   //直立+方向
  {
    Speed_L=(int)(AngleOut+g_fDirectionControlOut);
    Speed_R=(int)(AngleOut-g_fDirectionControlOut);
  }


  if(Speed_L > 6000)       Speed_L =6000;            //输出限幅
  else if(Speed_L < -6000)   Speed_L = -6000;

  if(Speed_R > 6000)  Speed_R = 6000;
  else if(Speed_R < -6000) Speed_R = -6000;


  if(Speed_L>=0)                                 //angle大于0，向前，小于0，向后
  {
    PWMSet(FTM_CH2, 0)  ;
    PWMSet(FTM_CH3,Speed_L+MOTOR_DEAD_VAL_L);    //加入死区电压(uint32_t)(Speed_L + MOTOR_DEAD_VAL_L)
  }
  else
  {
    PWMSet(FTM_CH3,0)  ;
    PWMSet(FTM_CH2,-Speed_L+MOTOR_DEAD_VAL_L);    //加入死区电压
  }

  if(Speed_R  >=0)    //angle大于0，向前，小于0，向后     右电机死区电压50
  {
    PWMSet(FTM_CH1, 0)  ;
    PWMSet(FTM_CH0,Speed_R+MOTOR_DEAD_VAL_R);    //加入死区电压
  }
  else
  {
    PWMSet(FTM_CH0, 0)  ;
    PWMSet(FTM_CH1,-Speed_R+MOTOR_DEAD_VAL_R);    //加入死区电压
  }

}

/**********************************角度控制************************************/

/*___________________卡尔曼滤波_______________________*/
float Kalman_Filter(float angle_m,float gyro_m)
{
  //以下基本无需再调
  static float x=0;          //最优角度初值
  static float p=0.000001;   //最优角度对应协方差初值
  static float Q=0.000001;
  static float R=0.35;       //old=0.35
  static float k=0;
  static int sign=0;
  if(sign==0)
  {
    sign=1;
    x=angle_m;
  }
  x=x+ gyro_m*0.00348f;
  p=p+Q;
  k=p/(p+R);
  x=x+k*(angle_m-x);
  p=(1-k)*p;
  return x;
}

//*********************************南***大学开源***************************

void AngleControl(void)
{
  GyroYo=GyroY;
  GyroY=mpu6050_read_gY()+55;//读取数字式陀螺仪的Y轴采样值

  //------------------------↓将数字值处理，防止溢出↓--------------------------
  if(GyroY<32768) GyroY+=65535;
  GyroY=GyroY-GyroYfloat;
  if((GyroY-GyroYo)>32768) GyroY-=65535;
  else if((GyroYo-GyroY)>32768) GyroY+=65535;

  Gyropitch=(float)GyroY*Gyrorate*2.5;                                  //计算角速度

  Acc_X=mpu6050_read_aX();                              /***加速度计X轴 -> C0***/

  AccZ_Angle=(Acc_X+(float)Anglefloat)*Anglerate;                   //计算加速度计得出的角度


  if(AccZ_Angle>80) AccZ_Angle=80;                           //对加速度计测量角度限幅
  else if(AccZ_Angle<-80) AccZ_Angle=-80;


  Angle=Kalman_Filter(AccZ_Angle,Gyropitch);                 //卡尔曼融合角度

  AngleOut=(float)Angle_Kp*(Angle-AngleCom)+(float)Angle_Kd*Gyropitch;     //PD控制直立


  if(AngleOut>10000)
  {
    AngleOut=10000;
  }
  else if(AngleOut<-9000)
  {
    AngleOut=-9000;   //限幅
  }

}

/**********************************速度控制************************************/

void Speedcount_init(void)     //速度检测初始化

{
  gpio_init (PTE1, GPI,0);    //检测编码器正反转
  gpio_init (PTH2, GPI,0);
  ftm_pulse_init(FTM0, FTM_PS_1, TCLK1);          //FTM2来脉冲计数。不分频，输入引脚为 TCLK1
  ftm_pulse_init(FTM1, FTM_PS_1, TCLK2);          //FTM2来脉冲计数。不分频，输入引脚为 TCLK1
}


void Speedcount(void)     //速度检测
{
  leftcount = ftm_pulse_get (FTM1);
  rightcount = ftm_pulse_get (FTM0);

  if(PTH2_IN == 1)                        //正转
  {
    leftspeed   =  leftcount;
  }
  else                                    //反转
  {
    leftspeed   = -leftcount;
  }

  if(PTE1_IN == 0)                        //正转
  {
    rightspeed   = rightcount;
  }
  else                                    //反转
  {
    rightspeed   = -rightcount;
  }

  ftm_pulse_clean(FTM1) ;             //清空计数。然后延时阶段为硬件测量脉冲数
  ftm_pulse_clean(FTM0) ;


  SpeedNow=leftspeed+rightspeed;


  SpeedOld[4]=SpeedOld[3];
  SpeedOld[3]=SpeedOld[2];
  SpeedOld[2]=SpeedOld[1];
  SpeedOld[1]=SpeedOld[0];
  SpeedOld[0]=SpeedNow;
  if((SpeedNow-SpeedOld[4])> 25) SpeedNow=SpeedOld[4];//速度防突变滤波
  if((SpeedNow-SpeedOld[4])<-25) SpeedNow=SpeedOld[4];

    /*_____________________________________疯转处理________________________________________*/
  run_normal_eva = (SpeedOld[4]+SpeedOld[3]+SpeedOld[2]+SpeedOld[1])/4;//简单求下平均速度

  if((!run_start)&& ABS(run_normal_eva-SetSpeedMax)<80)
  {
  run_start  = true;//已经正常起步
  }

  if(run_start && (SpeedNow-SetSpeedMax)>25|| Angle>=50)//如果起步后发现超速或者太低头
  {
  run_error = true;//出现疯转及危险

  chaosu = 10000;
  }
  if(run_error&&(SpeedNow<SetSpeedMax)|| Angle<50)
  {run_error = false;
  chaosu = 0;}//减速后回复正常


  g_fCarSpeed=g_fCarSpeed*0.1+0.9*SpeedNow;

  ControlSpeed=ControlSpeed*0.95+g_fCarSpeed*0.05;

}
  /******************************重庆不慢队开源速度控制********************************/
void Speedcontrol(void)
{
  static float setspeed=0;
  static float PreError[20]= {0};
  float  SpeedError,Speed_temp;
  uint8 i;
  float  SpeedFilterRatio=0.85;     //速度设定值滤波，防止速度控制变化太剧烈

  //设定速度滤波


  if(startflag && SetSpeed!=SetSpeedMax && !StandMode)      //车模起步处理 非直立模式 当前目标速度没有达到最大设定速度
  {
    if(SpeedNow<80)
    {SetSpeed+=0.032*(float)SetSpeedMax;}
    if(SpeedNow>=80)
    {SetSpeed+=0.01*(float)SetSpeedMax;}
    if(SetSpeed>SetSpeedMax)  SetSpeed=(float)SetSpeedMax;
  }
  //速度滤波，防止因为速度变化过大而车身晃动
  Speed_temp=SetSpeed;

  if(startflag && SetSpeed!=SetSpeedMax && !StandMode)    Speed_temp=Speed_temp*0.9;

  setspeed = Speed_temp*(1-SpeedFilterRatio)+setspeed*SpeedFilterRatio;

  if(!startflag)  setspeed=0; //启动的时候把速度置为零


  SpeedError=setspeed-ControlSpeed;

  //求出最近20个偏差的总和作为积分项
  SpeedControlIntegral=0;
  for(i=0; i<19; i++)
  {
    PreError[i]=PreError[i+1];
    SpeedControlIntegral+=PreError[i];
  }
  PreError[19]=SpeedError;
  SpeedControlIntegral+=PreError[19];

  //速度更新
  SpeedControlOutOld    =       SpeedControlOutNew;
  SpeedControlOutNew    =       Speed_Kp*SpeedError+Speed_Ki*SpeedControlIntegral;   //PI控制


    /*______________________坡道速度控制，缺坡道检测______________________*/
        if(run_error)
        {
        SpeedControlOutNew*=0.152;
        SpeedControlOutNew= SpeedControlOutOld*0.7+SpeedControlOutNew*0.3;
        SpeedControlOutOld=SpeedControlOutNew;
        }

  SpeedControlOutNew    =       SpeedControlOutOld*0.9+SpeedControlOutNew*0.1;

  /*-----------------------------------------南***大学开源转向参数-----------------------------------------*/

  if((SetSpeedMax!=0))
  {
    SpeedRate=((float)(g_fCarSpeed)/(float)(SetSpeedMax));
    if(SpeedRate>0.9f) //接近设定的最大速度
    {
      Direction_Kp=Turn_Kp*(1.0f-(1.0-SpeedRate)*SpeedStep1);//1-（0.05）*0.3   近似不改变
    }
    else if(SpeedRate>0.8f)
    {
      Direction_Kp=Turn_Kp*(1.0f-0.1f*SpeedStep1-(0.9f-SpeedRate)*SpeedStep2);
    }
    else if(SpeedRate>0.7f)
    {
      Direction_Kp=Turn_Kp*(1.0f-0.1f*SpeedStep1-0.1f*SpeedStep2-(0.8f-SpeedRate)*SpeedStep3);
    }
    else if(SpeedRate>0.6f)
    {
      Direction_Kp=Turn_Kp*(1.0f-0.1f*SpeedStep1-0.1f*SpeedStep2-0.1f*SpeedStep3-(0.7f-SpeedRate)*SpeedStep4);
    }
    else if(SpeedRate>0.5f)
    {
      Direction_Kp=Turn_Kp*(1.0f-0.1f*SpeedStep1-0.1f*SpeedStep2-0.1f*SpeedStep3-0.1f*SpeedStep4-(0.6f-SpeedRate)*SpeedStep5);
    }
    else
    {
      Direction_Kp=Turn_Kp*(1.0f-0.1f*SpeedStep1-0.1f*SpeedStep2-0.1f*SpeedStep3-0.1f*SpeedStep4-0.1f*SpeedStep5-(0.5f-SpeedRate)*SpeedStep6);
    }

    if(Direction_Kp<(Turn_Kp*0.5f)) Direction_Kp=Turn_Kp*0.5f; //最小0.5f
    if(Direction_Kp>Turn_Kp)        Direction_Kp=Turn_Kp;      //最大
  }

  /*-----------------------------------------更新转向参数终---------------------------------------*/


}

void SpeedControlOutput(void)
{
  float fValue;
  fValue=SpeedControlOutNew-SpeedControlOutOld;  //求速度偏差

  g_fSpeedControlOut=fValue*(++g_nSpeedControlPeriod)/20+SpeedControlOutOld;     //g_nSpeedControlPeriod  变量需传回来参考下

  if (g_fSpeedControlOut>6000)
    g_fSpeedControlOut=6000;                                //限幅
  else if(g_fSpeedControlOut<-6000)
    g_fSpeedControlOut=-6000;

  if(g_nSpeedControlPeriod==20)
  {
    g_nSpeedControlPeriod=0;
  }
}

/**********************************方向控制************************************/
/******************************************************************************/
  float fvalue,fDvalue,ERROR,fDvalue_exper1,fDvalue_exper2;
void DirectionControl(void)
{
  /*****************************************************南***大学开源转向函数*********************************/

 

  GyroXo=GyroX;
  GyroX=mpu6050_read_gZ()+4;

  //------------------------↓将数字值处理，防止溢出↓--------------------------

  if(GyroX<32768) GyroX+=65535;
  GyroX=GyroX-GyroXfloat;
  if((GyroX-GyroXo)>32768) GyroX-=65535;
  else if((GyroXo-GyroX)>32768) GyroX+=65535;

  //----------------------------------------------------------------------------
  Gyroroll=GyroX*Gyrorate;                     //需要滤波

  Gyroroll=Gyroroll*0.7+GyrorollOld[0]*0.3; //滤波 5.18

  GyrorollOld[0]=Gyroroll;                  


  g_fDirectionControlOutOld = g_fDirectionControlOutNew;

  fvalue =Get_ad()/20;       //归到-100~100

  //	Turn_Offset=fvalue*(fvalue*fvalue/1250.0+2)/10;

  Turn_Offset=fvalue*(fvalue*fvalue/625.0+4);  //三次
  /*
  将原始偏差做了一个一元三次函数的处理，保留一次项和三次项，
  这样处理有利于方向控制的平滑，
  且对于原始偏差的小偏差和大偏差响应有所区别
  */

  fvalueNew = fvalue;
  ERROR=fvalueNew - fvalueOld;                //与上一次求偏差
  ERROR = ERROR*20;
  //模糊控制

  error_d=ERROR;
  error_d = (error_d>= 32? 32:error_d);	//偏差限幅
  error_d = (error_d<=-32?-32:error_d);

  Fuzzy_out=Fuzzy( fvalue,error_d);
  Delta_P=Fuzzy_out* Fuzzy_Kp;
  Delta_D=Fuzzy_out* Fuzzy_Kd;

 // fDvalue=         (Direction_Kp+Fuzzy_Kp) * Turn_Offset + Turn_Kd * ERROR + (AngelSpeed_Kd+Fuzzy_Kd) * Gyroroll;    //2.5转向
  fDvalue_exper1 = (Direction_Kp+Fuzzy_Kp) * Turn_Offset + (Turn_Kd+Fuzzy_Kd) * ERROR + (AngelSpeed_Kd) * Gyroroll; //
 // fDvalue_exper2 = (Direction_Kp) * Turn_Offset + Turn_Kd * ERROR + (AngelSpeed_Kd) * Gyroroll;

  fvalueOld = fvalueNew;

  g_fDirectionControlOutNew=fDvalue_exper1;
 //   g_fDirectionControlOutNew=fDvalue;//输出pd控制转向值

  //       g_fDirectionControlOutNew=Turn_Out_Filter(g_fDirectionControlOutNew);              //转动输出滤波

  if (g_fDirectionControlOutNew>6000)   g_fDirectionControlOutNew=6000;                   //限幅
  else if(g_fDirectionControlOutNew<-6000)   g_fDirectionControlOutNew=-6000;
}

void DirectionControlOutput(void)
{
  float fValue;
  fValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld;

  g_fDirectionControlOut = fValue * (++g_nDirectionControlPeriod) / 5 + g_fDirectionControlOutOld;

  if(g_nDirectionControlPeriod==5)

  {
    g_nDirectionControlPeriod=0;
  }
}

float  Turn_Out_Filter(float turn_out)    //转向控制输出滤波
{
  float Turn_Out_Filtered;
  static float Pre1_Error[4];
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
//  Turn_Out_Filtered=Pre1_Error[0]*0.4+Pre1_Error[1]*0.3+Pre1_Error[2]*0.2+Pre1_Error[3]*0.1;
 Turn_Out_Filtered=Pre1_Error[0]*0.8+Pre1_Error[1]*0.2;
  return Turn_Out_Filtered;
}

extern float Podao_sign;//坡道标志
extern int16 Clear_time_podao;
int16 special_sign = 0;
//SetSpeedMax_temp;
//Speed_Kp_temp;
//Turn_Kp_temp;
//Turn_Kd_temp;
//Cir_threshold_temp;
//Angle_Kp_temp;
//Angle_Kd_temp;
void special_temp_deal(void)//每5ms执行一次
{
  special_sign = 10000;

  if(Podao_sign)//坡道处理            情形1
  {
    Clear_time_podao--;//清除倒计时
    if(Clear_time_podao>70)SetSpeedMax = 155;
    if(Clear_time_podao<=70)SetSpeedMax = 140;
    Speed_Kp = 55;//加速别太快
    Turn_Kp = 100;//防止抖动过大
  }

  if(!run_start)//启动速度过低    情形2
  {
    Turn_Kp = 70;
    Speed_Kp = 70;
  }
  else if(!Podao_sign&&run_start)//正常路段
  {
    SetSpeedMax=SetSpeedMax_temp;
    Speed_Kp=Speed_Kp_temp;
    Turn_Kp=Turn_Kp_temp;
    Turn_Kd=Turn_Kd_temp;
    Cir_threshold=Cir_threshold_temp;
    Angle_Kp=Angle_Kp_temp;
    Angle_Kd=Angle_Kd_temp;
  }

}



void control_oled(void)
{
  /**********************************************************OLED ************************/

  OLED_P8x16num(0,0,Angle);
  OLED_P8x16num(0,2,Gyropitch);//俯仰
  OLED_P8x16num(0,4,Gyroroll);//转向
  OLED_P8x16num(64,0,AngleOut);
  OLED_P8x16num(64,2,-g_fSpeedControlOut);
  OLED_P8x16num(64,4,(int)g_fDirectionControlOut);

  OLED_P8x16num(10,6,Get_Vbat());

}
extern int16 right_ad1;           // 右1传感器
extern int16 left_ad1;            // 左1传感器
extern int16 middle_ad;           //中传感器
extern int16 right_ad2;           //右2传感器
extern int16 left_ad2;            //左2传感器
extern int16 middle_ad2;           //中传感器
void senddata(void)
{
  //***********************************虚拟示波器****************************************/
  int16 var[5];
  //	var[0] =AccZ_Angle;
  //	var[1] =Gyropitch;
  //	var[2] =Angle;
  var[3] =leftspeed+rightspeed;
  //	var[4] =ControlSpeed;
  //	int16 var[5];
  var[0] =left_ad1;
  var[1] =left_ad2;
  var[2] =middle_ad;
  //var[3] =right_ad2;
  var[4] =right_ad1;
  //var[4] =middle_ad2;

  vcan_sendware((uint8_t *)var, sizeof(var));

}

uint8 Page_Index=0,Para_Index=1,Para_Checked=0,Para_Choice=0,OLED_Refresh=0;
uint8 Para_Index_Limit=7;
uint8 Display_Edge=1;
extern int16 Big_Cir_Is;
extern int16 Sum_Cir_Num ;

float Step[7]= {0.01,0.1,1.0,10.0,100.0,1000.0,10000.0};  //默认初始调节步长为0.01，下面的2选取
unsigned char Step_Index=2;
//admin_stop_time

void OLED_Draw_UI()  //OLED调参
{

  uint8 i;
  /*********************************光标上移+数值增加***************************修改******/
  /*
  Page_Index页面序号
  Para_Index0~6的行序号
  */
  if(PTI2_IN==0)
  {
    DELAY_MS(10);
    if(PTI2_IN==0)
    {
      if(Para_Choice==true)
      {
        if(Para_Checked==false)
        {
          if(Para_Index==0) Para_Index=Para_Index_Limit;
          else Para_Index-=1;
        }
        else
        {
          /**基本参数**/
          if(Page_Index==0&&Para_Index<=6)                    //修改第3页的参数
          {
            if(Para_Index==5)AngelSpeed_Kd+=Step[Step_Index];
            if(Para_Index==6)what+=Step[Step_Index];
            if(Para_Index==1)Speed_Kp+=Step[Step_Index];
            if(Para_Index==2)Speed_Ki+=Step[Step_Index];
            if(Para_Index==0)SetSpeedMax+=Step[Step_Index];
            if(Para_Index==3)Turn_Kp+=Step[Step_Index];
            if(Para_Index==4)Turn_Kd+=Step[Step_Index];
          }
          /**调试模式**/
          if(Page_Index==2&&Para_Index<=6)                    //修改第2页的参数
          {
            if(Para_Index==0)Angle_Kp+=Step[Step_Index];
            if(Para_Index==1)Angle_Kd+=Step[Step_Index];
            if(Para_Index==2)Anglefloat+=Step[Step_Index];
            //                                          if(Para_Index==3)                                //调试模式
            //                                          {
            //                                            if(Debug_Index==4)Debug_Index=0;
            //                                            else Debug_Index++;
            //                                          }

            if(Para_Index==3)Fuzzy_Kp+=Step[Step_Index];    //大圆环
            if(Para_Index==4)Fuzzy_Kd+=Step[Step_Index];    //大圆环
            if(Para_Index==5)admin_stop_time+=Step[Step_Index];    //大圆环
            if(Para_Index==6)what+=Step[Step_Index];    //大圆环
          }
          if(Page_Index==1&&Para_Index<=6)                    //修改第3页的参数
          {

            if(Para_Index==0)Sum_Cir_Num+=Step[Step_Index];
            if(Para_Index==1)Mid_Cir_Is+=Step[Step_Index];
            if(Para_Index==2)Mid_Cir2_Is+=Step[Step_Index];    //大圆环
            if(Para_Index==3)Big_Cir_Is+=Step[Step_Index];
            if(Para_Index==4)Cir_LR_threshold+=Step[Step_Index];    //大圆环
            if(Para_Index==5)Cir_threshold+=Step[Step_Index];    //大圆环
            if(Para_Index==6)Podao_threshold+=Step[Step_Index];    //大圆环
          }
        }
      }


    }
    while(PTI2_IN==0);//直到按键松开再运行
  }
  /********************************光标下移+数值减小****************************修改*****/
  /*
  Page_Index页面序号    E6

  Para_Index0~6的行序号
  */
  if(PTE2_IN==0)
  {
    DELAY_MS(10);
    if(PTE2_IN==0)
    {
      if(Para_Choice==true)
      {
        if(Para_Checked==false)
        {
          if(Para_Index==Para_Index_Limit)Para_Index=0;   //防止序号超出范围   Para_Index为0~6
          else  Para_Index+=1;
        }
        else
        {
          /**PID参数**/
          if(Page_Index==0&&Para_Index<=6)                    //修改第3页的参数
          {
            if(Para_Index==5)AngelSpeed_Kd-=Step[Step_Index];
            if(Para_Index==6)what-=Step[Step_Index];
            if(Para_Index==1)Speed_Kp-=Step[Step_Index];
            if(Para_Index==2)Speed_Ki-=Step[Step_Index];
            if(Para_Index==0)SetSpeedMax-=Step[Step_Index];
            if(Para_Index==3)Turn_Kp-=Step[Step_Index];
            if(Para_Index==4)Turn_Kd-=Step[Step_Index];
          }


          /**调试模式**/
          if(Page_Index==2&&Para_Index<=6)                    //修改第1页的参数
          {
            if(Para_Index==0)Angle_Kp-=Step[Step_Index];
            if(Para_Index==1)Angle_Kd-=Step[Step_Index];
            if(Para_Index==2)Anglefloat-=Step[Step_Index];
            //                                    if(Para_Index==3)                                //调试模式
            //                                    {
            //                                      if(Debug_Index==0)Debug_Index=4;
            //                                      else Debug_Index--;
            //                                    }
            if(Para_Index==3)Fuzzy_Kp-=Step[Step_Index];    //大圆环
            if(Para_Index==4)Fuzzy_Kd-=Step[Step_Index];    //大圆环
            if(Para_Index==5)admin_stop_time-=Step[Step_Index];    //大圆环
            if(Para_Index==6)what-=Step[Step_Index];    //大圆环
          }


          if(Page_Index==1&&Para_Index<=6)                    //修改第3页的参数
          {
            if(Para_Index==0)Sum_Cir_Num-=Step[Step_Index];
            if(Para_Index==1)Mid_Cir_Is-=Step[Step_Index];
            if(Para_Index==2)Mid_Cir2_Is-=Step[Step_Index];    //大圆环
            if(Para_Index==3)Big_Cir_Is-=Step[Step_Index];
            if(Para_Index==4)Cir_LR_threshold-=Step[Step_Index];    //大圆环
            if(Para_Index==5)Cir_threshold-=Step[Step_Index];    //大圆环
            if(Para_Index==6)Podao_threshold-=Step[Step_Index];    //大圆环
          }
        }
      }
    }

    while(PTE2_IN==0);  //直到按键松开再运行
  }

  /************************************取消选择***************************************/
  if(PTC7_IN==0)
  {
    //去抖
    DELAY_MS(10);
    if(PTC7_IN==0)
    {
      if(Para_Choice==true) Para_Choice=false; //不选择参量
      if(Para_Checked==true)Para_Checked=false;
      if(Page_Index==2)
      {
        if(Display_Edge) Display_Edge=0;
        else Display_Edge=1;
      }
    }
    while(PTC7_IN==0);  //直到按键松开再运行
  }

  /**************五项按键向左-翻页-设置页面数-加减步长0.01-10的改变*********修改********/
  /*
  Page_Index页面序号
  Para_Index0~6的行序号
  */
  if(PTC6_IN==0)
  {
    //去抖
    DELAY_MS(10);
    if(PTC6_IN==0)
    {
      if(Para_Checked)          //选定数值后，向左按键能使步长增加10倍
      {
        if(Step_Index==6)
          Step_Index=0;   //最大的步长为10   向左可移动到0.0001 、0.001，灵活性更增加
        else Step_Index++;
      }
      else
      {
        Para_Index=0;
        if(Page_Index==0) Page_Index=2;                                         //当参数没被选中的时候，按左右键翻页 显示的页面数（当前3页）
        else Page_Index--;
        OLED_Fill(0);//清屏
      }
    }
    while(PTC6_IN==0);//直到按键松开再运行
  }

  /*************************参量+数值选择***********************/
  /*
  Page_Index页面序号
  Para_Index0~6的行序号
  */
  if(PTI3_IN==0)
  {
    //去抖
    DELAY_MS(10);
    if(PTI3_IN==0)
    {
      if(Para_Index==7) //上下连接
      {

        Para_Index=0;
      }
      else
      {
        if(Para_Choice==false)  Para_Choice=true;  //  选择参量
        else
        {
          if(Para_Checked==false) Para_Checked=true;
          else Para_Checked=false;
        }
      }
    }
    while(PTI3_IN==0); //直到按键松开再运行
  }

  ////////////////////////OLED显示///////////////////////////////////////////

  /**白色选择**/
  else if(Page_Index==0)

  {
    for(i=0; i<7; i++)
    {
      if(i==Para_Index&&Para_Choice==true&&Para_Checked==false)
      {
        reverse=1;
        OLED_P6x8Str(0,i+1,Para_Switch1[i]);   //将OLED显示的参量名反转显示
        reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Switch1[i]);
    }
    OLED_Set_Pos(116,i+1);

    ///////////////////
    if(Para_Index==5&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 6,AngelSpeed_Kd,5);//第6行显示程序变量名的数值
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 6,AngelSpeed_Kd,5);

    if(Para_Index==6&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 7,what,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 7,what,5);

    if(Para_Index==1&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 2,Speed_Kp,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 2,Speed_Kp,5);

    if(Para_Index==2&&Para_Checked)//这里添加例程在第1页第七行显示hello  1.0000
    {
      reverse=1;
      OLED_PrintValueF(72, 3,Speed_Ki,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 3,Speed_Ki,5);
    if(Para_Index==0&&Para_Checked)//这里添加例程在第1页第七行显示hello  1.0000
    {
      reverse=1;
      OLED_PrintValueF(72, 1,SetSpeedMax,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 1,SetSpeedMax,5);
    if(Para_Index==3&&Para_Checked)//这里添加例程在第1页第七行显示hello  1.0000
    {
      reverse=1;
      OLED_PrintValueF(72, 4,Turn_Kp,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 4,Turn_Kp,5);
    if(Para_Index==4&&Para_Checked)//这里添加例程在第1页第七行显示hello  1.0000
    {
      reverse=1;
      OLED_PrintValueF(72, 5,Turn_Kd,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 5,Turn_Kd,5);
  }

  ///////////////////////////////////page2////////////////////////////////////////
  else if(Page_Index==2)

  {       //////////////////////////////page2 参量名显示
    for(i=0; i<7; i++)
    {
      if(i==Para_Index&&Para_Choice==true&&Para_Checked==false)
      {
        reverse=1;
        OLED_P6x8Str(0,i+1,Car_Set_Name[i]);
        reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Car_Set_Name[i]);

      OLED_Set_Pos(116,i+1);

    }
    //////////////////////////////page2  参数显示
    if(Para_Index==0&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 1,Angle_Kp,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 1,Angle_Kp,5);

    if(Para_Index==1&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 2,Angle_Kd,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 2,Angle_Kd,5);

    if(Para_Index==2&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 3,Anglefloat,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 3,Anglefloat,5);

    //		if((Para_Index==3)&&Para_Checked)  //调试模式
    //		{
    //                  reverse=1;
    //                  OLED_P6x8Str(72, 4,Debug_Mode[Debug_Index]);
    //                  reverse=0;
    //		}
    //		else
    //                  OLED_P6x8Str(72, 4,Debug_Mode[Debug_Index]);
    if(Para_Index==3&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 4,Fuzzy_Kp,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 4,Fuzzy_Kp,5);
    if(Para_Index==4&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 5,Fuzzy_Kd,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 5,Fuzzy_Kd,5);
    if(Para_Index==5&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 6,admin_stop_time,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 6,admin_stop_time,5);

    if(Para_Index==6&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 7,what,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 7,what,5);
  }



  ////////////////////////////////////page3///////////////////////////////////////
  else if(Page_Index==1)

  {       //////////////////////////////page3 参量名显示
    for(i=0; i<7; i++)
    {
      if(i==Para_Index&&Para_Choice==true&&Para_Checked==false)
      {
        reverse=1;
        OLED_P6x8Str(0,i+1,Para_Switch2[i]);   //反转显示
        reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Switch2[i]);//正常显示

      OLED_Set_Pos(116,i+1);

    }
    //////////////////////////////page3  圆环方向选择显示

//       Sum_Cir_Num
//       Mid_Cir_Is
//       Big_Cir_Is
//       Big_Cir2_Is
//       Cir_threshold

    if(Para_Index==0&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 1,Sum_Cir_Num,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 1,Sum_Cir_Num,5);

    if(Para_Index==1&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 2,Mid_Cir_Is,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 2,Mid_Cir_Is,5);
    if(Para_Index==2&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 3,Mid_Cir2_Is,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 3,Mid_Cir2_Is,5);

    if(Para_Index==3&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 4,Big_Cir_Is,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 4,Big_Cir_Is,5);

    if(Para_Index==4&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 5,Cir_LR_threshold ,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 5,Cir_LR_threshold ,5);
    if(Para_Index==5&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 6,Cir_threshold,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 6,Cir_threshold,5);

    if(Para_Index==6&&Para_Checked)
    {
      reverse=1;
      OLED_PrintValueF(72, 7,Podao_threshold,5);
      reverse=0;
    }
    else
      OLED_PrintValueF(72, 7,Podao_threshold,5);
  }

  //退出循环
  if(PTA0_IN==0)
    DELAY_MS(20);
  {
    if(PTA0_IN==0)
      ready=true;

  }

}

////////////////////////////////////////贝克模糊PID////////////////////////////////////////
/*Start:
*模糊PID
*返回隶属度
*输入PianCha绝对值 基准数  小值 大值
*End*/
uint16_t Dajiao_Change_RangeB = 40;
uint16_t Dajiao_Change_RangeS = 15;

double Final_P;
double Final_D;
double Findl_D_T;

#define RANGE_MOHU_S 5  //小区域
#define RANGE_MOHU_B 10 //大区域

double MoHu_PID(double temppiancha, double basevalue, double smallvalue, double bigvalue)
{
  static double k1;
  static double k2;
  static int temp = 0;

  //无的区间
  if (temppiancha <= Dajiao_Change_RangeS - RANGE_MOHU_S) /* --x----x--*/
  {
    // setText用户自定义("中间:" + basevalue);
    return basevalue; //返回基准值
  }
  //无到小的装换区间内
  else if (temppiancha > Dajiao_Change_RangeS - RANGE_MOHU_S && temppiancha < Dajiao_Change_RangeS + RANGE_MOHU_S)
  {
    temp = temppiancha - (Dajiao_Change_RangeS - RANGE_MOHU_S); //计算超出的区域
    k1 = (2 * RANGE_MOHU_S - temp) * 1.0 / (2 * RANGE_MOHU_S);
    k2 = 1 - k1;                               //斜率相同 减少计算量
    //                      setText用户自定义("无到小k1:" + k1);
    //                      setText用户自定义("无到小k2:" + k2);
    //                      setText用户自定义("无到小1:" + basevalue * k1);
    //                      setText用户自定义("无到小2:" + smallvalue * k2);
    return (basevalue * k1 + smallvalue * k2); //返回基准值加上小区域的值
  }
  //小的区间
  else if (temppiancha >= Dajiao_Change_RangeS + RANGE_MOHU_S && temppiancha <= Dajiao_Change_RangeB - RANGE_MOHU_B) //已经处于小区域了
  {
    //  setText用户自定义("小:" + smallvalue);
    return (smallvalue);
  }
  //小到大的区间
  else if (temppiancha > Dajiao_Change_RangeB - RANGE_MOHU_B && temppiancha < Dajiao_Change_RangeB + RANGE_MOHU_B)
  {
    temp = temppiancha - (Dajiao_Change_RangeB - RANGE_MOHU_B); //计算超出的区域
    k1 = (2 * RANGE_MOHU_B - temp) * 1.0 / (2 * RANGE_MOHU_B);
    k2 = 1 - k1;
    //                        setText用户自定义("无到小k1:" + k1);
    //                        setText用户自定义("无到小k2:" + k2);
    //                        setText用户自定义("小到大1:" + smallvalue*k1);
    //                        setText用户自定义("小到大2:" + bigvalue*k2);
    return (smallvalue * k1 + bigvalue * k2); //返回基准值加上小区域的值
  }
  //大的区间
  else if (temppiancha >= Dajiao_Change_RangeB + RANGE_MOHU_B)
  {
    // setText用户自定义("大:" + bigvalue);
    return bigvalue;
  }
  else
  {
    //  setText用户自定义("错误取件");
    return 50; //返回错误值
  }
}


//意外停车函数
void Accident_stop()
{

  if(Duikang_stop)
  {
   DisableInterrupts;

    PWMSet(FTM_CH0, 0 );
    PWMSet(FTM_CH1, 350);
    PWMSet(FTM_CH2, 350);
    PWMSet(FTM_CH3, 0 );

    led(LED0, LED_ON);
    led(LED1, LED_ON);
    led(LED2, LED_ON);
    led(LED3, LED_ON);

    PWMSet(FTM_CH0, 0 );
    PWMSet(FTM_CH1, 0);
    PWMSet(FTM_CH2, 0);
    PWMSet(FTM_CH3, 0 );
    port_pull(PTA1,ENABLE);

    over = false;
    Stop=true;
    starting=false;
    ready=false;
  }





  if(over_stop)                           //正常停车
  {
    DELAY_MS(50);
    DisableInterrupts;

    PWMSet(FTM_CH0, 0 );
    PWMSet(FTM_CH1, 350);
    PWMSet(FTM_CH2, 350);
    PWMSet(FTM_CH3, 0 );
     DELAY_MS(30);            //等待两秒进入调参页面
    led(LED0, LED_ON);
    led(LED1, LED_ON);
    led(LED2, LED_ON);
    led(LED3, LED_ON);


    PWMSet(FTM_CH0, 0 );
    PWMSet(FTM_CH1, 0);
    PWMSet(FTM_CH2, 0);
    PWMSet(FTM_CH3, 0 );
  //  port_pull(PTA1,ENABLE);

    over = false;
    Stop=true;
    starting=false;
    ready=false;
  }
}
