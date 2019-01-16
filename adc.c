/******************* (C) COPYRIGHT 2018.7.10  JF TEAM *****START OF FILE************/
/*!
* @file       main.c
* @brief      疾风KEA128主程序
* @author     南工电控疾风队
* @version    v8.0
* @date       2018-7-9

*/
#include "include.h"
#define value_middle 2000

//与圆环相关的清除时间
int16 Clear_time = 38;
int16 Circle_AutoClear_time= 600;//3s后清楚圆环标志

float value=0;
float numerator=0.0;              //分子
float denominator=1.0;            //分母  (不为0)

float error_d=0;

int16 denominator_avr=1;
int16 denominator_avr_practice=1;
int16 denominator_ctrl=1;
int16 podao_judge_practice = 0;
int16 podao_xiabiansuanfa=0;
int16 middle_ctrl=100;

int16 round_left=0;
int16 round_right=0;
int16 round_left_sign=0;
int16 round_right_sign=0;
int16 round_flag=0;
int16 round_flag1=0;

float Cir_value=0;
int16 value_circle = 0;
int16 Clear_time_podao = 0;

float Clear_cir_Yes = false;
float Out_Cir_deal = false;//出圆环反方向走直线处理标志
float Out_Cir_deal_left_to_right = false;//左环处理
float Out_Cir_deal_right_to_left = false;
float In_circle = false;
float Circle_autoclear = false;


float left_cir=false;    //左圆环
float right_cir=false;
float left_big=false;    //左大圆环
float right_big=false;
float left_mid=false;    //左大圆环
float right_mid=false;
float Podao_sign=false;//坡道标志

int16 sign = 0;//待删

int16 right_ad1=0;           // 右1传感器
int16 left_ad1=0;            // 左1传感器
int16 middle_ad=0;           //中传感器
int16 right_ad2=0;           //右2传感器
int16 left_ad2=0;            //左2传感器
int16 middle_ad2=0;          //左2传感器
int16 left_and_right=0;          //上位机用来作为坡道和圆环的阈值判断 page3中的两个参数大小

int16 RIGHT=0;             //P1
int16 FORWARD=0;           //P7
int16 LEFT=0;              //P9

uint16 S,S1,S2,S3,S4;

extern float Gyroroll;//陀螺仪转向防止坡道误判
float Gyroroll_add = 0;
extern float Angle; //卡尔曼重合角度
extern int16 SetSpeedMax;

/***************************************AD初始化************************************/

void ADInit()
{
  //电感数值采集
  adc_init(ADC0_SE6);             // ADC 5   左
  adc_init(ADC0_SE7);             // ADC 4   右
  adc_init(ADC0_SE12);             // ADC 1   左2
  adc_init(ADC0_SE13);             // ADC 0   右2
  adc_init(ADC0_SE14);             // ADC 3   中
  adc_init(ADC0_SE15);             // ADC 6   中2
  adc_init(ADC0_SE2);             // adc   电源电压
}


/////////////////////////////////////重庆大学冒泡法电感取均值（暂不使用）////////////////////////////////////////////////
float left_adc,right_adc,left_adc2,right_adc2,mid_adc,mid_adc_old=0;
float noisy = false;

void EM_ADC()
{

  uint8 i, j;
  long  ADC_Sum[5];
  ADC_Sum[0]=0;
  ADC_Sum[1]=0;
  ADC_Sum[2]=0;
  ADC_Sum[3]=0;
  ADC_Sum[4]=0;

  //采集10次
#define FILTER_N 10
  float filter_temp, filter_sum_L = 0, filter_sum_R = 0,filter_sum_L2 = 0, filter_sum_R2 = 0,filter_sum_M = 0;
  float filter_buf_L[FILTER_N];
  float filter_buf_R[FILTER_N];
  float filter_buf_L2[FILTER_N];
  float filter_buf_R2[FILTER_N];
  float filter_buf_M[FILTER_N];

  for(i = 0; i < FILTER_N; i++)
  {
    filter_buf_M[i] = adc_once(ADC0_SE14,ADC_12bit);
    filter_buf_L[i] = adc_once(ADC0_SE6,ADC_12bit);
    filter_buf_R[i] = adc_once(ADC0_SE7,ADC_12bit);
    filter_buf_L2[i] = adc_once(ADC0_SE12,ADC_12bit);
    filter_buf_R2[i] = adc_once(ADC0_SE13,ADC_12bit);

  }

  //采样值从小到大排列（冒泡法）
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_L[i] > filter_buf_L[i + 1]) {
        filter_temp = filter_buf_L[i];
        filter_buf_L[i] = filter_buf_L[i + 1];
        filter_buf_L[i + 1] = filter_temp;
      }
    }
  }

  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_R[i] > filter_buf_R[i + 1]) {
        filter_temp = filter_buf_R[i];
        filter_buf_R[i] = filter_buf_R[i + 1];
        filter_buf_R[i + 1] = filter_temp;
      }
    }
  }
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_R2[i] > filter_buf_R2[i + 1]) {
        filter_temp = filter_buf_R2[i];
        filter_buf_R2[i] = filter_buf_R2[i + 1];
        filter_buf_R2[i + 1] = filter_temp;
      }
    }
  }
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_M[i] > filter_buf_M[i + 1]) {
        filter_temp = filter_buf_M[i];
        filter_buf_M[i] = filter_buf_M[i + 1];
        filter_buf_M[i + 1] = filter_temp;
      }
    }
  }
  for(j = 0; j < FILTER_N - 1; j++) {
    for(i = 0; i < FILTER_N - 1 - j; i++) {
      if(filter_buf_L2[i] > filter_buf_L2[i + 1]) {
        filter_temp = filter_buf_L2[i];
        filter_buf_L2[i] = filter_buf_L2[i + 1];
        filter_buf_L2[i + 1] = filter_temp;
      }
    }
  }


  //去除最大最小极值后求平均
  for(i = 1; i < FILTER_N - 1; i++)
  {
    filter_sum_L += filter_buf_L[i];
    filter_sum_R += filter_buf_R[i];
    filter_sum_L2 += filter_buf_L2[i];
    filter_sum_R2 += filter_buf_R2[i];
    filter_sum_M += filter_buf_M[i];

  }
  left_adc = filter_sum_L / (FILTER_N - 2);
  right_adc =filter_sum_R/ (FILTER_N - 2);
  left_adc2 = filter_sum_L2 / (FILTER_N - 2);
  right_adc2 =filter_sum_R2/ (FILTER_N - 2);
  mid_adc = filter_sum_M / (FILTER_N - 2);


  //防突变
  //  if((abs(left_adc-right_adc)<15)&&(abs(left_adc2-right_adc2)<15)&&((mid_adc- mid_adc_old)>210))
  //  {
  //  noisy = true;
  //  mid_adc = mid_adc_old*0.8+mid_adc*0.2;
  //  mid_adc = mid_adc_old*0.8+mid_adc*0.2;
  //  }
  //
  middle_ad = mid_adc  ;
  left_ad1  = left_adc  ;
  right_ad1 = right_adc  ;
  left_ad2  = left_adc2 ;
  right_ad2 = right_adc2 ;

  noisy = false;
  mid_adc_old = middle_ad ;
}


float round_dir=0;
int16 numnumnum = 200;

/*——————————————————电感采集———————————————————*/
float Get_ad()
{
  float a,b,c;
  static float old_value=0;
  //    static int round_left=0;
  //    static int round_right=0;
  static int left_adold[4];    //存储数值方便进行上升下降判断比如：
  static int right_adold[4];
  static int middle_ad_old[4];
  //        static int denominator_old[4];
  left_adold[0]=left_adold[1];  //左右斜电感的增减性判断左右圆环（nb）
  left_adold[1]=left_adold[2];
  left_adold[2]=left_adold[3];
  left_adold[3]=left_ad2;

  right_adold[0]=right_adold[1];
  right_adold[1]=right_adold[2];
  right_adold[2]=right_adold[3];
  right_adold[3]=right_ad2;
  //
  middle_ad_old[0]=middle_ad_old[1];
  middle_ad_old[1]=middle_ad_old[2];
  middle_ad_old[2]=middle_ad_old[3];
  middle_ad_old[3]=middle_ad;
  //
  //    denominator_old[0]=denominator_old[1];
  //	denominator_old[1]=denominator_old[2];
  //	denominator_old[2]=denominator_old[3];
  //	denominator_old[3]=denominator;
  //

  //////////////////////传顺掐头去尾找平均///////////////////

  int16 ad[5][10];
  int i=0,j=0;
  for(j=0;j<10;j++)
  {
    ad[0][j] = adc_once   (ADC0_SE14,  ADC_12bit);     // 中
    ad[1][j] = adc_once   (ADC0_SE6,   ADC_12bit);     // 左            //这里进行电感数值采集
    ad[2][j] = adc_once   (ADC0_SE7,   ADC_12bit);     // 右
    ad[3][j] = adc_once   (ADC0_SE12,  ADC_12bit);     // 左2
    ad[4][j] = adc_once   (ADC0_SE13,  ADC_12bit);     // 右2
  }

  //去掉最大值，去掉最小值，取平均
  for(i=0;i<5;i++)
  {
    uint16 maxad=0,minad=4095,sum=0;
    for(j=0;j<10;j++)
    {
      maxad= MAX( maxad , ad[i][j] );
      minad= MIN( minad , ad[i][j] );
      sum+=ad[i][j];
    }
    sum=(sum-maxad-minad)/8;
    switch(i)
    {
    case 0: middle_ad = sum  ; break;
    case 1: left_ad1  = sum  ; break;
    case 2: right_ad1 = sum  ; break;
    case 3: left_ad2  = sum  ; break;
    case 4: right_ad2 = sum  ; break;
    default: break;
    }
  }
  ///////////////////////////////////////////////////////////

  // EM_ADC();      //重庆大学冒泡

  left_and_right = left_ad1+right_ad1;

  if(left_ad1<=0)
    left_ad1=0;
  if(left_ad2<=0)
    left_ad2=0;
  if(middle_ad<=0)
    middle_ad=0;
  if(right_ad1<=0)
    right_ad1=0;
  if(right_ad2<=0)
    right_ad2=0;
  if(middle_ad2<=0)
    middle_ad2=0;

  numerator   = (float)(left_ad1*(-2000)+left_ad2*(-320)+middle_ad*0+right_ad2*320+right_ad1*2000);                //五电感检测   300的时候还不错
  denominator = (float)(left_ad1+  left_ad2+     middle_ad+     right_ad2+     right_ad1);
  old_value=numerator/denominator;


  //  numerator   = (float)(left_ad1*(-2000)+middle_ad*0+right_ad1*2000);                //三电感检测
  //  denominator = (float)(left_ad1+   middle_ad+   right_ad1);
  //  old_value=numerator/denominator;

  //    numerator   = (float)(left_ad1*0  +right_ad1*4000);                //二电感检测
  //    denominator = (float)(left_ad1+   right_ad1);
  //    old_value=numerator/denominator;




  //    //斜电感模拟偏差的导数
  //  	error_d = (float)((left_ad2*(-30)+right_ad2*30)/(left_ad2+middle_ad+right_ad2));//垂直电感的差比和作为偏差
  //	error_d = (error_d>= 32? 32:error_d);	//偏差限幅
  //	error_d = (error_d<=-32?-32:error_d);

  if(denominator>=100)
  {
    value=old_value;
  }
  /**********************左右极限判定**********************/
  if(denominator<100)
  {
    if(value>value_middle)
    {
      value=2000;
    }
    else if(value<value_middle)
    {
      value=-2000;
    }
    else
    {
      value=value;
    }
  }
//  /*____________________________________5.29出圆环走直线   目前仅测试大左圆环有效果，如果元素距离很近对后面元素反应也不好则得不偿失  国赛不使用_____________________________________*/
//  if(Clear_cir_Yes&&Out_Cir_deal_left_to_right)//左环向右处理
//  {
//     value=round_deal2(6,value,(middle_ad*0+right_ad1*2000)/( middle_ad+right_ad1));
//     sign = 10000;//只是用来看看出圆环的时候电感是多少
// }
//  if(Clear_cir_Yes&&Out_Cir_deal_right_to_left)//右环向左处理
//  {
//     value=round_deal2(8,value,(left_ad1*(-2000)+middle_ad*0)/(left_ad1+middle_ad));
//  }
//  //左右电机性能不同，13的时候很靠左侧  缺点：大弯出来急弯不好过，左小环参数还要调整



  /**********************小弯道处理*****************************/
  if(value>=1500)
  {

    a=value-1500;
    b=a/500;
    c=b*b+1;

    value=a*c+1500;
  }

  if(value<=-1500)
  {

    a=-1500-value;
    b=a/500;
    c=b*b+1;

    value=-1500-a*c;
  }
  /*-------限幅---------*/
  if(value>1500)
  {
    value=1500;
  }
  if(value<-1500)
  {
    value=-1500;
  }

  static uint32 run_times=0,cont=0;
  cont++;
  if(cont>=20)                                  //100ms算一次平均值
  {
    run_times++;
    denominator_avr=(uint32)(denominator_avr*1+denominator*9)/10;//求平均
    denominator_avr_practice = (denominator_avr-1200)*4;//选择性放大
    if(denominator_avr_practice<500)//清晰化处理
    {
      denominator_avr_practice = 0;
    }
    denominator_ctrl=(denominator_ctrl*(run_times-1)+denominator_avr)/run_times;
    cont=0;
  }

  /*________对抗停车________*/

  if(abs(left_ad1-right_ad1<10)&&abs( right_ad2-left_ad2<10)&&(SpeedNow>40)&&denominator<110)
  {Duikang_stop = true;}

  /*________出圆环走直线______*/
  Out_Circle_deal();

  /*________坡道程序denominator_avr两个阈值需要设置一下 _____*/
  Slope_deal();
  //  if(!In_circle &&(middle_ad>420)&&(left_ad1+right_ad1)<Podao_threshold&&(denominator_avr_practice>Cir_threshold))  //圆环判据&&中间电感到峰值的时候
  //  {
  //    SetSpeedMax = 155;
  //    Podao_sign = true;
  //    Clear_time_podao = 80;
  //
  //  }
  //  special_temp_deal();//坡道这一特殊元素识别后的处理函数
  //  if(!Clear_time_podao)//倒计时时间到
  //  {
  //    special_sign = 0;//特殊元素函数标志位
  //    Podao_sign = false;
  //  }



  /*_______________________________环岛程序（不能因为坡道影响了圆环的判断）_________倾斜角度大于46的时候保护不识别圆环____________________________*/
  //  if((middle_ad>500)&&(denominator_avr>Cir_threshold)&& (middle_ad-middle_ad_old[2]<15 )&&!round_flag&& !round_left && !round_right )
  if((Angle<48)&&(middle_ad>500)&&(left_ad1+right_ad1)> Cir_LR_threshold &&(denominator_avr_practice>Cir_threshold)&& (middle_ad-middle_ad_old[2]<15 )&&!round_flag&& !round_left && !round_right)  //圆环判据&&中间电感到峰值的时候
  {  //加了个左右电感和设置一个阈值，能避免坡道坡道
    if(In_circle == false)//此时可以入圆环
    {
      round_flag = 8;//轻延入环
      //      port_pull(PTA1,ENABLE);//蜂鸣器预警
      sign = 20000;//只是用来看看出圆环的时候电感是多少
    }
    if(In_circle == true)//此时正在出圆环
    {
      Clear_time = 40;   //出环标志清除时间（技巧）大环也可以跑过了
      Clear_cir_Yes = true;//出环准许
      Out_Cir_deal = true;//出环反方向跑标志
    }
  }
  /*--------------------------左右转向+八字电感增减性----------------------------*/
  if(round_flag)
  {
    round_flag--;//延时
    //Gyroroll_add+=Gyroroll_add+Gyroroll*2;//陀螺仪计数 *******************************************Gyroroll_add看看圆环的时候偏移是多少**************************************************

    /*___方向判断____*/
    if((left_ad2-left_adold[2]>0)&&(right_ad2-right_adold[2]<0)) //初步为左圆环  电感数值左增右减
    {
      round_left_sign++ ;//防抖保险
    }
    else if ((left_ad2-left_adold[2]<0)&&(right_ad2-right_adold[2]>0))
    {
      round_right_sign++;
    }
    /*___参数处理____*/
    if(round_left_sign>=7)//左圆环确认
    {
      if((Next_Cir_Num == Big_Cir_Is)||(Next_Cir_Num == Big_Cir2_Is)||(Next_Cir_Num == Big_Cir3_Is))//当前所在圆环与大圆环UI确认
      {round_left=100;left_big = true; value_circle  = 10000;} //大小入环处理时间+左大环标志
      else
      {round_left=100; value_circle  = 5000;}     //小环处理时间

      left_cir = true;     //左圆环标志
      Out_Cir_deal_left_to_right = true;//左环出环要往右走
      // value_circle  = 10000;//左圆环在上位机示波器中强调
    }
    else if(round_right_sign>=7)  //右圆环稍微慢一点
    {
      if((Next_Cir_Num == Big_Cir_Is)||(Next_Cir_Num == Big_Cir2_Is)||(Next_Cir_Num == Big_Cir3_Is))//当前所在圆环与大圆环UI确认
      {round_right=100;right_big = true; value_circle = -10000;}
      else
      {round_right=100; value_circle = -5000;}

      right_cir = true;
      Out_Cir_deal_right_to_left = true;
      // value_circle = -10000;
    }
  }
  /*----------------------------------------转向入环控制------------------------------------------*/
  /*
  后面的数字约小越内切
  */
  else if(left_cir)//左圆环标志位
  {
    port_pull(PTA1,ENABLE);//蜂鸣器预警
    Gyroroll_add = 0;//********************************************************转向累加清零*************************************************8
    In_circle = true;//入环标志                                           1
    if(left_big)//如果是左大圆环                                          2
    {value=round_deal4(round_left,value,(left_ad1*(-2000)+middle_ad*0)/(left_ad1+ middle_ad)+615);}//左大环参数不用改，速度180以上
    else//左小圆环                                                       3
    {value=round_deal1(round_left,value,(left_ad1*(-2000)+middle_ad*0)/(left_ad1+ middle_ad)+600);}// 610速度150以上完美内切

    round_left--;//策略执行时间                                           4
    if(!round_left)//执行完毕                                            5
    {left_cir=false;left_big = false;}//及时同步清除圆环标志和大圆环标志6
  }
  else if(right_cir)
  {
    port_pull(PTA1,ENABLE);//蜂鸣器预警
    Gyroroll_add = 0;
    In_circle = true;
    if(right_big)
    {value=round_deal4(round_right,value,(middle_ad*0+right_ad1*2000)/( middle_ad+right_ad1)-615);}//右大环参数
    else
    {value=round_deal1(round_right,value,(middle_ad*0+right_ad1*2000)/( middle_ad+right_ad1)-600);}//右小环参数   数值越大越晚打角

    round_right--;
    if(!round_right)
    {right_cir=false;right_big = false;}
  }
  else
  {
    round_flag=0;     //圆环确认清零
    value_circle = 0; //上位机标志清零
    round_left_sign=0;//左环确认防抖清零
    round_right_sign=0;
  }

  /*________最好最后加一个二次标志，防止入环没处理好误判，3秒钟后再次清除圆环标志_______*/
  if( Clear_cir_Yes )           //出圆环清标志,这里需要反方向跑一点，防止速度快卡边角（比如左环出环靠右走还明白啦？）
  {
    {
      /*这里会有出圆环柔顺处理，代码在AD函数中,AD采集会因为内部程序多这一问题导致采集较慢，需要优化*/
      Clear_time--; //技巧：小延时，保证圆环单一计数
    }
    if(!Clear_time)             //切点处已跑过
    {
      sign = 0;//测试出环蜂鸣器
      port_pull(PTA1,DISABLE);//出环停止蜂鸣器
      Clear_time = 38;          //清除保护（瞎扯） 100圆环清除时间为70比较合适   50的圆环清除时间40很合适
      Clear_cir_Yes = false;   //init
      Out_Cir_deal = false;
      In_circle = false;
      Out_Cir_deal_left_to_right = false;
      Out_Cir_deal_right_to_left = false;
      Circle_AutoClear_time = 600;//这里加一个，圆环正常跑完初始化
      Next_Cir_Num ++;          //准备下一个圆环
      // value_circle  = -10000;
    }
  }
  /*_______________________________环岛出环未识别清零____________________________*/
  if(In_circle)  //防止圆环出环如果没有识别到
  {
    Circle_autoclear = true;
    Circle_AutoClear_time--;//3.5s倒计时
  }
  if(!Circle_AutoClear_time)//倒计时时间到
  {
    port_pull(PTA1,DISABLE);
    Circle_autoclear = false;
    Circle_AutoClear_time = 600;//这里加一个，误判初始化
    Next_Cir_Num ++;
    In_circle = false;
  }


  if(value>2000)
  {
    value=2000;
  }
  if(value<=-2000)
  {
    value=-2000;
  }

  return value;
}

/*-------------------------------------中环处理办法-------------------------------------*/
/*
长时间
round_left=200
round_right=200
*/
double round_deal4(int times,float value1,float value2)//完美适配70及以上圆环
{

  double deal_value=0;
  if(times>80) deal_value= value1*5*(times-80)+value2*5*(100-times);
  else deal_value=value2*70;//入环后走10分之一左右的距离，乘的系数越大越切内

  return (deal_value/100);                //+rate*200

}
double round_deal3(int times,float value1,float value2)
{
  double deal_value=0;
  if(times>130) deal_value= value1*5*(times-130)+value2*5*(150-times);
  else if(times<50)     deal_value=value2*20  ;//deal_value=value1*2*(50-times)+value2*2*(times);
  else deal_value=value2*70;//入环后走10分之一左右的距离，乘的系数越大越切内
  return (deal_value/100);                //+rate*200

}

/*-------------------------------------大环处理办法-------------------------------------*/
/*
长时间
round_left=200
round_right=200
*/
double round_deal2(int times,float value1,float value2)
{

  double deal_value=0;
  if(times>180) deal_value= value1*5*(times-180)+value2*5*(200-times);
  else if(times<50) deal_value=value1*2*(50-times)+value2*2*(times);
  else deal_value=value2*70;//入环后走10分之一左右的距离，乘的系数越大越切内

  return (deal_value/100);                //+rate*200

}
/*-------------------------------------小环处理办法-------------------------------------*/
/*
短时间
round_left=100
round_right=100
value=round_deal1(     round_right,     value,     (middle_ad*0+right_ad1*2000)/( middle_ad+right_ad1)-620);
*/
double round_deal1(int times,float value1,float value2)
{
  double deal_value=0;
  if(times>80) {deal_value= value1*20*(times-80)+value2*3*(100-times); }      //value1*5*(times-80)+value2*5*(100-times);
  else if(times<50) {deal_value=value1*2*(50-times)+value2*2*(times); }
  else {deal_value=value2*160; }    //根据上位机来看，在这个区间*150数值上升恰到好处

  if(deal_value>46000)
  {
    deal_value=46000;
  }

  if(deal_value<-46000)
  {
    deal_value=-46000;
  }

  return (deal_value/100);                //+rate*200
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Out_Circle_deal()
{
  /*____________________________________5.29出圆环走直线   目前仅测试大左圆环有效果，如果元素距离很近对后面元素反应也不好则得不偿失  国赛不使用_____________________________________*/
  if(Clear_cir_Yes&&Out_Cir_deal_left_to_right)//左环向右处理
  {
     value=round_deal2(6,value,(middle_ad*0+right_ad1*2000)/( middle_ad+right_ad1));
     sign = 10000;//只是用来看看出圆环的时候电感是多少
 }
  if(Clear_cir_Yes&&Out_Cir_deal_right_to_left)//右环向左处理
  {
     value=round_deal2(8,value,(left_ad1*(-2000)+middle_ad*0)/(left_ad1+middle_ad));
  }
  //左右电机性能不同，13的时候很靠左侧  缺点：大弯出来急弯不好过，左小环参数还要调整
}

void Slope_deal()
{
  if(!In_circle &&(middle_ad>420)&&(left_ad1+right_ad1)<Podao_threshold&&(denominator_avr_practice>Cir_threshold))  //圆环判据&&中间电感到峰值的时候
  {
    SetSpeedMax = 155;
    Podao_sign = true;
    Clear_time_podao = 80;

  }
  special_temp_deal();//坡道这一特殊元素识别后的处理函数
  if(!Clear_time_podao)//倒计时时间到
  {
    special_sign = 0;//特殊元素函数标志位
    Podao_sign = false;
  }
}


int Get_Vbat()
{
  static int32 vbat_ad=0;
  static int32 vbat_value=8000;
  vbat_ad=adc_once(ADC0_SE2,  ADC_12bit);     //读取电池电压AD值
  vbat_value=(70*vbat_value+30*vbat_ad*3818/1000)/100;      //    *5000/4096   * 14.7/4.7     //低通滤波

  return   vbat_value;
}

/*******************************************OLED显示电感采集数值********************************************/

void Show_ad()
{
  char leftad1[5];
  char leftad2[5];
  char middlead[5];
  char rightad1[5];
  char rightad2[5];
  char valuead[5];

  sprintf(leftad1, "%04d", left_ad1);
  sprintf(leftad2, "%04d", left_ad2);
  sprintf(middlead,"%04d", middle_ad);
  sprintf(rightad1,"%04d", right_ad1);
  sprintf(rightad2,"%04d", right_ad2);
  sprintf(valuead, "%04d", left_and_right );

  OLED_P8x16Str(0,0,leftad1);
  OLED_P8x16Str(0,2,leftad2);
  OLED_P8x16Str(90,0,rightad1);
  OLED_P8x16Str(90,2,rightad2);
  OLED_P8x16Str(45,0,middlead);
  OLED_P8x16num(0,6,Angle);
  OLED_P8x16Str(45,6,valuead);

  printf("%d---%d---%d---%d---%d---%d   \n",left_ad1,left_ad2,middle_ad,right_ad2,right_ad1,left_and_right );

  DELAY_MS(100);
  //
}
/*-------control.c para--------*/

void    vcan_sendware(uint8 *wareaddr, uint32 waresize);
extern uint16 RoadType;
extern float g_fSpeedControlOut;
extern int16_t leftspeed,rightspeed,SpeedNow; //速度
extern float AngleOut;  //直立输出  限幅调整好了
extern float g_fDirectionControlOut,fValue_one,g_fDirectionControlOutNew, fDvalue_exper2, fDvalue_exper1, fDvalue;//转向输出  限幅3000 而且坡道的阈值0.2也要修改一下
extern int16 run_normal_eva,chaosu;//当期速度达到预设速度
extern float ERROR;
extern int16 stop_sign_practice,over_stop_sign ;
extern int16     SetSpeedMax_temp;                               //最大速度


//
//left_ad1;
//left_ad2;
//middle_ad;
//right_ad1;
//right_ad2;
/*------------end------------*/
void sendaddata(void)
{
  /*-------------------------------------虚拟示波器-------------------------------------*/
  int16 var[8];

  var[0] =value_circle;//
  var[1] =special_sign;//进圆环之前很短的时间内看看变化大不大
  var[2] =denominator_avr_practice;

  var[3] =denominator_avr;   //
  var[4] =SetSpeedMax;
  var[5] =middle_ad;//
  var[6] =right_ad1;
  var[7] =right_ad2;//看坡道识别后是否改变最大速度设定
  //  var[6] =Next_Cir_Num;
  //  var[7] =g_nSpeedControlPeriod;
  vcan_sendware((uint8_t *)var, sizeof(var));
}

//  fDvalue_exper2
//  var[3] =value_circle;
//  var[4] =right_ad2;   over_stop_sign

//  var[5] =value;
//  var[6] =Next_Cir_Num;
//  var[7] =g_nSpeedControlPeriod;

//  var[0] =fDvalue;
//  var[1] =value_circle;
//  var[2] =fDvalue_exper2;
