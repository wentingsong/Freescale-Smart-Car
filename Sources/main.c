#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


/////////////////////////////////////////////////////////////////////
//该例程PLL 80M，BUS 40M
//具体拨码开关等请参照DEMO板版图进行初始化及使用
//仅有数码管计数显示及PWM初始化
//若用AD采样，参考实验例程进行初始化及使用
//摄像头组若用BUS 80M参照以前的例程进行修改  相应的PWM初始化也需修改 否则会烧坏舵机！！！
/////////////////////////////////////////////////////////////////////



#define SteerCenter 1222            //设定舵机中间值
#define SteerLeft 942
#define SteerRight 1502
#define ATD_SEQURE 8                  //定义转换序列长度
#define Threshold  30                //设定阈值


//舵机控制 
uchar ad_data[ATD_SEQURE];                       //存放AD转换结果
int stop=0;
int stop1=0;

 
int interrupt_count=0;
uint count_count=0;
int yuzhi=30;
uchar label;
uint max;
int  speed;	//存放脉冲累加结果
int  pre_speed;
signed int temp0,temp1,temp2;
float index;
uchar a;
float steer=0;
signed int pos_30newAdd;	//最近30的 和
float  Line_center;                              //中心线位置

uchar Get_Line;                                  //指示此次有无检测到黑线

int  Control_Steer;
int Steer_PWM_Outputwan[37]={280,272,264,258,
240,225,205,185,160,135,110,90,
70,55,30,20,10,5,0,-5,-10,-20,-30,-55,
-70,-90,-110,-135,-160,-185,-205,-225,
-240,-258,-264,-272,-280}; 
/*int Steer_PWM_Outputzhi[37]={280,272,264,258,
240,225,205,185,160,135,110,90,
70,55,30,20,10,5,0,-5,-10,-20,-30,-55,
-70,-90,-110,-135,-160,-185,-205,-225,
-240,-258,-264,-272,-280};  */
/*int Steer_PWM_Output[37]={280,280,275,265,
255,240,225,200,175,155,130,110,
85,60,40,20,12,6,0,-6,-12,-20,-40,-60,
-80,-100,-120,-140,-160,-185,-215,-235,
-250,-265,-275,-280,-280}; */

/*int Steer_PWM_Outputwan[37]={280,255,235,215,
195,180,165,155,145,140,135,125,
115,100,85,65,45,25,0,-25,-45,-65,-85,-100,
-115,-125,-135,-140,-145,-155,-165,-180,
-195,-215,-235,-255,-280}; */

/*int Steer_PWM_Outputzhi[37]={280,260,240,220,
200,180,160,140,120,105,90,75,
60,45,30,20,10,5,0,-5,-10,-20,-30,-45,
-60,-75,-90,-105,-120,-140,-160,-180,
-200,-220,-240,-260,-280}; */
int Steer_PWM_Outputzhi[37]={280,250,220,200,
170,145,120,100,80,64,51,40,
30,21,18,13,8,4,0,-4,-8,-13,-18,-21,
-30,-40,-51,-64,-80,-100,-120,-145,
-170,-200,-220,-250,-280}; 
//int	dzhizhidao=35;
int	dzhizhidao=50;
int dzhiwandao=22;//60
//int dzhiwantozhi=35;  //28
int dzhiwantozhi=14;
int dzhizhitowan=14;//50
int dzhichusaidao=25; //  95


/*int pdzhiwandao=35;//20
int pdzhizhidao=10;
int pdzhizhitowan=35; //35
int pdzhiwantozhi=20;
int pdzhichusaidao=40;*/
int	spwm[10];   
int	pwm,D_pwm,PD_pwm;
uint zhidaochangdu=0;	//记录直道长度

#define zhixian 50	
//************************赛道类型	25   17
#define qidong	0	
#define zhidao	1	
#define wandao	2	
#define wantozhi	3	
#define chusaidao	4	
#define zhitowan	5	
#define end	6	


//*****************************************************************
#define wandaotozhidao 50	//弯道进入直道的积累
#define zhidaotowandao 300	//直道入弯的积累  //285
#define zhidaotowandao2 600//450 //350
//*****************************************************************
uint zhidaosudu=37;//直道速度 31 34
uint wandaosudu=27;//弯道速度21 20
uchar moto_start=0;	
uchar wandao_st=0;		


uint  deadline=0;

int  Pre_Control_Speed=0;	
int	Control_Speed=700;	
int  Pre_Control_Steer[10];	
int	D_Steer;	
	
//int	Control_Steer;

//#define Speed_K	35	//速度Kp
#define Speed_D 250 //180                    //450 400
#define Speed_P	500	//速度Ki     //450
//#define Speed_D	50	//速度Ki
	
//int  ant_Control_Speed;	
int	Pre_offset1=0;	
int	Pre_offset0=0;	
int	This_offset=0;	
//*******************************************************赛道状态
uchar	saidao_state=zhidao;	
uchar	pre_state=zhidao;	
int Speed_Wanted;	//目标速度


//*******************************************************

//*******************************************************
 signed char position[100];

float chusaidao_pos;
 
#pragma CODE_SEG DEFAULT


unsigned char Led[] = {0xFB,0xF7,0xDF,0xBF};                        //PE2.3.5.6口显示
unsigned char LedCode[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};    //数码管显示
unsigned char LedData[]={0,0,0,0};
unsigned char LedNum = 0;

int i,j,k; 
int time = 0;
int count = 0;


void initGPIO(void){         //通用IO口初始化
    DDRA = 0x00;             //A口输入
    DDRB = 0xFF;             //B口输出
    DDRE = 0x00;             //E口2.3.5.6  LED输出
    DDRS = 0xFF;
    DDRP = 0x05; 
            //单电机的话只要用到PTP_PTP0作为IO，PP1作为PWM输出进行正反转控制  双电机用到PTP_PTP2 和PP3
    //PPSJ_PPSJ7 = 1;//positive edge
    //PIFJ_PIFJ7 = 1;//write 1 to clear interrupt flag 
   // PIEJ_PIEJ7 = 1;//interrupt enable  
}




//
void ATD_init(void){                   //ATD初始化
    ATD0CTL0 = 0x07;                  //多路转换时转换8路
    ATD0CTL1_SRES = 0;           //转换精度为8位
    ATD0CTL2_AFFC = 1;                //中断标志位自动清零
    ATD0CTL2_ASCIE = 1;               //一个序列传唤结束触发中断
    ATD0CTL3 = 0xC0;                  //结果寄存器对齐方式右对齐，转换序列长度为8，循环转换，freeze模式下继续转换
    //ATD0CTL4 = 0x04;                  //采样周期为4个周期，atdclk=busclk/2*(0+1),无锁相环总线时钟为8M，故
    ATD0CTL4 = 0x08;                                 //ATDclk=4M
    ATD0CTL5_SCAN = 1;                //连续转换模式 
    ATD0CTL5_MULT = 1;                //多通道采样
    }  //
    
void initPLL(void){                 //锁相环初始化，将总线频率调整到40M
    CLKSEL=0x00;                        //禁止锁相环，时钟有外部晶振提供，总线频率=外部晶振/2
    PLLCTL_PLLON=1;                     //打开锁相环
    SYNR=0x49;          
    REFDV=0x43;                         // pllclock=fvco=2*osc*(1+SYNR)/(1+REFDV)=80MHz;
    POSTDIV = 0x00;
    _asm(nop);                           //BUS CLOCK=40M
    _asm(nop);
    while(!(CRGFLG_LOCK==1));            // 等待锁相环初始化完成
    CLKSEL_PLLSEL =1;                    // 使用锁相环 
}
void initPWM(void){          
     
     PWMPOL = 0xFF;                      //PWM极性选择，选择一个周期开始时为高电平
     PWMPRCLK = 0x22;                    //CLOCK A，B时钟分频，均选择从总线四分频 10M
     PWMSCLA  = 5;                       //CLOCK SA从CLOCK A十分频，1M
     PWMSCLB  = 5;                       //CLOCK SB从CLOCK B十分频，1M
     PWMCTL = 0xF0;                      //01级联，23级联，45级联，67级联
     PWMCLK = 0xFF;                      //PWM始终选择，选择CLOCK SA SB为PWM时钟

     PWMPER01=1000;                       //第一个电机频率为1k
     PWMDTY01=0;
     
     PWMPER23=1000;
     PWMDTY23=0;                           //第二个电机频率为1k
         
     PWMPER67=20000;                        //舵机PWM频率为50Hz  
     PWMDTY67=SteerCenter;               
             
     PWME_PWME1=1;                           
     PWME_PWME7=1;                          
     PWME_PWME3=1;                           //PWM开始输出
                                                                       
}
void InitRTI(void) {                      // 实时中断初始化                     
    RTICTL = 0xaf;
    //RTICTL = 0xaf;//10msRTICTL = 0x83;                            //0.25ms中断
    CRGINT = 0x80;                            // 打开实时中断
}
void InitPACNT(void){
    PACTL = 0x40;                        //脉冲累加功能，下降沿计数
    PACNT = 0; 
   // TSCR1=0x80;
    //TSCR2=0x60;      
} 
void check(void){
label=PORTE;
if(label==251||label==247||label==243) count_count++;
}

//uint a;
void main(void) {
  /* put your own code here */
  uchar pa;
uint ii;

  DisableInterrupts;
for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);
for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);

for(ii=0;ii<60000;ii++);   

  initPLL();
  InitRTI();
  initGPIO();
  ATD_init();
  //PIH_init();
  initPWM();
  InitPACNT();
  PTS = 0x00;
  /*for(i=0;i<4;i++)
    for(j=0;j<300;j++)
      for(k=0;k<300;k++)
        if(j%10 == 0 && k%100 ==0){
          PORTE=Led[i%4];                 //E口流水灯
        }  */
  PORTE = 0xFF;   
/***********干簧管检测****/

//if(interrupt_count>=300) label=PORTE;
  
  /**************************************拨码开关**/
pa=PORTA;

/*if(!(pa&0x04)&&!(pa&0x02)&&!(pa&0x01)) //000
{
zhidaosudu=33;
}
else if(!(pa&0x04)&&!(pa&0x02)&&(pa&0x01))//001
{
//for(ii=0;ii<49;ii++)Steer_PWM_Output[ii]=Steer_PWM_Output0[ii];
zhidaosudu=34;
}
else if(!(pa&0x04)&&(pa&0x02)&&!(pa&0x01))//010
{
zhidaosudu=35;
}
else if(!(pa&0x04)&&(pa&0x02)&&(pa&0x01)) //011
{
zhidaosudu=36;
}
else if((pa&0x04)&&!(pa&0x02)&&!(pa&0x01)) //100
{
zhidaosudu=37;
}
else if((pa&0x04)&&!(pa&0x02)&&(pa&0x01))//101
{
zhidaosudu=38;
}
else if((pa&0x04)&&(pa&0x02)&&!(pa&0x01))//110
{
zhidaosudu=39;

}
else if((pa&0x04)&&(pa&0x02)&&(pa&0x01)) //111
{
zhidaosudu=40;
}
pa=PORTA;
if(!(pa&0x40)&&!(pa&0x20)&&!(pa&0x10)) //000
{
wandaosudu=22;
// for(ii=0;ii<49;ii++)Steer_PWM_Output[ii]=Steer_PWM_Output0[ii];
}
else if(!(pa&0x40)&&!(pa&0x20)&&(pa&0x10)) //001
{
wandaosudu=23;
// for(ii=0;ii<49;ii++)Steer_PWM_Output[ii]=Steer_PWM_Output1[ii];
}
else if(!(pa&0x40)&&(pa&0x20)&&!(pa&0x10)) // 010
{
wandaosudu=24;
}
else if(!(pa&0x40)&&(pa&0x20)&&(pa&0x10)) // 011
{
wandaosudu=25;
}
if((pa&0x40)&&!(pa&0x20)&&!(pa&0x10)) // 100
{
wandaosudu=26;
}
else if((pa&0x40)&&!(pa&0x20)&&(pa&0x10)) // 101
{
wandaosudu=27;
}
else if((pa&0x40)&&(pa&0x20)&&!(pa&0x10)) //110
{
wandaosudu=28;
}
else if((pa&0x40)&&(pa&0x20)&&(pa&0x10)) // 111
{
wandaosudu=29;
}
*/
if (pa==0x00){
zhidaosudu=30;
wandaosudu=20;	
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25; 
yuzhi= 30;
} 
else if (pa==0x01){
zhidaosudu=31;
wandaosudu=21;
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25; 
yuzhi= 30;
}
else if (pa==0x02){
zhidaosudu=32;
wandaosudu=22;
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25;
yuzhi=  30;
}
else if (pa==0x03){
zhidaosudu=33;
wandaosudu=23;
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25; 
yuzhi= 30;
}
else if (pa==0x04){
zhidaosudu=34;
wandaosudu=24;dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25; 
yuzhi= 30;
}
else if (pa==0x05){
zhidaosudu=35;
wandaosudu=25;
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25; 
yuzhi=  35;
}
else if (pa==0x06){
zhidaosudu=36;
wandaosudu=26;
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25; 
yuzhi= 35;
}
else if (pa==0x07){
zhidaosudu=37;
wandaosudu=26;
dzhizhidao=50;
dzhiwandao=22;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25;
yuzhi=  35 ;
}
else if (pa==0x08){
zhidaosudu=37;
wandaosudu=27;
dzhizhidao=50;
dzhiwandao=20;
dzhiwantozhi=14;
dzhizhitowan=14;
dzhichusaidao=25;
yuzhi=  37;
}
	EnableInterrupts;

 

  //PWMDTY67=1480;
  for(;;) {
     if (interrupt_count>500) check();

    
  
    PTP_PTP0 = 0;      //第一路电机控制    反转将  PTP_PTP0 = 1 即可
    //PWMDTY01 = 100;
    
    //PTP_PTP2 = 0;     //第二路电机控制  反转将  PTP_PTP2 = 1 即可
    //PWMDTY23 = 100;
    _FEED_COP(); /* feeds the dog */

if(stop1>400)
{

PWMDTY01=0;
DisableInterrupts;
}    
    	
//	 if(lap_cnt > 1)   {
//  PWME_PWME3=0;

//PWME_PWME1=0; 

//PWMDTY01=0;

//  DisableInterrupts;
// }
    
  } /* loop forever */
  /* please make sure that you never leave main */
  

 



}
#pragma CODE_SEG __NEAR_SEG NON_BANKED

////
void interrupt 7 RTI_INT(void) {    
interrupt_count++;

  time++;
  if(time >=100){            //数码管计数
     
      time=0;
      count++;
      LedData[0] = (int)speed/10%10;
      LedData[1] = (int)speed%10;
      LedData[2] = (int)speed/10%10;
      LedData[3] = (int)speed%10;
  }

  PTS = ~(0x01 << LedNum);

  PORTB = LedCode[LedData[LedNum]];                                    
  
  LedNum++;
  if(LedNum >= 4) LedNum = 0;
  CRGFLG |= 0x80; 

  

DisableInterrupts; //关中断
   

speed = PACNT;	//读脉冲累加寄存器
PACNT = 0;	//脉冲累加器置0

   
max=0;

if (count_count>=1)
PWMDTY01=0;

 else{
  
for(i=0;i<=5;i++){
if(max<ad_data[i]){
max=ad_data[i]; a=i;
}
}

                                              
if(max<yuzhi){
Get_Line=0;                  
stop++;
if (stop>500) stop1++;
}
else{
stop=0;
stop1=0;
if(a==0){
a=1;
temp0=ad_data[a+1]-ad_data[a-1]; ////分子 
temp1=temp0<<2; //delta_x=8; //分子 
temp0=ad_data[a]-ad_data[a+1];

temp2=temp0; 
temp0=ad_data[a]-ad_data[a-1]; 
temp2=temp2+temp0;//分母 

if(temp2==0){
Get_Line=0; 
Line_center=0;
}

else{
index=(float)temp1/((float)temp2); Line_center=(a<<3)+index; 

if(Line_center<0||Line_center>4){
Line_center=0;
//Get_Line = 0;
}
else{
Get_Line = 1;
}
}
}

else if(a==5){
a=4;
temp0=ad_data[a+1]-ad_data[a-1]; ////分子 
temp1=temp0<<2; //delta_x=8; //分子
temp0=ad_data[a]-ad_data[a+1];
temp2=temp0; 
temp0=ad_data[a]-ad_data[a-1]; 
temp2=temp2+temp0;//分母 

if(temp2==0){
Get_Line=0; 
Line_center=36;
} 
else{
index=(float)temp1/((float)temp2);
Line_center=(a<<3)+index;

if(Line_center>36||Line_center<33){
Line_center=36;
//Get_Line = 0;
}
else{
Get_Line = 1;
}
}
}

else{

//////////////////////////////////////////////////拟合过程 
temp0=ad_data[a+1]-ad_data[a-1]; ////分子 
temp1=temp0<<2; //delta_x=8; //分子
temp0=ad_data[a]-ad_data[a+1]; 
temp2=temp0; 
temp0=ad_data[a]-ad_data[a-1]; 
temp2=temp2+temp0;//分母


if(temp2==0){
index=0;
}

else{
index=(float)temp1/((float)temp2);
}

Line_center=(a<<3)+index;

if(Line_center<0){
Line_center=0;
//Get_Line = 0;
}

else if(Line_center>36){
Line_center=36;
//Get_Line = 0;
}
else{
Get_Line = 1;
}
}
}



//-----------------------------------积累 //*********************************************************************


if(Get_Line){
for(i=99;i>=1;i--) position[i]=position[i-1]; ///////////////////记录赛道信息
position[0]= (int)Line_center-18; 
pos_30newAdd=position[0];
 
for(i=1;i<30;i++){
pos_30newAdd+=position[i];
}
}

//*********************************************************************赛道状态信息的判断

//*********************************************************************赛道状态信息的判断

if(saidao_state!=chusaidao) {//记录未出赛道时候的状态
pre_state=saidao_state;
}
if(saidao_state==chusaidao){//出赛道

if(Get_Line) //检测到赛道 
saidao_state=pre_state;
}

else if(Get_Line==0){
saidao_state=chusaidao;
chusaidao_pos=Line_center;      //记录出赛道时的位置
}

else if(pre_state==zhidao){	//上次是直道
if((pos_30newAdd>zhidaotowandao)||(pos_30newAdd<-zhidaotowandao)){
saidao_state=zhitowan;
}
else saidao_state=zhidao;
}

else if(pre_state==zhitowan){	//上次是直道到弯道
if((pos_30newAdd>zhidaotowandao2)||(pos_30newAdd<-zhidaotowandao2)){
saidao_state=wandao;  
}
//Control_Speed=50;	//给一定的启动速度
else if((pos_30newAdd>zhidaotowandao)||(pos_30newAdd<-zhidaotowandao)){
saidao_state=zhitowan;
}

else saidao_state=zhidao;
}

else if(pre_state==wandao){    //上次是弯道
if((pos_30newAdd<wandaotozhidao)&&(pos_30newAdd>-wandaotozhidao)){
saidao_state=wantozhi;   //进入缓冲区
}

else saidao_state=wandao;
}

else if(pre_state==wantozhi){
if(zhidaochangdu>zhixian){
saidao_state=zhidao;
zhidaochangdu=0;
}
else{
saidao_state=wantozhi; 
zhidaochangdu+=speed;
}
}
//else if((pos_30newAdd>zhidaotowandao)||(pos_30newAdd<-zhidaotowandao)){
//saidao_state=wandao;zhidaochangdu=0;
//}



//*********************************************************************


if(saidao_state==chusaidao){
Line_center=chusaidao_pos<18?0:36;
}



switch(saidao_state)

{

case(zhidao):	//直道                  

{

Speed_Wanted=zhidaosudu;
if (speed>=zhidaosudu-2)   Speed_Wanted=zhidaosudu-2;

if(Line_center<36.0){
temp0=(int)Line_center;
steer=(Line_center-(float)temp0)*((float)(Steer_PWM_Outputzhi[temp0]-Steer_PWM_Outputzhi[temp0+1]));
steer=(float)Steer_PWM_Outputzhi[temp0]-steer;
}

else steer=-280;
PD_pwm=D_pwm;
pwm=(int)(SteerCenter) - (int)steer;
D_pwm=pwm-spwm[1];

Control_Steer=pwm+(int)(10*D_pwm/dzhizhidao);//+(int)(10*(PD_pwm-D_pwm)/pdzhizhidao); //舵机P控制式，位置式 
}break;
case(wandao):	//弯道

{

Speed_Wanted=wandaosudu+6;//Moto_out[Line_center>>1]; 
if(!wandao_st)wandao_st++;
if(Line_center<36.0){
temp0=(int)Line_center;
steer=(Line_center-(float)temp0)*((float)(Steer_PWM_Outputwan[temp0]-Steer_PWM_Outputwan[temp0+1]));
steer=(float)Steer_PWM_Outputwan[temp0]-steer;
}

else steer=-280;
PD_pwm=D_pwm;
pwm=(int)(SteerCenter) - (int)steer;
D_pwm=pwm-spwm[1];
Control_Steer=pwm+(int)(10*D_pwm/dzhiwandao);//+(int)(10*(PD_pwm-D_pwm)/pdzhiwandao); //舵机Pd控制式，位置式 

}break;

case(zhitowan):	//直道到弯道

{

Speed_Wanted=wandaosudu;
 //if(speed>wandaosudu)
 
//{

//Speed_Wanted=wandaosudu; 
 
//}

if(Line_center<36.0){
temp0=(int)Line_center;
steer=(Line_center-(float)temp0)*((float)(Steer_PWM_Outputzhi[temp0]-Steer_PWM_Outputzhi[temp0+1]));
steer=(float)Steer_PWM_Outputwan[temp0]-steer;
}

else steer=-280;
PD_pwm=D_pwm;
pwm=(int)(SteerCenter) - (int)steer;
D_pwm=pwm-spwm[1];
Control_Steer=pwm+(int)(10*D_pwm/dzhizhitowan);//+(int)(10*(PD_pwm-D_pwm)/pdzhizhitowan); //舵机P控制式，位置式
}break;

case(wantozhi): //弯道到直道
{ Speed_Wanted=wandaosudu+3;//Moto_out[Line_center>>1];

if(Line_center<36.0){
temp0=(int)Line_center;
steer=(Line_center-(float)temp0)*((float)(Steer_PWM_Outputzhi[temp0]-Steer_PWM_Outputzhi[temp0+1]));
steer=(float)Steer_PWM_Outputzhi[temp0]-steer;
}

else steer=-280;
PD_pwm=D_pwm;
pwm=(int)(SteerCenter) - (int)steer;
D_pwm=pwm-spwm[1];
Control_Steer=pwm+(int)(10*D_pwm/dzhiwantozhi);//+(int)(10*(PD_pwm-D_pwm)/pdzhiwantozhi); //舵机P控制式，位置式
}break;

case(chusaidao):  //出赛道

{

Speed_Wanted=wandaosudu+7;//Moto_out[Line_center>>1]; 
/*if(Line_center<36.0){
temp0=(int)Line_center;
steer=(Line_center-(float)temp0)*((float)(Steer_PWM_Outputwan[temp0]-Steer_PWM_Outputwan[temp0+1]));
steer=(float)Steer_PWM_Outputwan[temp0]-steer;
}

else steer=-280;
PD_pwm=D_pwm;
pwm=(int)(SteerCenter) - (int)steer;
D_pwm=pwm-spwm[1];
Control_Steer=pwm+(int)(10*D_pwm/dzhichusaidao);//+(int)(10*(PD_pwm-D_pwm)/pdzhichusaidao); //舵机P控制式，位置式 */
if (max<yuzhi){
if (Control_Steer<1222) Control_Steer=942;
else Control_Steer=1502;
}
}break;

default:break;

}




if(Control_Steer<942)
Control_Steer=942;

else if(Control_Steer>1502)
Control_Steer=1502; 
PWMDTY67 = Control_Steer;
 

for(i=9;i--;i>0) 
spwm[i]=spwm[i-1];
spwm[0]=pwm;



/*if(moto_start==0)

{

//ant_Control_Speed=0; 
Control_Speed++;
Control_Speed++; 
if(Control_Speed>60)moto_start++;

}

else

{*/

pre_speed = speed;
Pre_offset1 = Pre_offset0;	//偏差信号赋值

Pre_offset0 = This_offset;	//偏差信号赋值

This_offset = (Speed_Wanted)-speed;

Pre_Control_Speed = Control_Speed;

//Control_Speed = Pre_Control_Speed + Speed_K*This_offset;//P控制式，增量式

//Control_Speed=Pre_Control_Speed+Speed_K*(This_offset-Pre_offset0)+This_offset+This_offset-Pre_offset0-(Pre_offset0-Pre_offset1);//PI控制式，增量式
Control_Speed=Pre_Control_Speed+Speed_D*(This_offset-Pre_offset0)+Speed_P*This_offset;
// Control_Speed = Pre_Control_Speed + Speed_K*speed+Speed_D*(speed-pre_speed);
if(Control_Speed<0)Control_Speed=0;

else if(Control_Speed>800)Control_Speed=800;

//}

/*if(saidao_state==zhitowan)

{

Control_Speed=0;

}  */

PWMDTY01= Control_Speed; 
 //调试
}
EnableInterrupts;   
 
//开中�

}

void interrupt 22  ATD_Process(void){                //AD转换中断服务程序
      DisableInterrupts;                            //关中断
      ad_data[0] = ATD0DR0L;
      ad_data[3] = ATD0DR1L;
      ad_data[1] = ATD0DR2L;
      ad_data[4] = ATD0DR3L;
      ad_data[2] = ATD0DR4L;
      ad_data[5] = ATD0DR5L;
      ad_data[6] = ATD0DR6L;
      ad_data[7] = ATD0DR7L;
      EnableInterrupts;                            //开中断
}

//interrupt VectorNumber_Vportp void isr_portp(void)
//{      
//    lap_cnt++;
 //   PIFP_PIFP3 = 1;
//}




#pragma CODE_SEG DEFAULT