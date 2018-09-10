#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


/////////////////////////////////////////////////////////////////////
//������PLL 80M��BUS 40M
//���岦�뿪�ص������DEMO���ͼ���г�ʼ����ʹ��
//��������ܼ�����ʾ��PWM��ʼ��
//����AD�������ο�ʵ�����̽��г�ʼ����ʹ��
//����ͷ������BUS 80M������ǰ�����̽����޸�  ��Ӧ��PWM��ʼ��Ҳ���޸� ������ջ����������
/////////////////////////////////////////////////////////////////////



#define SteerCenter 1222            //�趨����м�ֵ
#define SteerLeft 942
#define SteerRight 1502
#define ATD_SEQURE 8                  //����ת�����г���
#define Threshold  30                //�趨��ֵ


//������� 
uchar ad_data[ATD_SEQURE];                       //���ADת�����
int stop=0;
int stop1=0;

 
int interrupt_count=0;
uint count_count=0;
int yuzhi=30;
uchar label;
uint max;
int  speed;	//��������ۼӽ��
int  pre_speed;
signed int temp0,temp1,temp2;
float index;
uchar a;
float steer=0;
signed int pos_30newAdd;	//���30�� ��
float  Line_center;                              //������λ��

uchar Get_Line;                                  //ָʾ�˴����޼�⵽����

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
uint zhidaochangdu=0;	//��¼ֱ������

#define zhixian 50	
//************************��������	25   17
#define qidong	0	
#define zhidao	1	
#define wandao	2	
#define wantozhi	3	
#define chusaidao	4	
#define zhitowan	5	
#define end	6	


//*****************************************************************
#define wandaotozhidao 50	//�������ֱ���Ļ���
#define zhidaotowandao 300	//ֱ������Ļ���  //285
#define zhidaotowandao2 600//450 //350
//*****************************************************************
uint zhidaosudu=37;//ֱ���ٶ� 31 34
uint wandaosudu=27;//����ٶ�21 20
uchar moto_start=0;	
uchar wandao_st=0;		


uint  deadline=0;

int  Pre_Control_Speed=0;	
int	Control_Speed=700;	
int  Pre_Control_Steer[10];	
int	D_Steer;	
	
//int	Control_Steer;

//#define Speed_K	35	//�ٶ�Kp
#define Speed_D 250 //180                    //450 400
#define Speed_P	500	//�ٶ�Ki     //450
//#define Speed_D	50	//�ٶ�Ki
	
//int  ant_Control_Speed;	
int	Pre_offset1=0;	
int	Pre_offset0=0;	
int	This_offset=0;	
//*******************************************************����״̬
uchar	saidao_state=zhidao;	
uchar	pre_state=zhidao;	
int Speed_Wanted;	//Ŀ���ٶ�


//*******************************************************

//*******************************************************
 signed char position[100];

float chusaidao_pos;
 
#pragma CODE_SEG DEFAULT


unsigned char Led[] = {0xFB,0xF7,0xDF,0xBF};                        //PE2.3.5.6����ʾ
unsigned char LedCode[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};    //�������ʾ
unsigned char LedData[]={0,0,0,0};
unsigned char LedNum = 0;

int i,j,k; 
int time = 0;
int count = 0;


void initGPIO(void){         //ͨ��IO�ڳ�ʼ��
    DDRA = 0x00;             //A������
    DDRB = 0xFF;             //B�����
    DDRE = 0x00;             //E��2.3.5.6  LED���
    DDRS = 0xFF;
    DDRP = 0x05; 
            //������Ļ�ֻҪ�õ�PTP_PTP0��ΪIO��PP1��ΪPWM�����������ת����  ˫����õ�PTP_PTP2 ��PP3
    //PPSJ_PPSJ7 = 1;//positive edge
    //PIFJ_PIFJ7 = 1;//write 1 to clear interrupt flag 
   // PIEJ_PIEJ7 = 1;//interrupt enable  
}




//
void ATD_init(void){                   //ATD��ʼ��
    ATD0CTL0 = 0x07;                  //��·ת��ʱת��8·
    ATD0CTL1_SRES = 0;           //ת������Ϊ8λ
    ATD0CTL2_AFFC = 1;                //�жϱ�־λ�Զ�����
    ATD0CTL2_ASCIE = 1;               //һ�����д������������ж�
    ATD0CTL3 = 0xC0;                  //����Ĵ������뷽ʽ�Ҷ��룬ת�����г���Ϊ8��ѭ��ת����freezeģʽ�¼���ת��
    //ATD0CTL4 = 0x04;                  //��������Ϊ4�����ڣ�atdclk=busclk/2*(0+1),�����໷����ʱ��Ϊ8M����
    ATD0CTL4 = 0x08;                                 //ATDclk=4M
    ATD0CTL5_SCAN = 1;                //����ת��ģʽ 
    ATD0CTL5_MULT = 1;                //��ͨ������
    }  //
    
void initPLL(void){                 //���໷��ʼ����������Ƶ�ʵ�����40M
    CLKSEL=0x00;                        //��ֹ���໷��ʱ�����ⲿ�����ṩ������Ƶ��=�ⲿ����/2
    PLLCTL_PLLON=1;                     //�����໷
    SYNR=0x49;          
    REFDV=0x43;                         // pllclock=fvco=2*osc*(1+SYNR)/(1+REFDV)=80MHz;
    POSTDIV = 0x00;
    _asm(nop);                           //BUS CLOCK=40M
    _asm(nop);
    while(!(CRGFLG_LOCK==1));            // �ȴ����໷��ʼ�����
    CLKSEL_PLLSEL =1;                    // ʹ�����໷ 
}
void initPWM(void){          
     
     PWMPOL = 0xFF;                      //PWM����ѡ��ѡ��һ�����ڿ�ʼʱΪ�ߵ�ƽ
     PWMPRCLK = 0x22;                    //CLOCK A��Bʱ�ӷ�Ƶ����ѡ��������ķ�Ƶ 10M
     PWMSCLA  = 5;                       //CLOCK SA��CLOCK Aʮ��Ƶ��1M
     PWMSCLB  = 5;                       //CLOCK SB��CLOCK Bʮ��Ƶ��1M
     PWMCTL = 0xF0;                      //01������23������45������67����
     PWMCLK = 0xFF;                      //PWMʼ��ѡ��ѡ��CLOCK SA SBΪPWMʱ��

     PWMPER01=1000;                       //��һ�����Ƶ��Ϊ1k
     PWMDTY01=0;
     
     PWMPER23=1000;
     PWMDTY23=0;                           //�ڶ������Ƶ��Ϊ1k
         
     PWMPER67=20000;                        //���PWMƵ��Ϊ50Hz  
     PWMDTY67=SteerCenter;               
             
     PWME_PWME1=1;                           
     PWME_PWME7=1;                          
     PWME_PWME3=1;                           //PWM��ʼ���
                                                                       
}
void InitRTI(void) {                      // ʵʱ�жϳ�ʼ��                     
    RTICTL = 0xaf;
    //RTICTL = 0xaf;//10msRTICTL = 0x83;                            //0.25ms�ж�
    CRGINT = 0x80;                            // ��ʵʱ�ж�
}
void InitPACNT(void){
    PACTL = 0x40;                        //�����ۼӹ��ܣ��½��ؼ���
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
          PORTE=Led[i%4];                 //E����ˮ��
        }  */
  PORTE = 0xFF;   
/***********�ɻɹܼ��****/

//if(interrupt_count>=300) label=PORTE;
  
  /**************************************���뿪��**/
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

    
  
    PTP_PTP0 = 0;      //��һ·�������    ��ת��  PTP_PTP0 = 1 ����
    //PWMDTY01 = 100;
    
    //PTP_PTP2 = 0;     //�ڶ�·�������  ��ת��  PTP_PTP2 = 1 ����
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
  if(time >=100){            //����ܼ���
     
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

  

DisableInterrupts; //���ж�
   

speed = PACNT;	//�������ۼӼĴ���
PACNT = 0;	//�����ۼ�����0

   
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
temp0=ad_data[a+1]-ad_data[a-1]; ////���� 
temp1=temp0<<2; //delta_x=8; //���� 
temp0=ad_data[a]-ad_data[a+1];

temp2=temp0; 
temp0=ad_data[a]-ad_data[a-1]; 
temp2=temp2+temp0;//��ĸ 

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
temp0=ad_data[a+1]-ad_data[a-1]; ////���� 
temp1=temp0<<2; //delta_x=8; //����
temp0=ad_data[a]-ad_data[a+1];
temp2=temp0; 
temp0=ad_data[a]-ad_data[a-1]; 
temp2=temp2+temp0;//��ĸ 

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

//////////////////////////////////////////////////��Ϲ��� 
temp0=ad_data[a+1]-ad_data[a-1]; ////���� 
temp1=temp0<<2; //delta_x=8; //����
temp0=ad_data[a]-ad_data[a+1]; 
temp2=temp0; 
temp0=ad_data[a]-ad_data[a-1]; 
temp2=temp2+temp0;//��ĸ


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



//-----------------------------------���� //*********************************************************************


if(Get_Line){
for(i=99;i>=1;i--) position[i]=position[i-1]; ///////////////////��¼������Ϣ
position[0]= (int)Line_center-18; 
pos_30newAdd=position[0];
 
for(i=1;i<30;i++){
pos_30newAdd+=position[i];
}
}

//*********************************************************************����״̬��Ϣ���ж�

//*********************************************************************����״̬��Ϣ���ж�

if(saidao_state!=chusaidao) {//��¼δ������ʱ���״̬
pre_state=saidao_state;
}
if(saidao_state==chusaidao){//������

if(Get_Line) //��⵽���� 
saidao_state=pre_state;
}

else if(Get_Line==0){
saidao_state=chusaidao;
chusaidao_pos=Line_center;      //��¼������ʱ��λ��
}

else if(pre_state==zhidao){	//�ϴ���ֱ��
if((pos_30newAdd>zhidaotowandao)||(pos_30newAdd<-zhidaotowandao)){
saidao_state=zhitowan;
}
else saidao_state=zhidao;
}

else if(pre_state==zhitowan){	//�ϴ���ֱ�������
if((pos_30newAdd>zhidaotowandao2)||(pos_30newAdd<-zhidaotowandao2)){
saidao_state=wandao;  
}
//Control_Speed=50;	//��һ���������ٶ�
else if((pos_30newAdd>zhidaotowandao)||(pos_30newAdd<-zhidaotowandao)){
saidao_state=zhitowan;
}

else saidao_state=zhidao;
}

else if(pre_state==wandao){    //�ϴ������
if((pos_30newAdd<wandaotozhidao)&&(pos_30newAdd>-wandaotozhidao)){
saidao_state=wantozhi;   //���뻺����
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

case(zhidao):	//ֱ��                  

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

Control_Steer=pwm+(int)(10*D_pwm/dzhizhidao);//+(int)(10*(PD_pwm-D_pwm)/pdzhizhidao); //���P����ʽ��λ��ʽ 
}break;
case(wandao):	//���

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
Control_Steer=pwm+(int)(10*D_pwm/dzhiwandao);//+(int)(10*(PD_pwm-D_pwm)/pdzhiwandao); //���Pd����ʽ��λ��ʽ 

}break;

case(zhitowan):	//ֱ�������

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
Control_Steer=pwm+(int)(10*D_pwm/dzhizhitowan);//+(int)(10*(PD_pwm-D_pwm)/pdzhizhitowan); //���P����ʽ��λ��ʽ
}break;

case(wantozhi): //�����ֱ��
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
Control_Steer=pwm+(int)(10*D_pwm/dzhiwantozhi);//+(int)(10*(PD_pwm-D_pwm)/pdzhiwantozhi); //���P����ʽ��λ��ʽ
}break;

case(chusaidao):  //������

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
Control_Steer=pwm+(int)(10*D_pwm/dzhichusaidao);//+(int)(10*(PD_pwm-D_pwm)/pdzhichusaidao); //���P����ʽ��λ��ʽ */
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
Pre_offset1 = Pre_offset0;	//ƫ���źŸ�ֵ

Pre_offset0 = This_offset;	//ƫ���źŸ�ֵ

This_offset = (Speed_Wanted)-speed;

Pre_Control_Speed = Control_Speed;

//Control_Speed = Pre_Control_Speed + Speed_K*This_offset;//P����ʽ������ʽ

//Control_Speed=Pre_Control_Speed+Speed_K*(This_offset-Pre_offset0)+This_offset+This_offset-Pre_offset0-(Pre_offset0-Pre_offset1);//PI����ʽ������ʽ
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
 //����
}
EnableInterrupts;   
 
//���ж

}

void interrupt 22  ATD_Process(void){                //ADת���жϷ������
      DisableInterrupts;                            //���ж�
      ad_data[0] = ATD0DR0L;
      ad_data[3] = ATD0DR1L;
      ad_data[1] = ATD0DR2L;
      ad_data[4] = ATD0DR3L;
      ad_data[2] = ATD0DR4L;
      ad_data[5] = ATD0DR5L;
      ad_data[6] = ATD0DR6L;
      ad_data[7] = ATD0DR7L;
      EnableInterrupts;                            //���ж�
}

//interrupt VectorNumber_Vportp void isr_portp(void)
//{      
//    lap_cnt++;
 //   PIFP_PIFP3 = 1;
//}




#pragma CODE_SEG DEFAULT