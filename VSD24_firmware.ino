#include <EEPROM.h>//

#define U8      unsigned char
#define U16     unsigned int
#define U32     unsigned long
#define U2      byte

#define PWM_001  ((volatile unsigned long *) (0x422101a0)) 
#define PWM_002  ((volatile unsigned long *) (0x422201a0)) 
#define PWM_003  ((volatile unsigned long *) (0x4222019c)) 
#define PWM_004  ((volatile unsigned long *) (0x42220198)) 
#define PWM_005  ((volatile unsigned long *) (0x422181bc))  
#define PWM_006  ((volatile unsigned long *) (0x422181b8))  
#define PWM_007  ((volatile unsigned long *) (0x422181b4))  
#define PWM_008  ((volatile unsigned long *) (0x422181b0))  
#define PWM_009  ((volatile unsigned long *) (0x42218184))
#define PWM_010  ((volatile unsigned long *) (0x42218180))
#define PWM_011  ((volatile unsigned long *) (0x42220194)) 
#define PWM_012  ((volatile unsigned long *) (0x42220190)) 
#define PWM_013  ((volatile unsigned long *) (0x42220188))  
#define PWM_014  ((volatile unsigned long *) (0x42220184)) 
#define PWM_015  ((volatile unsigned long *) (0x42220180))  
#define PWM_016  ((volatile unsigned long *) (0x422201bc)) 
#define PWM_017  ((volatile unsigned long *) (0x422201b8)) 
#define PWM_018  ((volatile unsigned long *) (0x422201b4))  
#define PWM_019  ((volatile unsigned long *) (0x422181a4))  
#define PWM_020  ((volatile unsigned long *) (0x422181a0))   
#define PWM_021  ((volatile unsigned long *) (0x4221819c))  
#define PWM_022  ((volatile unsigned long *) (0x42218198))  
#define PWM_023  ((volatile unsigned long *) (0x42218194))  
#define PWM_024  ((volatile unsigned long *) (0x42228188))

#define PWM_01  6
#define PWM_02  37
#define PWM_03  36
#define PWM_04  35
#define PWM_05  34
#define PWM_06  33
#define PWM_07  32
#define PWM_08  31
#define PWM_09  28
#define PWM_10  27
#define PWM_11  20
#define PWM_12  19
#define PWM_13  17
#define PWM_14  16
#define PWM_15  15
#define PWM_16  23
#define PWM_17  22
#define PWM_18  21
#define PWM_19  24
#define PWM_20  14
#define PWM_21  9
#define PWM_22  5
#define PWM_23  4
#define PWM_24  25

#define TIMERPERIOD 500  
#define IO_RATE   500  

#define D3ASTATE  3  
#define D2BSTATE  2  
#define D18STATE  18 

#define USbTxType1     1
#define UARt1TxType2   2
#define UARt2TxType3   3

U8  GpioRcSetReg  = 0x00;
U8  GpioRcSetReg1 = 0x00;	
U8  GpioRcSetReg2 = 0x00;	

U16  PWMangle[30]  = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

U16  PWMspeed[30]  = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
char  UsbTempBuffer[400];
U2   UartRxNotFinishFlig = 0;
U16  UsbRxCount = 0;
int data002;

U8  msec5count;                       
U8  msec10count;                      
U8  msec100count;                     
U8  msec300count;                     
U8  msec500count;                     
U8  sec1count;                        
U8  sec5count;                        
U8  sec10count;  
U8  uartrxnotfinishcount;
U16 EepromServoStopTimesCount = 0;
//定时器4中断标志位
U2 MSEC1   = 0;
U2 MSEC5   = 0;          
U2 MSEC10  = 0;    
U2 MSEC100 = 0;     
U2 MSEC300 = 0;  
U2 MSEC500 = 0;       
U2 SEC1    = 0;       
U2 SEC5    = 0;        
U2 SEC10   = 0;
U2 EepromServoStopTimesON1OFF0Flag = 1;
U16 EepromServoRunTimes = 0;


U8 DIStateBps;        
U8 DIStateBpsApp;      
U2 On1Off0LineFlag = 0;

U2 USbRxFLAG   = 0;
U2 UARt1RxFLAG = 0;
U2 UARt2RxFLAG = 0;

U2 ActionCompleteFlag = 0;    
U2 UartFunctionON1OFF0 = 1; 
U2 EepromServoRunStateON1OFF0Flag = 0;
U16 EepromServoRunNumber = 1;

struct PWM_DATA					
{
  U16 	buff;	  
  U16 	angle;	  	
  U16 	value;	  
  U16   speed;    
  U16   timer;    
  U2    SpeedFlag;
  U2    TimerFlag;
  U2    PulseOoffsetFlag;
  U2    DigitalOutFlag;
  U2    DigitalOutStatus;
  U2    BytseOutFlag;
  U16   BytseOutrRegister;
};
struct PWM_DATA PWM01[30];

U32 ArrayNumberPWM01;
U16 UsbTemp = 0;

#define LoopExecutionFlagAddr             0x0001
#define ActionGroupNumberAddr             0x0002
#define PwmDataFirstCommonalityAddr       0x000A

U2         EepromStatusFlag = 1;
uint16     LoopExecutionFlag;      
uint16 	   ActionGroupNumber;	 
uint16 	   ActionGroupNumberNow;  
uint16 	   ServoPwm[24];
uint16 	   ServoSpeed[24];
uint16     ExecutionTime = 255;  
uint16     ServoRxActionGroupNumber;
uint16     ServoRxRegister[50];     

uint16  Status;
uint16  Data;


void led_kongzhi(void)
{
  toggleLED();
}
void pwm_io_init(void)
{

  pinMode(PWM_01, OUTPUT); 
  pinMode(PWM_02, OUTPUT); 
  pinMode(PWM_03, OUTPUT); 
  pinMode(PWM_04, OUTPUT); 
  pinMode(PWM_05, OUTPUT); 
  pinMode(PWM_06, OUTPUT); 
  pinMode(PWM_07, OUTPUT); 
  pinMode(PWM_08, OUTPUT); 
  pinMode(PWM_09, OUTPUT); 
  pinMode(PWM_10, OUTPUT); 
  pinMode(PWM_11, OUTPUT); 
  pinMode(PWM_12, OUTPUT); 
  pinMode(PWM_13, OUTPUT); 
  pinMode(PWM_14, OUTPUT); 
  pinMode(PWM_15, OUTPUT); 
  pinMode(PWM_16, OUTPUT); 
  pinMode(PWM_17, OUTPUT); 
  pinMode(PWM_18, OUTPUT); 
  pinMode(PWM_19, OUTPUT); 
  pinMode(PWM_20, OUTPUT); 
  pinMode(PWM_21, OUTPUT); 
  pinMode(PWM_22, OUTPUT); 
  pinMode(PWM_23, OUTPUT); 
  pinMode(PWM_24, OUTPUT);

  pinMode(D3ASTATE,INPUT_PULLUP);
  pinMode(D2BSTATE,INPUT_PULLUP);
  pinMode(D18STATE,INPUT_PULLUP); 
  
  Serial1.begin(9600); 
  Serial2.begin(9600);
}

HardwareTimer timer1(1);
HardwareTimer timer2(2);
HardwareTimer timer3(3);
HardwareTimer timer4(4);
void time_init(void)
{
  timer1.pause();
  timer2.pause();
  timer3.pause();
  timer4.pause();

  timer1.setPeriod(50); // 
  timer2.setPeriod(50); // 
  timer3.setPeriod(50); // 
  timer4.setPeriod(TIMERPERIOD); // 

  timer1.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer2.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer3.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer4.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  
  timer1.setCompare(TIMER_CH1,1);  //
  timer2.setCompare(TIMER_CH1,1);  //
  timer3.setCompare(TIMER_CH1,1);  //
  timer4.setCompare(TIMER_CH1,1);  //
  
  timer1.attachCompare1Interrupt(handler01);
  timer2.attachCompare1Interrupt(handler02);
  timer3.attachCompare1Interrupt(handler03);
  timer4.attachCompare1Interrupt(handler04);

  timer1.refresh();
  timer2.refresh();
  timer3.refresh();
  timer4.refresh();
  

  timer1.resume();
  timer2.resume();
  timer3.resume();
  timer4.resume();
  
}
void TimerReinstallInit(U16 ReinsTimer)
{
  timer1.setPeriod(ReinsTimer); // 
}
void Timer2ReinstallInit(U16 ReinsTimer2)
{
  timer2.setPeriod(ReinsTimer2); // 
}
void Timer3ReinstallInit(U16 ReinsTimer3)
{
  timer3.setPeriod(ReinsTimer3); // 
}

void setup(void) 
{
    pinMode(BOARD_LED_PIN, OUTPUT);
    pwm_io_init();//
    time_init();
    EepromFormatIntConfiguration();
    Status = EEPROM.init();
}

void loop(void) 
{
  Msec1_task();//
  Msec10();//
  LineOn1Off0();
  CheckIOState();//
  EepromRunProgram();
  
}

void Msec10(void)
{
  if(MSEC10)
  {
    MSEC10 = 0;
  }
}

int my_atoi(char* pstr,U8 number)  
{  
  int Ret_Integer = 0;  
  int Integer_sign = 1;  
  int PointerNumber = 1;//
  if(pstr == NULL)  
  {  
    return ' '; 
  }   

  if(*pstr == '-')  
  {  
    Integer_sign = -1;  
  }  
  if(*pstr == '-' || *pstr == '+')  
  {  
    pstr++;  
    PointerNumber++;
  }   

  while((*pstr >= '0' && *pstr <= '9')) 
  {  
     if(number >= PointerNumber) 
        Ret_Integer = Ret_Integer * 10 + *pstr - '0';  
      pstr++; 
      PointerNumber++;
  } 
  Ret_Integer = Integer_sign * Ret_Integer;   
  return Ret_Integer;  
} 
int JudgeNumericalRange(int NUMER,int BigRANGE,int SmallRANGE)
{
  if( NUMER >=  BigRANGE)
    NUMER = BigRANGE;
  else if( NUMER <= SmallRANGE )
    NUMER = SmallRANGE;  
  else
    NUMER = NUMER;
  return  NUMER; 
}

void UartTxData(U8 txtype,U8 txorder)
{
  switch(txtype) 
  {
    case USbTxType1:
      switch(txorder) 
      {
        case 1:
          SerialUSB.println("DF-USBSSC24-V0.1 ");
        break;
        case 2:
          SerialUSB.print(ArrayNumberPWM01,DEC); 
          SerialUSB.print(":"); 
          SerialUSB.print(IO_RATE + PWM01[ArrayNumberPWM01 - 1].buff,DEC); 
          SerialUSB.println("us"); 
        break;
        case 3:
          SerialUSB.println('+'); 
        break;
        case 4:
          SerialUSB.println('.');
        break;
        case 5:
          SerialUSB.println("EEPROM WRITE OK."); 
        break;
        case 6:
          SerialUSB.println("EEPROM WRITE NO!"); 
        break;
        case 7:
          SerialUSB.print("ER:");
          for(U8 i = 0;i < 50;i++)
          {
            SerialUSB.print(ServoRxRegister[i],DEC);  
            if(i<49)
              SerialUSB.print(","); 
          }
          SerialUSB.println(";"); 
        break;
        default:
        break;
      }
    break;
    case UARt1TxType2:
      switch(txorder) 
      {
        case 1:
          Serial1.println("DF-USBSSC24-V0.1 ");
        break;
        case 2:
          Serial1.print(ArrayNumberPWM01,DEC); 
          Serial1.print(":"); 
          Serial1.print(IO_RATE + PWM01[ArrayNumberPWM01 - 1].buff,DEC); 
          Serial1.println("us"); 
        break;
        case 3:
          Serial1.println('+'); 
        break;
        case 4:
          Serial1.println('.');
        break;
        case 5:
          Serial1.println("EEPROM WRITE OK."); 
        break;
        case 6:
          Serial1.println("EEPROM WRITE NO!"); 
        break;
        case 7:
          Serial1.print("ER:");
          for(U8 i = 0;i < 50;i++)
          {
            Serial1.print(ServoRxRegister[i],DEC);  
            if(i<49)
              Serial1.print(","); 
          }
          Serial1.println(";"); 
        break;
        default:
        break;
      }
    break;
    case UARt2TxType3:
      switch(txorder) 
      {
        case 1:
          Serial2.println("DF-USBSSC24-V0.1 ");
        break;
        case 2:
          Serial2.print(ArrayNumberPWM01,DEC); 
          Serial2.print(":"); 
          Serial2.print(IO_RATE + PWM01[ArrayNumberPWM01 - 1].buff,DEC); 
          Serial2.println("us"); 
        break;
        case 3:
          Serial2.println('+'); 
        break;
        case 4:
          Serial2.println('.');
        break;
        case 5:
          Serial2.println("EEPROM WRITE OK."); 
        break;
        case 6:
          Serial2.println("EEPROM WRITE NO!"); 
        break;
        case 7:
          Serial2.print("ER:");
          for(U8 i = 0;i < 50;i++)
          {
            Serial2.print(ServoRxRegister[i],DEC);  
            if(i<49)
              Serial2.print(","); 
          }
          Serial2.println(";"); 
        break;
        default:
        break;
      }
    break;
    default:
    break;
  }
}
U8 UartTxType(void)
{
  U8  TxTypeTemp = 0;
  if(USbRxFLAG)
  {
    TxTypeTemp = USbTxType1;
  }
  else if(UARt1RxFLAG)
  {
    TxTypeTemp = UARt1TxType2;
  }
  else if(UARt2RxFLAG)
  {
    TxTypeTemp = UARt2TxType3;
  }
  else
  {
    TxTypeTemp = 0;
  }
  return TxTypeTemp;
}
void ParseRxData(void)
{
  U16 j,k,i = 0;
  U16 iTemp;
  U16 DataTempCounter  = 0; 
  U2  SportsStatusFlag = 0;
  char DataTemp[10];
  U8  TxTypeTemp1 = 0;
  U16 RxEepromAddr;



  if(((UARt1RxFLAG == 1)||(USbRxFLAG == 1)||(UARt2RxFLAG == 1))&&(UartRxNotFinishFlig == 0))
  { 

    {
      if((UsbTemp > 0) && (UsbTempBuffer[UsbTemp-1] == 13))
      { 
        TxTypeTemp1 = UartTxType();

        while(UsbTemp > i)
        {
          switch(UsbTempBuffer[i]) 
          {

            case 'X'://
              DisplayPages(0x440);//
              DisplayPagesEnd(0x10);//
              i = ++iTemp;
            break;
            case 'E'://
              iTemp = i;     
            
              if(UsbTempBuffer[++iTemp] == 'W' && iTemp < UsbTemp)//
              {//
                ++iTemp;
                if((UsbTempBuffer[iTemp]=='L'||UsbTempBuffer[iTemp]=='N') && iTemp < UsbTemp)//
                {//
                  if( UsbTempBuffer[iTemp]=='L' )
                    LoopExecutionFlag = 1;//
                  else
                    LoopExecutionFlag = 0;// 
                  if(UsbTempBuffer[++iTemp] != ',')//
                    return;    
                  while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))
                  {
                    DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;//
                    DataTempCounter++;
                  }
                  ActionGroupNumber = (U16)JudgeNumericalRange(my_atoi(DataTemp,DataTempCounter),1024,1);//
                  DataTempCounter = 0;
                  EepromFormatInt(); //
                  EepromWriteStatusJudge(LoopExecutionFlagAddr, LoopExecutionFlag);// 
                  EepromWriteStatusJudge(ActionGroupNumberAddr, ActionGroupNumber);// 
                  Status = EEPROM.read(ActionGroupNumberAddr, &ActionGroupNumber);
                  EepromStatusInform(TxTypeTemp1);
                }
                else
                {
                  for( U8 i = 0;i < 50;i++ )
                  {//
                    while((iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))//( UsbTempBuffer[iTemp] != ',')&&
                    {
                      DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;//UsbTempBuffer[iTemp] - '0';
                      DataTempCounter++;
                      ++iTemp;
                    }
                    if(i == 0)//
                    {
                      ActionGroupNumberNow = (U16)JudgeNumericalRange(my_atoi(DataTemp,DataTempCounter),1024,1);
                      ++iTemp;
                      k = ((ActionGroupNumberNow - 1)*50 + PwmDataFirstCommonalityAddr + i);//
                      EepromWriteStatusJudge(((ActionGroupNumberNow - 1)*50 + PwmDataFirstCommonalityAddr + i),ActionGroupNumberNow);//
                    }
                    else if(i>=1 && i<= 24)//
                    {
                      ServoPwm[i-1] = (U16)JudgeNumericalRange(my_atoi(DataTemp,DataTempCounter),2500,500);
                      ++iTemp;
                      EepromWriteStatusJudge(((ActionGroupNumberNow - 1)*50 + PwmDataFirstCommonalityAddr+i),ServoPwm[i-1]);//
                    }
                    else if(i>=25 && i<= 48)//
                    {
                      ServoSpeed[i-25] = (U16)JudgeNumericalRange(my_atoi(DataTemp,DataTempCounter),5000,1);
                      ++iTemp;
                      EepromWriteStatusJudge(((ActionGroupNumberNow - 1)*50 + PwmDataFirstCommonalityAddr+i),ServoSpeed[i-25]);//
                    }
                    else if(i == 49)
                    {
                      ExecutionTime = (U16)JudgeNumericalRange(my_atoi(DataTemp,DataTempCounter),30000,50);
                      ++iTemp;
                      EepromWriteStatusJudge(((ActionGroupNumberNow - 1)*50 + PwmDataFirstCommonalityAddr+i),ExecutionTime);//
                    }
                    else
                    {
                      ++iTemp;
                    }
                    DataTempCounter = 0;
                  }
                  EepromStatusInform(TxTypeTemp1);//
                }
              }             
              else if(UsbTempBuffer[iTemp] == 'R' && iTemp < UsbTemp)//
              { 
                Status = EEPROM.read(ActionGroupNumberAddr,&Data);
                ServoRxActionGroupNumber = Data;
                for(U16 jj = 1;jj <=  ServoRxActionGroupNumber;jj++)//
                {
                  for(U8 kk = 0;kk < 50;kk++)//
                  {
                    RxEepromAddr = ((jj - 1) * 50) + 10 + kk;//
                    Status = EEPROM.read(RxEepromAddr,&ServoRxRegister[kk]);
                  }
                  UartTxData(TxTypeTemp1,7);//
                }
              }
              i = ++iTemp;
            break;

            case 'V':
               iTemp = i;
               if(UsbTempBuffer[++iTemp] == 'E' && UsbTempBuffer[++iTemp] == 'R')
               {
                 UartTxData(TxTypeTemp1,1);//
               }
               else
               {
                 return;
               }
               i = iTemp;
            break;
            case '#':
               iTemp = i;
               while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))
               {
                 DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;
                 DataTempCounter++;
               }
               ArrayNumberPWM01 = my_atoi(DataTemp,DataTempCounter);
               ArrayNumberPWM01 = ArrayNumberPWM01 + 1;
               if(( ArrayNumberPWM01 > 24)||( ArrayNumberPWM01 < 1 ))
               {
                 DataTempCounter = 0;
                 i = iTemp;
                 return;
               }  
               DataTempCounter = 0;
               i = iTemp;
            break;
            case 'P':
               iTemp = i;

               PWM01[ArrayNumberPWM01 - 1].DigitalOutFlag = 0;//
               PWM01[ArrayNumberPWM01].BytseOutFlag = 0;

               OpenPulseFunction(ArrayNumberPWM01 - 1);
               if(UsbTempBuffer[iTemp+1] == 'O')
               {
                 iTemp++;
                 PWM01[ArrayNumberPWM01 - 1].PulseOoffsetFlag = 1;
               }
               while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&((UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9')||(UsbTempBuffer[iTemp] == '-')||(UsbTempBuffer[iTemp] == '+')))
               {
                 DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;
                 DataTempCounter++;
               }
               if(PWM01[ArrayNumberPWM01 - 1].PulseOoffsetFlag)
               {
                 PWMspeed[ArrayNumberPWM01 - 1] = my_atoi(DataTemp,DataTempCounter);
                 PWMspeed[ArrayNumberPWM01 - 1] = JudgeNumericalRange(PWMspeed[ArrayNumberPWM01 - 1],100,-100);
               }
               else
               {
                 PWMspeed[ArrayNumberPWM01 - 1] = my_atoi(DataTemp,DataTempCounter);
                 PWMspeed[ArrayNumberPWM01 - 1] = JudgeNumericalRange(PWMspeed[ArrayNumberPWM01 - 1],2500,500);
                 PWMspeed[ArrayNumberPWM01 - 1] = PWMspeed[ArrayNumberPWM01 - 1] - 500;
               }
               DataTempCounter = 0;
               i = iTemp;
            break;
            case 'S':
               iTemp = i;
               while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))
               {
                 DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;
                 DataTempCounter++;
               }
               PWM01[ArrayNumberPWM01 - 1].SpeedFlag = 1;
               PWM01[ArrayNumberPWM01 - 1].speed = my_atoi(DataTemp,DataTempCounter);//atoi
               PWM01[ArrayNumberPWM01 - 1].speed = JudgeNumericalRange(PWM01[ArrayNumberPWM01 - 1].speed,5000,1);//
               DataTempCounter = 0;
               i = iTemp;
            break;
            case 'T':
               iTemp = i;
               while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))
               {
                 DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;
                 DataTempCounter++;
               }
               PWM01[ArrayNumberPWM01 - 1].TimerFlag = 1;
               PWM01[ArrayNumberPWM01 - 1].timer = my_atoi(DataTemp,DataTempCounter);//atoi
               PWM01[ArrayNumberPWM01 - 1].timer = JudgeNumericalRange(PWM01[ArrayNumberPWM01 - 1].timer,65535,1);//
               for(j = 0;j<24;j++)
               {
                 PWM01[j].timer     = PWM01[ArrayNumberPWM01 - 1].timer;
                 PWM01[j].TimerFlag = PWM01[ArrayNumberPWM01 - 1].TimerFlag;
                 PWM01[j].SpeedFlag = 0;
               }
               DataTempCounter = 0;
               i = iTemp;
            break; 
            case 'H'://
               iTemp = i;
               PWM01[ArrayNumberPWM01 - 1].DigitalOutFlag = 1;//
               PWM01[ArrayNumberPWM01 - 1].DigitalOutStatus = 1;//
               i = ++iTemp;
            break;//
            case 'L'://
               iTemp = i;
               PWM01[ArrayNumberPWM01 - 1].DigitalOutFlag = 1;//
               PWM01[ArrayNumberPWM01 - 1].DigitalOutStatus = 0;//
               i = ++iTemp;
            break;
            case ':'://
               iTemp = i;
               //
               while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))
               {
                 DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;
                 DataTempCounter++;
               }
               PWM01[ArrayNumberPWM01].BytseOutrRegister = my_atoi(DataTemp,DataTempCounter);//
               switch(ArrayNumberPWM01) 
               {
                 case 1://
                   if(PWM01[ArrayNumberPWM01].BytseOutrRegister>=0 && PWM01[ArrayNumberPWM01].BytseOutrRegister<=255)
                   {
                     PWM01[ArrayNumberPWM01].BytseOutFlag = 1;
                     GpioRcSetReg = 0;
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0001) *PWM_001 = 1;//_H; 
                     else                    *PWM_001 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0002) *PWM_002 = 1;//_L;
                     else                    *PWM_002 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0004) *PWM_003 = 1;//_L;
                     else                    *PWM_003 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0008) *PWM_004 = 1;//_L;
                     else                    *PWM_004 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0010) *PWM_005 = 1;//_L;
                     else                    *PWM_005 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0020) *PWM_006 = 1;//_L;
                     else                    *PWM_006 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0040) *PWM_007 = 1;//_L;
                     else                    *PWM_007 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0080) *PWM_008 = 1;//_L;	
                     else                    *PWM_008 = 0;//PWM1_H; 
                   }
                   else
                   {
                     PWM01[ArrayNumberPWM01 - 1].DigitalOutFlag = 0;
                     PWM01[ArrayNumberPWM01].BytseOutFlag = 0;
                     *PWM_001 = 0;//PWM1_H; 
                     *PWM_002 = 0;//PWM1_H; 
                     *PWM_003 = 0;//PWM1_H; 
                     *PWM_004 = 0;//PWM1_H; 
                     *PWM_005 = 0;//PWM1_H; 
                     *PWM_006 = 0;//PWM1_H; 
                     *PWM_007 = 0;//PWM1_H; 
                     *PWM_008 = 0;//PWM1_H; 
                   }
                 break;
                 case 2://
                   if(PWM01[ArrayNumberPWM01].BytseOutrRegister>=0 && PWM01[ArrayNumberPWM01].BytseOutrRegister<=255)
                   {
                     PWM01[ArrayNumberPWM01].BytseOutFlag = 1;
                     GpioRcSetReg1 = 0;
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0001) *PWM_009 = 1;//PWM1_H; 
                     else                    *PWM_009 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0002) *PWM_010 = 1;//PWM2_L;
                     else                    *PWM_010 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0004) *PWM_011 = 1;//PWM3_L;
                     else                    *PWM_011 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0008) *PWM_012 = 1;//PWM4_L;
                     else                    *PWM_012 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0010) *PWM_013 = 1;//PWM5_L;
                     else                    *PWM_013 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0020) *PWM_014 = 1;//PWM6_L;
                     else                    *PWM_014 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0040) *PWM_015 = 1;//PWM7_L;
                     else                    *PWM_015 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0080) *PWM_016 = 1;//PWM8_L;	
                     else                    *PWM_016 = 0;//PWM1_H; 
                   }
                   else
                   {
                     PWM01[ArrayNumberPWM01 - 1].DigitalOutFlag = 0;
                     PWM01[ArrayNumberPWM01].BytseOutFlag = 0;
                     *PWM_009 = 0;//PWM1_H; 
                     *PWM_010 = 0;//PWM1_H; 
                     *PWM_011 = 0;//PWM1_H; 
                     *PWM_012 = 0;//PWM1_H; 
                     *PWM_013 = 0;//PWM1_H; 
                     *PWM_014 = 0;//PWM1_H; 
                     *PWM_015 = 0;//PWM1_H; 
                     *PWM_016 = 0;//PWM1_H; 
                   }
                 break;
                 case 3://
                   if(PWM01[ArrayNumberPWM01].BytseOutrRegister>=0 && PWM01[ArrayNumberPWM01].BytseOutrRegister<=255)
                   {
                     PWM01[ArrayNumberPWM01].BytseOutFlag = 1;
                     GpioRcSetReg2 = 0;
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0001) *PWM_017 = 1;//PWM1_H; 
                     else                    *PWM_017 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0002) *PWM_018 = 1;//PWM2_L;
                     else                    *PWM_018 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0004) *PWM_019 = 1;//PWM3_L;
                     else                    *PWM_019 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0008) *PWM_020 = 1;//PWM4_L;
                     else                    *PWM_020 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0010) *PWM_021 = 1;//PWM5_L;
                     else                    *PWM_021 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0020) *PWM_022 = 1;//PWM6_L;
                     else                    *PWM_022 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0040) *PWM_023 = 1;//PWM7_L;
                     else                    *PWM_023 = 0;//PWM1_H; 
                     if(PWM01[ArrayNumberPWM01].BytseOutrRegister & 0x0080) *PWM_024 = 1;//PWM8_L;	
                     else                    *PWM_024 = 0;//PWM1_H; 
                   }
                   else
                   {
                     PWM01[ArrayNumberPWM01 - 1].DigitalOutFlag = 0;
                     PWM01[ArrayNumberPWM01].BytseOutFlag = 0;
                     *PWM_017 = 0;//_H; 
                     *PWM_018 = 0;//_H; 
                     *PWM_019 = 0;//_H; 
                     *PWM_020 = 0;//_H; 
                     *PWM_021 = 0;//_H; 
                     *PWM_022 = 0;//_H; 
                     *PWM_023 = 0;//_H; 
                     *PWM_024 = 0;//_H; 
                   }
                 break;
                 default:
                 break;
               }
               i = iTemp;
            break;
            case 'Q'://
               iTemp = i;     
               if(UsbTempBuffer[++iTemp] == 'P' && iTemp < UsbTemp)
               {
                 while(( UsbTempBuffer[++iTemp] != ' ')&&(iTemp < UsbTemp)&&(UsbTempBuffer[iTemp]>='0')&&(UsbTempBuffer[iTemp]<='9'))
                 {
                   DataTemp[DataTempCounter] = UsbTempBuffer[iTemp] ;//
                   DataTempCounter++;
                 }
                 ArrayNumberPWM01 = my_atoi(DataTemp,DataTempCounter);//
                 ArrayNumberPWM01 = (U8)JudgeNumericalRange(ArrayNumberPWM01,24,1);//
                 DataTempCounter = 0;
                 SportsStatusFlag = 1;
                 //
                 UartTxData(TxTypeTemp1,2);//
               }
               else
               {             
                 for( j= 0;j<24;j++ )
                 {
                   if(PWM01[j].angle != PWM01[j].buff)
                   {// 
                     SportsStatusFlag = 1;
                   }
                 }
                 if(SportsStatusFlag)
                 {//'.'
                   UartTxData(TxTypeTemp1,3);//
                 }
                 else
                 {//'.'
                   UartTxData(TxTypeTemp1,4);//
                 }
               }
               i = ++iTemp;
            break;
            default:
              if(i == 0)
              {
                SportsStatusFlag = 1;
              }
              i++;
            break;
          } 
        }
        if(SportsStatusFlag==0)//
          Sys_PWMDataAccount();//
        else
          SportsStatusFlag = 0; 
      }
    }
    USbRxFLAG   = 0;//
    UARt1RxFLAG = 0;//
    UARt2RxFLAG = 0;//
    UsbTemp = 0; 
  }
}

void ParseRxDataOnLine(void)
{
  U16 j,k,i = 0;
  U16 iTemp;
  U16 DataTempCounter  = 0; //
  U2  SportsStatusFlag = 0;//  
  char DataTemp[10];
  U8  TxTypeTemp1 = 0;
  U16 RxEepromAddr;
  if(((UARt1RxFLAG == 1)||(USbRxFLAG == 1)||(UARt2RxFLAG == 1))&&(UartRxNotFinishFlig == 0))
  { 
    { 
      TxTypeTemp1 = UartTxType();
      if((UsbTemp > 0) && (UsbTempBuffer[UsbTemp-1] == 13))
      {
        while(UsbTemp > i)
        {
          TxTypeTemp1 = UartTxType();
          switch(UsbTempBuffer[i]) 
          {
            case 'O'://
              iTemp = i;
              ++iTemp;
              if(UsbTempBuffer[iTemp] == 'F')
              {//
                UartFunctionON1OFF0 = 0;// 
              }
              else if(UsbTempBuffer[iTemp] == 'N')
              {//
                UartFunctionON1OFF0 = 1;// //
              }
              i = ++iTemp;
            break;
            default:
              i++;
            break;
          }
        }
      }
    }  
    USbRxFLAG   = 0;//
    UARt1RxFLAG = 0;//
    UARt2RxFLAG = 0;//
    UsbTemp = 0; 
  }
}

void LineOn1Off0(void)
{
  if(On1Off0LineFlag == 0)  
  {
    ParseRxData();
  }
  else
  {
    ParseRxDataOnLine();
  }
}

void usbrxbuf(void)
{ 
  U16 i = 0;
  int newbytes;
  if(UARt1RxFLAG == 0 && UARt2RxFLAG == 0)
  {
    newbytes = SerialUSB.available();
    if ( !newbytes )
    { 
      USbRxFLAG = 0;
      return;   
    }
    if(uartrxnotfinishcount >= 40)//
    {
      UsbTemp = 0;
    } 
    UartRxNotFinishFlig = 0;
    uartrxnotfinishcount = 0;  
    while((SerialUSB.available())&&( UsbTemp+i < 390 ))
    {
      UsbTempBuffer[UsbTemp+i] = SerialUSB.read();
      i++;
      delayMicroseconds(100); 
      led_kongzhi();
    }
    digitalWrite(BOARD_LED_PIN, LOW);
    UsbTemp = UsbTemp + i; 
    if((UsbTempBuffer[UsbTemp-1] != 13))
    {
      if( UsbTemp < 390 )
      {
        UartRxNotFinishFlig = 1;
      }
      else
      {
        UartRxNotFinishFlig = 0;
        UsbTemp = 0;
        uartrxnotfinishcount = 0; 
      }
    }
    else
      UartRxNotFinishFlig = 0; 
    USbRxFLAG = 1; 
  } 
}

void uart1rxbuf(void)
{ 
  U16 i = 0;
  int newbytes;
  if(USbRxFLAG == 0 && UARt2RxFLAG == 0)
  {
    newbytes = Serial1.available();
    if (( !newbytes )|| (-1 == newbytes))
    { 
      UARt1RxFLAG = 0;
      return;  
    }
    if(uartrxnotfinishcount >= 40)
    {
      UsbTemp = 0;
    }  
    UartRxNotFinishFlig = 0;
    uartrxnotfinishcount = 0; 
    while(( Serial1.available() )&&( UsbTemp+i < 390 )) 
    {
      UsbTempBuffer[UsbTemp+i] = Serial1.read();
      i++;
      delayMicroseconds(100); 
    }
    UsbTemp = UsbTemp + i; 
    if((UsbTempBuffer[UsbTemp-1] != 13))
    {
      if( UsbTemp < 390 )
      {
        UartRxNotFinishFlig = 1;
      }
      else
      {
        UartRxNotFinishFlig = 0;
        UsbTemp = 0;
        uartrxnotfinishcount = 0; 
      }
    }
    UARt1RxFLAG = 1;  
  }  
}

void uart2rxbuf(void)
{ 
  U16 i = 0;
  int newbytes;
  if(USbRxFLAG == 0 && UARt1RxFLAG == 0)
  {
    newbytes = Serial2.available();
    if (( !newbytes ))
    { 
      UARt2RxFLAG = 0;
      return;  
    }
    if(uartrxnotfinishcount >= 40)//
    {
      UsbTemp = 0;
    }  
    UartRxNotFinishFlig = 0;
    uartrxnotfinishcount = 0; 
    while(( Serial2.available() )&&( UsbTemp+i < 390 )) 
    {
      UsbTempBuffer[UsbTemp+i] = Serial2.read();
      i++;
      delayMicroseconds(100); 
    }
    UsbTemp = UsbTemp + i; 
    if((UsbTempBuffer[UsbTemp-1] != 13))
    {
      if( UsbTemp < 390 )
      {
        UartRxNotFinishFlig = 1;//
      }
      else
      {
        UartRxNotFinishFlig = 0;
        UsbTemp = 0;
        uartrxnotfinishcount = 0; 
      }
    }
    UARt2RxFLAG = 1;  
  }  
}

void PWMDataCount(struct PWM_DATA *PWMData_point)
{
  if(PWMData_point->buff == PWMData_point->angle)	
    return;	
  else if(PWMData_point->buff < PWMData_point->angle)
  {
    if((PWMData_point->angle - PWMData_point->buff) <= PWMData_point->value)	
      PWMData_point->buff = PWMData_point->angle;
    else
      PWMData_point->buff += 	PWMData_point->value;
  }
  else
  {
    if((PWMData_point->buff - PWMData_point->angle) <= PWMData_point->value)	
      PWMData_point->buff = PWMData_point->angle;
    else
      PWMData_point->buff -= 	PWMData_point->value;
  }
}

void Sys_PWMDataAccount(void)
{
  U8 i,j;
  int PWMspeedTemp;
  for(i=0;i<24;i++)
  {
    if(PWM01[i].DigitalOutFlag == 0)
    {
      if((PWM01[i].SpeedFlag == 0)&&(PWM01[i].TimerFlag == 0))
      {
        if(PWM01[i].PulseOoffsetFlag)
        {
          PWM01[i].PulseOoffsetFlag = 0;
          PWMspeedTemp = PWM01[i].angle + PWMspeed[i];
          PWM01[i].angle = JudgeNumericalRange(PWMspeedTemp,2000,0);
          PWM01[i].value = 100;
        }
        else
        {
          PWM01[i].buff  = PWMspeed[i];//当前脉冲
          PWM01[i].angle = PWMspeed[i];//最终到达脉冲
        }
        continue;
      }
      if(PWM01[i].SpeedFlag == 1)
      {
        PWM01[i].SpeedFlag = 0;
        if(PWMspeed[i] == PWM01[i].buff)	
        {	
          PWM01[i].angle = PWMspeed[i];
          continue;	
        }  
        PWMangle[i] = PWM01[i].speed / 50;
        PWM01[i].value = JudgeNumericalRange(PWMangle[i],65535,1);
        PWM01[i].angle = PWMspeed[i];
      }
      if(PWM01[i].TimerFlag == 1)
      {
        PWM01[i].TimerFlag = 0;
        if(PWMspeed[i] > PWM01[i].buff)
          PWM01[i].speed = (((PWMspeed[i]-PWM01[i].buff))*1000/(PWM01[i].timer));//*1000
        if((PWMspeed[i] < PWM01[i].buff))  
          PWM01[i].speed = (((PWM01[i].buff-PWMspeed[i]))*1000/(PWM01[i].timer));//*1000
        if(PWMspeed[i] == PWM01[i].buff)
        {
          PWM01[i].angle = PWMspeed[i];		
          continue;	
        }  
        PWMangle[i] = PWM01[i].speed / 50; 
        
        PWM01[i].value = JudgeNumericalRange(PWMangle[i],65535,1);
        PWM01[i].angle = PWMspeed[i];
      }
    }
    else
    { 
      ClosePulseFunction(i,PWM01[i].DigitalOutStatus); 
    }
  }
}

void ClosePulseFunction(U8 number,U2 status)
{
  switch(number) 
  {
    case 0:
      GpioRcSetReg = GpioRcSetReg & 0xFE;
      *PWM_001 = status;
    break;
    case 1:
      GpioRcSetReg = GpioRcSetReg & 0xFD;
      *PWM_002 = status;
    break;
    case 2:
      GpioRcSetReg = GpioRcSetReg & 0xFB;
      *PWM_003 = status;
    break;
    case 3:
      GpioRcSetReg = GpioRcSetReg & 0xF7;
      *PWM_004 = status;
    break;
    case 4:
      GpioRcSetReg = GpioRcSetReg & 0xEF;
      *PWM_005 = status;
    break;
    case 5:
      GpioRcSetReg = GpioRcSetReg & 0xDF;
      *PWM_006 = status;
    break;
    case 6:
      GpioRcSetReg = GpioRcSetReg & 0xBF;
      *PWM_007 = status;
    break;
    case 7:
      GpioRcSetReg = GpioRcSetReg & 0x7F;
      *PWM_008 = status;
    break;
    case 8:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xFE;
      *PWM_009 = status;
    break;
    case 9:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xFD;
      *PWM_010 = status;
    break;
    case 10:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xFB;
      *PWM_011 = status;
    break;
    case 11:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xF7;
      *PWM_012 = status;
    break;
    case 12:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xEF;
      *PWM_013 = status;
    break;
    case 13:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xDF;
      *PWM_014 = status;
    break;
    case 14:
      GpioRcSetReg1 = GpioRcSetReg1 & 0xBF;
      *PWM_015 = status;
    break;
    case 15:
      GpioRcSetReg1 = GpioRcSetReg1 & 0x7F;
      *PWM_016 = status;
    break;
    case 16:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xFE;
      *PWM_017 = status;
    break;
    case 17:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xFD;
      *PWM_018 = status;
    break;
    case 18:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xFB;
      *PWM_019 = status;
    break;
    case 19:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xF7;
      *PWM_020 = status;
    break;
    case 20:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xEF;
      *PWM_021 = status;
    break;
    case 21:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xDF;
      *PWM_022 = status;
    break;
    case 22:
      GpioRcSetReg2 = GpioRcSetReg2 & 0xBF;
      *PWM_023 = status;
    break;
    case 23:
      GpioRcSetReg2 = GpioRcSetReg2 & 0x7F;
      *PWM_024 = status;
    break;
  }  
}

void OpenPulseFunction(U8 number1)
{
  switch(number1) 
  {
    case 0:
      GpioRcSetReg = GpioRcSetReg | 0x01;
    break;
    case 1:
      GpioRcSetReg = GpioRcSetReg | 0x02;
    break;
    case 2:
      GpioRcSetReg = GpioRcSetReg | 0x04;
    break;
    case 3:
      GpioRcSetReg = GpioRcSetReg | 0x08;
    break;
    case 4:
      GpioRcSetReg = GpioRcSetReg | 0x10;
    break;
    case 5:
      GpioRcSetReg = GpioRcSetReg | 0x20;
    break;
    case 6:
      GpioRcSetReg = GpioRcSetReg | 0x40;
    break;
    case 7:
      GpioRcSetReg = GpioRcSetReg | 0x80;
    break;
    case 8:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x01;
    break;
    case 9:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x02;
    break;
    case 10:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x04;
    break;
    case 11:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x08;
    break;
    case 12:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x10;
    break;
    case 13:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x20;
    break;
    case 14:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x40;
    break;
    case 15:
      GpioRcSetReg1 = GpioRcSetReg1 | 0x80;
    break;
    case 16:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x01;
    break;
    case 17:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x02;
    break;
    case 18:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x04;
    break;
    case 19:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x08;
    break;
    case 20:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x10;
    break;
    case 21:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x20;
    break;
    case 22:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x40;
    break;
    case 23:
      GpioRcSetReg2 = GpioRcSetReg2 | 0x80;
    break;
  }  
}

void handler01(void) 
{ 
  U8 i = 0;
  U16 TH;
  static U8	PWM_CH = 0;		
  {
    switch(PWM_CH)
    {
      case 0:	
        if(GpioRcSetReg & 0x01)	*PWM_001 = 0;//_H; 
        if(GpioRcSetReg & 0x02)	*PWM_002 = 0;//_L;
        if(GpioRcSetReg & 0x04)	*PWM_003 = 0;//_L;
        if(GpioRcSetReg & 0x08)	*PWM_004 = 0;//_L;
        if(GpioRcSetReg & 0x10)	*PWM_005 = 0;//_L;
        if(GpioRcSetReg & 0x20)	*PWM_006 = 0;//_L;
        if(GpioRcSetReg & 0x40)	*PWM_007 = 0;//_L;
        if(GpioRcSetReg & 0x80)	*PWM_008 = 0;//_L;	
        PWM_CH++;					
        PWMDataCount(&PWM01[0]);
        TimerReinstallInit(IO_RATE + PWM01[0].buff);
      break;
      case 1:	
        if(GpioRcSetReg & 0x80)	*PWM_008 = 0;//_L;
	if(GpioRcSetReg & 0x01)	*PWM_001 = 1;//_H;
        PWM_CH++;	
        PWMDataCount(&PWM01[1]);
        TimerReinstallInit(IO_RATE + PWM01[1].buff);
      break;
      case 2:	
        if(GpioRcSetReg & 0x01)	*PWM_001 = 0;//_L;
  	if(GpioRcSetReg & 0x02)	*PWM_002 = 1;//_H;
	PWM_CH++;	
        PWMDataCount(&PWM01[2]);
        TimerReinstallInit(IO_RATE + PWM01[2].buff);
      break;
      case 3:	
        if(GpioRcSetReg & 0x02)	*PWM_002 = 0;//_L;
	if(GpioRcSetReg & 0x04)	*PWM_003 = 1;//_H;
        PWM_CH++;	
	PWMDataCount(&PWM01[3]);
  	TimerReinstallInit(IO_RATE + PWM01[3].buff);//
      break;	
      case 4:	
        if(GpioRcSetReg & 0x04)	*PWM_003 = 0;//_L;
	if(GpioRcSetReg & 0x08)	*PWM_004 = 1;//_H;
	PWM_CH++;
        PWMDataCount(&PWM01[4]);
	TimerReinstallInit(IO_RATE + PWM01[4].buff);
      break;
      case 5:	
        if(GpioRcSetReg & 0x08)	*PWM_004 = 0;//_L;
	if(GpioRcSetReg & 0x10)	*PWM_005 = 1;//_H;
	PWM_CH++;					
        PWMDataCount(&PWM01[5]);
        TimerReinstallInit(IO_RATE + PWM01[5].buff);
      break;
      case 6:	
        if(GpioRcSetReg & 0x10)	*PWM_005 = 0;//_L;
	if(GpioRcSetReg & 0x20)	*PWM_006 = 1;//_H;
	PWM_CH++;
        PWMDataCount(&PWM01[6]);
        TimerReinstallInit(IO_RATE + PWM01[6].buff);
      break;			
      case 7:	
        if(GpioRcSetReg & 0x20)	*PWM_006 = 0;
	if(GpioRcSetReg & 0x40)	*PWM_007 = 1;
        PWM_CH++;
	PWMDataCount(&PWM01[7]);
        TimerReinstallInit(IO_RATE + PWM01[7].buff);
      break;				
      case 8: 	
        if(GpioRcSetReg & 0x01)	*PWM_001 = 0;
        if(GpioRcSetReg & 0x02)	*PWM_002 = 0;
        if(GpioRcSetReg & 0x04)	*PWM_003 = 0;
        if(GpioRcSetReg & 0x08)	*PWM_004 = 0;
        if(GpioRcSetReg & 0x10)	*PWM_005 = 0;//_L;
        if(GpioRcSetReg & 0x20)	*PWM_006 = 0;//_L;
        if(GpioRcSetReg & 0x40)	*PWM_007 = 0;//_L;
        if(GpioRcSetReg & 0x80)	*PWM_008 = 1;//_L;
        PWM_CH = 0;
        TH = 0;
        for(i=0;i<8;i++)	
          TH += PWM01[i].buff;
        TimerReinstallInit(20000 -  TH - 4000);
      break;
      default :	
      break;
    }
  } 	 	 
}

void handler02(void) 
{ 
  U8 i = 0;
  U16 TH;		
  static  U8  PWM_CH2 = 0;		
  {
    switch(PWM_CH2)
    {
      case 0:	
        if(GpioRcSetReg1 & 0x01)	*PWM_009 = 0;//_H;
        if(GpioRcSetReg1 & 0x02)	*PWM_010 = 0;//_L;
        if(GpioRcSetReg1 & 0x04)	*PWM_011 = 0;//_L;
        if(GpioRcSetReg1 & 0x08)	*PWM_012 = 0;//_L;
        if(GpioRcSetReg1 & 0x10)	*PWM_013 = 0;//_L;
        if(GpioRcSetReg1 & 0x20)	*PWM_014 = 0;//_L;
        if(GpioRcSetReg1 & 0x40)	*PWM_015 = 0;//_L;
        if(GpioRcSetReg1 & 0x80)	*PWM_016 = 0;//_L;	
        PWM_CH2++;					
        PWMDataCount(&PWM01[8]);
        Timer2ReinstallInit(IO_RATE + PWM01[8].buff);
      break;
      case 1:	
        if(GpioRcSetReg1 & 0x80)	*PWM_016 = 0;//_L;
	if(GpioRcSetReg1 & 0x01)	*PWM_009 = 1;//_H;
        PWM_CH2++;	
        PWMDataCount(&PWM01[9]);
        Timer2ReinstallInit(IO_RATE + PWM01[9].buff);
      break;
      case 2:	
        if(GpioRcSetReg1 & 0x01)	*PWM_009 = 0;//_L;
  	if(GpioRcSetReg1 & 0x02)	*PWM_010 = 1;//_H;
	PWM_CH2++;	
        PWMDataCount(&PWM01[10]);
        Timer2ReinstallInit(IO_RATE + PWM01[10].buff);
      break;
      case 3:	
        if(GpioRcSetReg1 & 0x02)	*PWM_010 = 0;//_L;
	if(GpioRcSetReg1 & 0x04)	*PWM_011 = 1;//_H;
        PWM_CH2++;	
	PWMDataCount(&PWM01[11]);
  	Timer2ReinstallInit(IO_RATE + PWM01[11].buff);
      break;	
      case 4:	
        if(GpioRcSetReg1 & 0x04)	*PWM_011 = 0;
	if(GpioRcSetReg1 & 0x08)	*PWM_012 = 1;
	PWM_CH2++;
        PWMDataCount(&PWM01[12]);
	Timer2ReinstallInit(IO_RATE + PWM01[12].buff);
      break;
      case 5:	
        if(GpioRcSetReg1 & 0x08)	*PWM_012 = 0;
	if(GpioRcSetReg1 & 0x10)	*PWM_013 = 1;
	PWM_CH2++;					
        PWMDataCount(&PWM01[13]);
        Timer2ReinstallInit(IO_RATE + PWM01[13].buff);
      break;
      case 6:	
        if(GpioRcSetReg1 & 0x10)	*PWM_013 = 0;
	if(GpioRcSetReg1 & 0x20)	*PWM_014 = 1;
	PWM_CH2++;
        PWMDataCount(&PWM01[14]);
        Timer2ReinstallInit(IO_RATE + PWM01[14].buff);
      break;			
      case 7:	
        if(GpioRcSetReg1 & 0x20)	*PWM_014 = 0;
	if(GpioRcSetReg1 & 0x40)	*PWM_015 = 1;
        PWM_CH2++;
	PWMDataCount(&PWM01[15]);
        Timer2ReinstallInit(IO_RATE + PWM01[15].buff);
      break;				
      case 8: 	
        if(GpioRcSetReg1 & 0x01)	*PWM_009 = 0;//_L;
        if(GpioRcSetReg1 & 0x02)	*PWM_010 = 0;//_L;
        if(GpioRcSetReg1 & 0x04)	*PWM_011 = 0;//_L;
        if(GpioRcSetReg1 & 0x08)	*PWM_012 = 0;//_L;
        if(GpioRcSetReg1 & 0x10)	*PWM_013 = 0;//_L;
        if(GpioRcSetReg1 & 0x20)	*PWM_014 = 0;//_L;
        if(GpioRcSetReg1 & 0x40)	*PWM_015 = 0;//_L;
        if(GpioRcSetReg1 & 0x80)	*PWM_016 = 1;//_L;
        TH = 0;
        PWM_CH2 = 0;
        for(i=8;i<16;i++)	
          TH += PWM01[i].buff;
        Timer2ReinstallInit(20000 -  TH - 4000);
      break;
      default :	
      break;
    }
  } 	 	 
}
void handler03(void) 
{ 
  U8 i = 0;
  U16 TH;		
  static  U8  PWM_CH3 = 0;		
  {
    switch(PWM_CH3)
    {
      case 0:	
        if(GpioRcSetReg2 & 0x01)	*PWM_017 = 0;//
        if(GpioRcSetReg2 & 0x02)	*PWM_018 = 0;//
        if(GpioRcSetReg2 & 0x04)	*PWM_019 = 0;//
        if(GpioRcSetReg2 & 0x08)	*PWM_020 = 0;
        if(GpioRcSetReg2 & 0x10)	*PWM_021 = 0;
        if(GpioRcSetReg2 & 0x20)	*PWM_022 = 0;
        if(GpioRcSetReg2 & 0x40)	*PWM_023 = 0;
        if(GpioRcSetReg2 & 0x80)	*PWM_024 = 0;
        PWM_CH3++;					
        PWMDataCount(&PWM01[16]);
        Timer3ReinstallInit(IO_RATE + PWM01[16].buff);
      break;
      case 1:	
        if(GpioRcSetReg2 & 0x80)	*PWM_024 = 0;
	if(GpioRcSetReg2 & 0x01)	*PWM_017 = 1;
        PWM_CH3++;	
        PWMDataCount(&PWM01[17]);
        Timer3ReinstallInit(IO_RATE + PWM01[17].buff);
      break;
      case 2:	
        if(GpioRcSetReg2 & 0x01)	*PWM_017 = 0;
  	if(GpioRcSetReg2 & 0x02)	*PWM_018 = 1;
	PWM_CH3++;	
        PWMDataCount(&PWM01[18]);
        Timer3ReinstallInit(IO_RATE + PWM01[18].buff);
      break;
      case 3:	
        if(GpioRcSetReg2 & 0x02)	*PWM_018 = 0;//PWM3_L;
	if(GpioRcSetReg2 & 0x04)	*PWM_019 = 1;//PWM4_H;
        PWM_CH3++;	
	PWMDataCount(&PWM01[19]);
  	Timer3ReinstallInit(IO_RATE + PWM01[19].buff);
      break;	
      case 4:	
        if(GpioRcSetReg2 & 0x04)	*PWM_019 = 0;//_L;
	if(GpioRcSetReg2 & 0x08)	*PWM_020 = 1;//_H;
	PWM_CH3++;
        PWMDataCount(&PWM01[20]);
	Timer3ReinstallInit(IO_RATE + PWM01[20].buff);
      break;
      case 5:	
        if(GpioRcSetReg2 & 0x08)	*PWM_020 = 0;
	if(GpioRcSetReg2 & 0x10)	*PWM_021 = 1;
	PWM_CH3++;					
        PWMDataCount(&PWM01[21]);
        Timer3ReinstallInit(IO_RATE + PWM01[21].buff);
      break;
      case 6:	
        if(GpioRcSetReg2 & 0x10)	*PWM_021 = 0;
	if(GpioRcSetReg2 & 0x20)	*PWM_022 = 1;
	PWM_CH3++;
        PWMDataCount(&PWM01[22]);
        Timer3ReinstallInit(IO_RATE + PWM01[22].buff);
      break;			
      case 7:	
        if(GpioRcSetReg2 & 0x20)	*PWM_022 = 0;
	if(GpioRcSetReg2 & 0x40)	*PWM_023 = 1;
        PWM_CH3++;
	PWMDataCount(&PWM01[23]);
        Timer3ReinstallInit(IO_RATE + PWM01[23].buff);
      break;				
      case 8: 	
        if(GpioRcSetReg2 & 0x01)	*PWM_017 = 0;//_L;
        if(GpioRcSetReg2 & 0x02)	*PWM_018 = 0;//_L;
        if(GpioRcSetReg2 & 0x04)	*PWM_019 = 0;//_L;
        if(GpioRcSetReg2 & 0x08)	*PWM_020 = 0;//_L;
        if(GpioRcSetReg2 & 0x10)	*PWM_021 = 0;//_L;
        if(GpioRcSetReg2 & 0x20)	*PWM_022 = 0;//_L;
        if(GpioRcSetReg2 & 0x40)	*PWM_023 = 0;//_L;
        if(GpioRcSetReg2 & 0x80)	*PWM_024 = 1;//_L;
        TH = 0;
        PWM_CH3 = 0;
        for(i=17;i<24;i++)	
          TH += PWM01[i].buff;
        Timer3ReinstallInit(20000 -  TH - 4000);
      break;
      default :	
      break;
    }
  }
}

void handler04(void) 
{ 
  MSEC1 = 1;
  if(EepromServoStopTimesCount)
    --EepromServoStopTimesCount;
  msec5count++;
  if(UartRxNotFinishFlig)
  {
    uartrxnotfinishcount++;
    if(uartrxnotfinishcount >= 50)
      uartrxnotfinishcount = 50;
  }
  if(msec5count >= 10)
  {
    msec5count = 0;
    MSEC5 = 1;
    msec10count++;
  }
  if(msec10count >= 2)
  {
    msec10count = 0;
    MSEC10 = 1;
    msec100count++;
  }
  if(msec100count >= 10)
  {
    msec100count = 0;
    MSEC100 = 1;
    msec500count++;
  }
  if(msec500count >= 5)
  {
    msec500count = 0;
    MSEC500 = 1;
    sec1count++;
  }
  if(sec1count >= 2)//1s
  {
    sec1count = 0;
    SEC1 = 1;
    sec5count++;
  }
  if(sec5count >= 5)
  {
    sec5count = 0;
    SEC5 = 1;
    sec10count++;
  }
  if(sec10count >= 2)
  {
    sec10count = 0;
    SEC10 = 1;
  }
  if(MSEC500)
  {
    MSEC500 = 0;
  }
}

void CheckIOState(void)
{
  U8 TempState[3];
  if( SEC1 ) 
  {
    SEC1 = 0;
    TempState[0] = digitalRead(D3ASTATE);//
    TempState[1] = digitalRead(D2BSTATE);
    TempState[2] = digitalRead(D18STATE);
    if( TempState[0] == HIGH && TempState[1] == HIGH)//
    {
      DIStateBpsApp = 1;//
    }  
    else if( TempState[0] == HIGH && TempState[1] == LOW)// 
    {
      DIStateBpsApp = 2;// 
    }
    else if( TempState[0] == LOW && TempState[1] == HIGH)//
    {
      DIStateBpsApp = 3;//
    }
    else if( TempState[0] == LOW && TempState[1] == LOW)//
    {
      DIStateBpsApp = 4;//
    }
    if(On1Off0LineFlag == TempState[2])
    {
      if( TempState[2] == LOW )
      {
        On1Off0LineFlag = 1;//
        UartFunctionON1OFF0 = 1;
      }
      else
      {
        On1Off0LineFlag  = 0;//
      }
      UartFunctionON1OFF0 = 1;
    }
    BpsSwitch();
  }  
}

void BpsSwitch(void)
{
  if( DIStateBpsApp != DIStateBps )//原始波特率不等于当前波特率
  {
    DIStateBps = DIStateBpsApp;
    switch(DIStateBps)//改变当前波特率
    {
      case 1://
        Serial1.begin(2400);//
        Serial2.begin(2400);
      break;
      case 2://
        Serial1.begin(9600);
        Serial2.begin(9600);
      break;
      case 3://38400
        Serial1.begin(38400);
        Serial2.begin(38400);
      break;
      case 4://115200
        Serial1.begin(115200);
        Serial2.begin(115200);
      break;
      default :	
      break;
    }
  }
}
void EepromFormatIntConfiguration(void) 
{
  //配置设定范围
  EEPROM.PageBase0 = 0x806F000;
  EEPROM.PageBase1 = 0x806F800;
  EEPROM.PageSize  = 0x800;
}

void EepromFormatInt(void) 
{
  EepromFormatIntConfiguration();
  Status = EEPROM.format();
  Status = EEPROM.init();
}

void DisplayConfig(void)
{
	SerialUSB.print  ("EEPROM.PageBase0 : 0x");
	SerialUSB.println(EEPROM.PageBase0, HEX);
	SerialUSB.print  ("EEPROM.PageBase1 : 0x");
	SerialUSB.println(EEPROM.PageBase1, HEX);
	SerialUSB.print  ("EEPROM.PageSize  : 0x");
	SerialUSB.print  (EEPROM.PageSize, HEX);
	SerialUSB.print  (" (");
	SerialUSB.print  (EEPROM.PageSize, DEC);
	SerialUSB.println(")");
}

void DisplayHex(uint16 value)
{
	if (value <= 0xF)
		SerialUSB.print("000");
	else if (value <= 0xFF)
		SerialUSB.print("00");
	else if (value <= 0xFFF)
		SerialUSB.print("0");
	SerialUSB.print(value, HEX);
}

void DisplayPages(uint32 endIndex)
{
	SerialUSB.println("Page 0     Top         Page 1");

	for (uint32 idx = 0; idx < endIndex; idx += 4)
	{
		SerialUSB.print  (EEPROM.PageBase0 + idx, HEX);
		SerialUSB.print  (" : ");
		DisplayHex(*(uint16*)(EEPROM.PageBase0 + idx));
		SerialUSB.print  (" ");
		DisplayHex(*(uint16*)(EEPROM.PageBase0 + idx + 2));
		SerialUSB.print  ("    ");
		SerialUSB.print  (EEPROM.PageBase1 + idx, HEX);
		SerialUSB.print  (" : ");
		DisplayHex(*(uint16*)(EEPROM.PageBase1 + idx));
		SerialUSB.print  (" ");
		DisplayHex(*(uint16*)(EEPROM.PageBase1 + idx + 2));
		SerialUSB.println();
	}
}

void DisplayPagesEnd(uint32 endIndex)
{
	SerialUSB.println("Page 0     Bottom      Page 1");

	for (uint32 idx = EEPROM.PageSize - endIndex; idx < EEPROM.PageSize; idx += 4)
	{
		SerialUSB.print  (EEPROM.PageBase0 + idx, HEX);
		SerialUSB.print  (" : ");
		DisplayHex(*(uint16*)(EEPROM.PageBase0 + idx));
		SerialUSB.print  (" ");
		DisplayHex(*(uint16*)(EEPROM.PageBase0 + idx + 2));
		SerialUSB.print  ("    ");
		SerialUSB.print  (EEPROM.PageBase1 + idx, HEX);
		SerialUSB.print  (" : ");
		DisplayHex(*(uint16*)(EEPROM.PageBase1 + idx));
		SerialUSB.print  (" ");
		DisplayHex(*(uint16*)(EEPROM.PageBase1 + idx + 2));
		SerialUSB.println();
	}
}

void EepromWriteStatusJudge(uint16 addr,uint16 data) 
{
  Status = EEPROM.write(addr, data);
  if((Status == 0)&&(EepromStatusFlag))
    EepromStatusFlag = 1;
  else
    EepromStatusFlag = 0;  
}

void EepromStatusInform(U8 txtype1)
{
  if(EepromStatusFlag)
    UartTxData(txtype1,5);//
  else
    UartTxData(txtype1,6);
  EepromStatusFlag = 1;//  
}

void EepromRunProgram(void)
{
  uint16 ServoLinage;
  uint16 ArrayNumberPwm;
  uint16 ArrayNumberSpeed;
  uint16 RxEepromAddr; 
  U2  SportsStatusFlag = 0;
  if((On1Off0LineFlag == 1))
  {
    if(UartFunctionON1OFF0 == 1)
    {
      Status = EEPROM.read(ActionGroupNumberAddr,&ServoLinage);
      Status = EEPROM.read(LoopExecutionFlagAddr,&LoopExecutionFlag);
      if((Status == 0) && ( ServoLinage > 0 && ServoLinage < 0xFFFF ))
      {
        if((ServoLinage>= EepromServoRunNumber)&&(EepromServoRunStateON1OFF0Flag)&&((EepromServoStopTimesCount == 0)))//
        {
          for(U8 kk = 0;kk < 50;kk++)
          {
            RxEepromAddr = ((EepromServoRunNumber - 1) * 50) + 10 + kk;
            Status = EEPROM.read(RxEepromAddr,&ServoRxRegister[kk]);
          }
          for(U8 kk = 1;kk < 25;kk++)
          {
            ArrayNumberPwm = kk - 1;
            PWMspeed[ArrayNumberPwm] = ServoRxRegister[kk];
            PWMspeed[ArrayNumberPwm] = JudgeNumericalRange(PWMspeed[ArrayNumberPwm],2500,500);
            PWMspeed[ArrayNumberPwm] = PWMspeed[ArrayNumberPwm] - 500;
          }
          for(U8 kk = 25;kk < 49;kk++)
          {
            ArrayNumberSpeed = kk - 25;
            PWM01[ArrayNumberSpeed].SpeedFlag = 1;
            PWM01[ArrayNumberSpeed].speed = ServoRxRegister[kk];
            PWM01[ArrayNumberSpeed].speed = JudgeNumericalRange(PWM01[ArrayNumberSpeed].speed,5000,1);
          }
          EepromServoRunTimes = JudgeNumericalRange(ServoRxRegister[49],30000,50);
          EepromServoStopTimesCount = EepromServoRunTimes * 2;
          if(!((GpioRcSetReg == 0xFF)&&(GpioRcSetReg1 == 0xFF)&&(GpioRcSetReg2 == 0xFF)))
          {
            GpioRcSetReg  = 0xFF;	//
            GpioRcSetReg1 = 0xFF;	//
            GpioRcSetReg2 = 0xFF;	//
          }
          Sys_PWMDataAccount();//
          EepromServoRunNumber++;
        }
        else if((ServoLinage< EepromServoRunNumber))
        {
          if(LoopExecutionFlag)
          {
            UartFunctionON1OFF0 = 1;
            EepromServoRunNumber = 1;
          }
          else
          {
            UartFunctionON1OFF0 = 0;
          } 
        }
      }
      EepromServoRunState();
    }
  }
  else
  {// 
    for(U8 kk = 0;kk<24;kk++)
      PWM01[ArrayNumberSpeed].SpeedFlag = 0;
  }  
} 

void EepromServoRunState(void) 
{
  for(U8 j= 0;j<24;j++ )
  {//
    if(PWM01[j].angle != PWM01[j].buff)
    {
      EepromServoRunStateON1OFF0Flag = 0;//
      return;
    }
    else
    {
      EepromServoRunStateON1OFF0Flag = 1;
    }    
  }
  
  if((EepromServoStopTimesCount == 0)&&(EepromServoRunStateON1OFF0Flag == 1))
  {
    EepromServoStopTimesCount = EepromServoRunTimes * 2;//
    EepromServoRunStateON1OFF0Flag = 1;
  }  
}
void  Msec1_task(void)
{
  if( MSEC1)
  {
    MSEC1 = 0;
    usbrxbuf();  //
    uart1rxbuf();//
    uart2rxbuf();//
  }
}




