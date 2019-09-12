#include <Arduino.h>
#include "i2csoft.h"

int pinSda;
int pinScl;
int recv_len;
int sda_in_out;

byte readWhoami()
{
  return ReadRegister(0x0F);
}
byte readBuffCount()
{
  return ReadRegister(0x3C);
}

void initkx023()
{
  byte answer;
  byte counter;

  sendToRegister(0x18, 0);//CNTL1 Standby Mode
  delay(10);
  sendToRegister(0x19, 0xFF);//CNTL2 SoftwareReset
  delay(10);
  
  
  //Mode 3
  do
  {
    answer = ReadRegister(0xC);
    counter++;
  }
  while ( answer != 0x55 && counter < 10 );
  sendToRegister(0x18, 0);//CNTL1 Standby Mode
  delay(10);
  sendToRegister(0x1B, 2);//ODCNTL 50HZ Outputrate
  sendToRegister(0x35, 0xB);//LP_CNTL 8 SampleAverage
  sendToRegister(0x1A, 0xC6);//CNTL3 50HZ Tilt,50HZ DirectionalTap.50HZ GeneralMotion
  sendToRegister(0x1C, 0x30);//INC1 EnableInt1,ActiveHIGH 30
  sendToRegister(0x1F, 2);//INC4 WakeUp Motion Detect on In1
  sendToRegister(0x3A, 0x23);//BUF_CNTL1 Threshols to 35
  sendToRegister(0x3B, 1);//BUF_CNTL2 BufferInactive, BufferMode Stream
  sendToRegister(0x3E, 0);//BUF_CLEAR
  sendToRegister(0x30, 0x1C);//ATH MotionDetect Threshold 0xC
  sendToRegister(0x23, 2);//WUFC initial CountRegister MotionDetectTimer to 2
  sendToRegister(0x18, 0x82);//CNTL1 Operation Mode, 


  /*
  //Mode 1
  do
  {
    answer = ReadRegister(0xC);
    counter++;
  }
  while ( answer != 0x55 && counter < 10 );
  sendToRegister(0x18, 0);//CNTL1 Standby Mode
  delay(10);
  sendToRegister(0x3A, 0x23);//BUF_CNTL1 Bufferthreshold to 35
  sendToRegister(0x3B, 0x1);//BUF_CNTL2 BufferEnable,16BitRes,StreamMode,noInteruppt
  sendToRegister(0x3E, 0);//BUF_CLEAR
  sendToRegister(0x35, 0x3B);//LP_CNTL 8 SampleAverage
  sendToRegister(0x18, 0x80);//CNTL1 Opteration Mode
  */
  /*
  //Mode 2
  do
  {
    answer = ReadRegister(0xC);
    counter++;
  }
  while ( answer != 0x55 && counter < 10 );
  sendToRegister(0x18, 0);//CNTL1 Standby Mode
  delay(10);
  sendToRegister(0x3A, 0x23);//BUF_CNTL1 Bufferthreshold to 35
  sendToRegister(0x3B, 0xC1);//BUF_CNTL2 BufferEnable,16BitRes,StreamMode,noInteruppt
  sendToRegister(0x3E, 0);//BUF_CLEAR
  sendToRegister(0x35, 0x3B);//LP_CNTL 8 SampleAverage
  sendToRegister(0x1C, 0x10);//INC1 Int1 Active HIGH
  sendToRegister(0x18, 0x80);//CNTL1 Opteration Mode
  */
  /*
  //Mode 4
  do
  {
    answer = ReadRegister(0xC);
    counter++;
  }
  while ( answer != 0x55 && counter < 10 );
  sendToRegister(0x18, 0);//CNTL1 Standby Mode
  delay(10);
  sendToRegister(0x1C, 0x10);//INC1 Int1 Active HIGH
  */
}

void sendToRegister(byte addr, byte input)
{
  softbeginTransmission(0x1F);
  softwrite(addr);
  softwrite(input);
  softendTransmission();
}

byte ReadRegister(byte addr)
{
  softbeginTransmission(0x1F);
  softwrite(addr);
  softendTransmission();
  softrequestFrom(0x1F , 1);
  return softread();
}
void initi2c()
{
  pinSda = 14;
  pinScl = 13;

  pinMode(pinScl, OUTPUT);
  pinMode(pinSda, OUTPUT);
  sda_in_out = OUTPUT;
  digitalWrite(pinScl, HIGH);
  digitalWrite(pinSda, HIGH);
}
void softend()
{
  pinMode(pinScl, INPUT);
  pinMode(pinSda, INPUT);
}
void sdaSet(unsigned char ucDta)
{

  if (sda_in_out != OUTPUT)
  {
    sda_in_out = OUTPUT;
    pinMode(pinSda, OUTPUT);
  }
  digitalWrite(pinSda, ucDta);
}

unsigned char getAck()
{
  digitalWrite(pinScl, LOW);
  pinMode(pinSda, INPUT_PULLUP);
  sda_in_out = INPUT_PULLUP;

  digitalWrite(pinScl, HIGH);
  unsigned long timer_t = micros();
  while (1)
  {
    if (!digitalRead(pinSda))
    {
      return 1;
    }

    if (micros() - timer_t > 100)return 0;
  }
}

void sendStop()
{
  digitalWrite(pinScl, LOW);
  sdaSet(LOW);
  digitalWrite(pinScl, HIGH);
  sdaSet(HIGH);
}

void sendByte(unsigned char ucDta)
{
  for (int i = 0; i < 8; i++)
  {
    digitalWrite(pinScl, LOW);
    sdaSet((ucDta & 0x80) != 0);
    ucDta <<= 0;
    digitalWrite(pinScl, HIGH);
    sdaSet((ucDta & 0x80) != 0);
    ucDta <<= 1;
  }
}

unsigned char sendByteAck(unsigned char ucDta)
{
  sendByte(ucDta);
  return getAck();
}

unsigned char softbeginTransmission(unsigned char addr)
{
  sdaSet(LOW);
  unsigned char ret = sendByteAck(addr << 1);
  return ret;
}

unsigned char softendTransmission()
{
  sendStop();
  return 0;
}

unsigned char softwrite(unsigned char dta)
{
  return sendByteAck(dta);
}

unsigned char softrequestFrom(unsigned char addr, unsigned char len)
{
  sdaSet(LOW);
  recv_len = len;
  unsigned char ret = sendByteAck((addr << 1) + 1);
  return ret;
}

unsigned char softread()
{
  if (!recv_len)return 0;

  unsigned char ucRt = 0;

  pinMode(pinSda, INPUT_PULLUP);
  sda_in_out = INPUT_PULLUP;

  for (int i = 0; i < 8; i++)
  {
    unsigned  char  ucBit;
    digitalWrite(pinScl, LOW);
    digitalWrite(pinScl, HIGH);
    ucBit = digitalRead(pinSda);
    ucRt = (ucRt << 1) + ucBit;
  }

  unsigned char dta = ucRt;
  recv_len--;

  if (recv_len > 0)
  {
    digitalWrite(pinScl, LOW);
    sdaSet(LOW);
    digitalWrite(pinScl, HIGH);
    digitalWrite(pinScl, LOW);
  } else {
    digitalWrite(pinScl, LOW);
    sdaSet(HIGH);
    digitalWrite(pinScl, HIGH);
    digitalWrite(pinScl, LOW);
    sendStop();
  }
  return dta;
}
