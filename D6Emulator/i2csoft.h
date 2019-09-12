#ifndef i2csoftCCC
#define i2csoftCCC

#include <stdint.h>
#include <Arduino.h>

#define CMD_SIZE                1                // COMMAND OR REGISTER ADDRESS SIZE
#define BUFF_SIZE               256                // BUFFER SIZE
    
#ifdef __cplusplus
extern "C" {
#endif

void initkx023();
byte readWhoami();
byte readBuffCount();
void sendToRegister(byte addr,byte input);
byte ReadRegister(byte addr);
void initi2c();
void sdaSet(unsigned char ucDta);
unsigned char getAck();
void sendStop();
void sendByte(unsigned char ucDta);
unsigned char sendByteAck(unsigned char ucDta);
unsigned char softbeginTransmission(unsigned char addr);
unsigned char softendTransmission();
unsigned char softwrite(unsigned char dta);
unsigned char softrequestFrom(unsigned char addr, unsigned char len);
unsigned char softread();
void softend();

#ifdef __cplusplus
} // extern "C"
#endif
#endif
