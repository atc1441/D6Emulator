#include <SPI.h>
#include <stdint.h>
#include <BLEPeripheral.h>

#include <nrf_nvic.h>
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>
#include "Adafruit_GFX.h"
#include "SSD1306.h"
#include <TimeLib.h>
#include <nrf.h>
#include "i2csoft.h"

#define wdt_reset() NRF_WDT->RR[0] = WDT_RR_RR_Reload
#define wdt_enable(timeout) \
  NRF_WDT->CONFIG = NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos); \
  NRF_WDT->CRV = (32768*timeout)/1000; \
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  \
  NRF_WDT->TASKS_START = 1

Adafruit_SSD1306 display(128, 32, &SPI, 28, 4, 29);

#define sleepDelay 7000
#define BUTTON_PIN              30
#define refreshRate 100

int menu;
volatile bool buttonPressed = false;
long startbutton;
unsigned long sleepTime, displayRefreshTime;
int timezone;
int steps;
int steps1;
String msgText;
boolean gotoBootloader = false;
boolean vibrationMode;

String bleSymbol = " ";

BLEPeripheral                   blePeripheral           = BLEPeripheral();
BLEService                      batteryLevelService     = BLEService("190A");
BLECharacteristic   TXchar        = BLECharacteristic("0002", BLENotify, 20);
BLECharacteristic   RXchar        = BLECharacteristic("0001", BLEWriteWithoutResponse, 20);

void buttonHandler() {
  buttonPressed = true;
}

void blePeripheralConnectHandler(BLECentral& central) {
  menu = 0;
  bleSymbol = "B";
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  menu = 0;
  bleSymbol = " ";
}

String answer = "";
String tempCmd = "";
int tempLen = 0, tempLen1;
boolean syn;

void characteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  char remoteCharArray[21];
  tempLen1 = characteristic.valueLength();
  tempLen = tempLen + tempLen1;
  memset(remoteCharArray, 0, sizeof(remoteCharArray));
  memcpy(remoteCharArray, characteristic.value(), tempLen1);
  tempCmd = tempCmd + remoteCharArray;
  if (tempCmd[tempLen - 2] == '\r' && tempCmd[tempLen - 1] == '\n') {
    answer = tempCmd.substring(0, tempLen - 2);
    tempCmd = "";
    tempLen = 0;
    filterCmd(answer);
  }
}

void filterCmd(String Command) {
  if (Command == "BT+RESET") {
    sendBLEcmd("BT+RESET:OK");
    delay(100);
    int err_code = sd_power_gpregret_set(0x01);
    sd_nvic_SystemReset();
    while (1) {};
  } else if (Command.substring(0, 7) == "AT+SYN=") {
    sendBLEcmd("AT+SYN:OK");
  }

}

void sendBLEcmd(String Command) {
  Command = Command + "\r\n";
  while (Command.length() > 0) {
    const char* TempSendCmd;
    String TempCommand = Command.substring(0, 20);
    TempSendCmd = &TempCommand[0];
    TXchar.setValue(TempSendCmd);
    Command = Command.substring(20);
  }
}

int getBatteryLevel() {
  return map(analogRead(3), 500, 715, 0, 100);
}

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(3, INPUT);
  if (digitalRead(BUTTON_PIN) == LOW) {
    NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  }
  pinMode(2, INPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(15, INPUT);
  wdt_enable(5000);
  blePeripheral.setLocalName("AcclBLE22");
  blePeripheral.setAdvertisingInterval(500);
  blePeripheral.setAppearance(0x0000);
  blePeripheral.setConnectable(true);
  blePeripheral.setDeviceName("AcclBLE22");
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService.uuid());
  blePeripheral.addAttribute(batteryLevelService);
  blePeripheral.addAttribute(TXchar);
  blePeripheral.addAttribute(RXchar);
  RXchar.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  blePeripheral.begin();
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, FALLING);
  display.begin(SSD1306_SWITCHCAPVCC);
  delay(100);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  digitalWrite(25, LOW);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  initi2c();
  initkx023();
}
long notifyRefreshTime;
void loop() {
  blePeripheral.poll();
  wdt_reset();
  if (millis() - notifyRefreshTime > 10) {
    notifyRefreshTime = millis();
    sendAcclNotify();
  }

  switch (menu) {
    case 0:
      if (millis() - displayRefreshTime > refreshRate) {
        displayRefreshTime = millis();
        displayMenu0();
      }
      break;
    case 1:
      if (millis() - displayRefreshTime > refreshRate) {
        displayRefreshTime = millis();
        displayMenu1();
      }
      break;
    case 2:
      if (millis() - displayRefreshTime > refreshRate) {
        displayRefreshTime = millis();
        displayMenu2();
      }
      break;
    case 3:
      if (millis() - displayRefreshTime > refreshRate) {
        displayRefreshTime = millis();
        displayMenu3();
      }
      break;
    case 4:
      if (millis() - displayRefreshTime > refreshRate) {
        displayRefreshTime = millis();
        displayMenu4();
      }
      break;
  }
  if (buttonPressed) {
    buttonPressed = false;
    switch (menu) {
      case 0:
        menu = 1;
        break;
      case 1:
        menu = 2;
        break;
      case 2:
        menu = 3;
        break;
      case 3:
        startbutton = millis();
        while (!digitalRead(BUTTON_PIN)) {}
        if (millis() - startbutton > 1000) {
          delay(100);
          displayBootloader(0);
          delay(1000);
          sd_nvic_SystemReset();
        } else {
          menu = 4;
        }
        break;
      case 4:
        startbutton = millis();
        while (!digitalRead(BUTTON_PIN)) {}
        if (millis() - startbutton > 1000) {
          delay(100);
          displayBootloader(0);
          delay(1000);
          int err_code = sd_power_gpregret_set(0x01);
          sd_nvic_SystemReset();
        } else {
          menu = 0;
        }
        break;
    }
  }
}


void displayMenu0() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(bleSymbol);
  display.print(" ");
  display.print(getBatteryLevel());
  display.print("%  ");
  display.println(analogRead(3));
  display.display();
}

void displayReboot(int reg) {
  display.setRotation(0);
  display.setCursor(0, 0);
  display.clearDisplay();
  display.println("Going reboot Now");
  display.println(reg);
  display.display();
}
void displayBootloader(int reg) {
  display.setRotation(0);
  display.setCursor(0, 0);
  display.clearDisplay();
  display.println("Going to Bootloader Now");
  display.println(reg);
  display.display();
}

void displayMenu1() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Menue1 Mac:");
  char tmp[16];
  sprintf(tmp, "%04X", NRF_FICR->DEVICEADDR[1] & 0xffff);
  String MyID = tmp;
  sprintf(tmp, "%08X", NRF_FICR->DEVICEADDR[0]);
  MyID += tmp;
  display.println(MyID);
  display.display();
}

void displayMenu2() {
  display.setRotation(0);
  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  res[0] = softread();
  res[1] = softread();
  res[2] = softread();
  res[3] = softread();
  res[4] = softread();
  res[5] = softread();
  byte x = (int16_t)((res[1] << 8) | res[0]) / 128;
  byte y = (int16_t)((res[3] << 8) | res[2]) / 128;
  byte z = (int16_t)((res[5] << 8) | res[4]) / 128;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Menue2 PushMSG:");
  display.println(msgText);
  display.print(x);
  display.print(",");
  display.print(y);
  display.print(",");
  display.println(z);
  display.display();
}

void displayMenu3() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Hello From Arduino");
  display.println("  :)");
  display.println("Hold for Reboot");
  display.display();
}
void displayMenu4() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Hello From Arduino");
  display.println("  :)");
  display.println("Hold for Bootloader");
  display.display();
}

void sendAcclNotify() {
  char res[7];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  res[0] = softread();
  res[1] = softread();
  res[2] = softread();
  res[3] = softread();
  res[4] = softread();
  res[5] = softread();
  res[6] = 23;
  byte x = (int16_t)((res[1] << 8) | res[0]) / 128;
  byte y = (int16_t)((res[3] << 8) | res[2]) / 128;
  byte z = (int16_t)((res[5] << 8) | res[4]) / 128;
  TXchar.setValue(res);
}
