#include <BLE.h>
#include <BLEBoard.h>
#include <BLEEventHandling.h>
#include <BLELog.h>
#include <BLESerial.h>
#include <BLEServiceList.h>
#include <BLEServices.h>
#include <BLETypes.h>
#include <Board.h>
#include <String.h>

// Buttons are active low so initial state is 1
uint16_t button1State = 1;
char final_value[] = "ON";
char final_value2[] = "OFF";

/* Pin number variables for Buttons on MSP432P401R LaunchPad */
uint8_t button1Pin = 73;

BLE_Char button1Char =
{
  {0x01, 0xFF},
   BLE_READABLE,
  "Button 1 State"
};

/* BLE LED Service is made up of LED Chars */
BLE_Char *simpleButtonServiceChars[] = {&button1Char};

/* LED Service Declaration */
BLE_Service simpleButtonService =
{
  {0x00, 0xFF},
  1, simpleButtonServiceChars
};

void setup() {
  Serial.begin(115200);
  ble.begin();

  // Note that the switches on the MSP432P401R LP need pullups
  pinMode(button1Pin, INPUT_PULLUP);

  /* Add and initialize Button Service */
  ble.addService(&simpleButtonService);
  ble.writeValue(&button1Char, button1State);
  
  /* Start Advertising */
  ble.setAdvertName("BLE I/O Demo");
  ble.startAdvert();

  /* Print a message to the console */
  Serial.println(" BLE Energia Buttons/LED Demo");
}

// the loop routine runs over and over again forever as a task.
void loop() {
  ble.handleEvents();

  if(ble.isConnected())
  {
 if (millis() % 1000 == 0)
  {
     button1State = digitalRead(button1Pin);
     if(button1State==1){
     ble.writeValue(&button1Char,final_value2);
     Serial.println(final_value2);
     } 
     if(button1State==0){
     ble.writeValue(&button1Char,final_value);
     Serial.println(final_value);
     }
    }
  }
}
