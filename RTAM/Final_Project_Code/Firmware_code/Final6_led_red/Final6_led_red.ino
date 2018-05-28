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
/* State variables for LED chars */

uint8_t led2RedValue = 0;
uint8_t led2GreenValue = 0;
uint8_t led2BlueValue = 0;

/* Pin number variables for LEDs on MSP432P401R LaunchPad */

uint8_t led2RedPin = 75;
uint8_t led2GreenPin = 76;
uint8_t led2BluePin = 77;

/* Declare Simple LED Service Characteristics here */


BLE_Char led2RedChar =
{
  {0xF2, 0xFF},
  BLE_WRITABLE| BLE_READABLE,
  "LED 2: Red"
};

BLE_Char led2GreenChar =
{
  {0xF3, 0xFF},
  BLE_WRITABLE| BLE_READABLE,
  "LED 2: Green"
};

BLE_Char led2BlueChar =
{
  {0xF4, 0xFF},
  BLE_WRITABLE| BLE_READABLE,
  "LED 2: Blue"
};

/* BLE LED Service is made up of LED Chars */
BLE_Char *simpleLEDServiceChars[] = {&led2RedChar, &led2GreenChar, &led2BlueChar};

/* LED Service Declaration */
BLE_Service simpleLEDService =
{
  {0xF0, 0xFF},
  3, simpleLEDServiceChars
};


void setup() {
  Serial.begin(115200);
  ble.begin();

  // Initalize I/O
  pinMode(led2RedPin, OUTPUT);
  pinMode(led2GreenPin, OUTPUT);
  pinMode(led2BluePin, OUTPUT);



  /* Add and intialize LED service */
  ble.addService(&simpleLEDService);
  ble.writeValue(&led2RedChar, led2RedValue);
  ble.writeValue(&led2GreenChar, led2GreenValue);
  ble.writeValue(&led2BlueChar, led2BlueValue);


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
     
     led2GreenValue = ble.readValue_char(&led2GreenChar);
    if (led2GreenValue==48){
      digitalWrite(led2GreenPin, LOW);
    }
    if (led2GreenValue==49){
       digitalWrite(led2GreenPin, HIGH);
    }

    led2RedValue = ble.readValue_char(&led2RedChar);
        if (led2RedValue==48){
      digitalWrite(led2RedPin, LOW);
    }
    if (led2RedValue==49){
       digitalWrite(led2RedPin, HIGH);
    }
    
    }
  }
}
