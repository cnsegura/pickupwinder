// Pickup winder controller

#include <Wire.h>               // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h"  // Used to send and recieve specific information from our sensor
#include <Arduino_GFX_Library.h>
#include "FreeMono14pt7b.h"


// Motor/PWM control 
uint8_t potInput = A0;
uint16_t motorSpeed;
uint16_t potAnalog; 

uint8_t motorEnA = 9; //PWM pin
uint8_t motorIn1 = 6;
uint8_t motorIn2 = 7;

// Hall effect sensor 
TMAG5273 sensor;  // Initialize hall-effect sensor
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;
uint16_t rotationCount;
const byte intPin = 8;
unsigned long lastInterrupt = 0;
bool thresholdCrossed = false; //ISR indicator

//Display control
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
Arduino_DataBus *bus = create_default_Arduino_DataBus();

// 0 == portrait (top is opposite pin header)
// 1 == CW/landscape rotation from avove
// 2 == portrait (top is on header side)
// 3 == CW rotation from above
Arduino_GFX *gfx = new Arduino_ST7796(bus, 3 /* RST */, 3 /* rotation */, true /* IPS */);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  
  //PWM setup
  pinMode(motorEnA, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  /*
            motorIn1 | motorIn2 | Direction
    value     LOW    |  LOW     | STOP
              LOW    |  HIGH    | CCW
              HIGH   |  LOW     | CW
              HIGH   |  HIGH    | STOP
  */
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  
  //Hall setup
  pinMode(intPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin), thresholdCross, FALLING);

  // If begin is successful (0), then start example
  if (sensor.begin(i2cAddress, Wire) == true)
  {
    Serial.println("Begin");
  } 
  else  // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while (1);  // Runs forever
  }
  sensor.setInterruptMode(TMAG5273_INTERRUPT_THROUGH_INT);
  // Set the !INT pin state - latched mode == false (vs 10us pulse mode == true)
  sensor.setIntPinState(false);
  // Enable the interrupt response for the thresholds
  sensor.setThresholdEn(true);
  // Set X, Y, Z, and T Thresholds for interrupt to be triggered
  sensor.setZThreshold(8);            // mT

  //Display setup
  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

  #ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
  #endif

  gfx->setFont(&FreeMono14pt7b);
  gfx->setTextColor(WHITE, BLACK);

  delay(5000); // 5 seconds

  gfx->fillScreen(BLACK); 

}

void loop() {
  uint8_t interruptPinState;
  float magZ;
  char buf[5]; //4 digits plus end /0
  
  potAnalog = analogRead(potInput);
  motorSpeed = map(potAnalog, 0, 1023, 0, 127);

  //Set motor speed based on analog value from above
  analogWrite(motorEnA, motorSpeed);

  if(thresholdCrossed == true ) {
    magZ = sensor.getZData();
    
    if(magZ < 8) { //we've fallen beneath the threshold that triggered the interrupt, presumably this means the magnet is moving away
      thresholdCrossed = false; 
      rotationCount++; 
      interruptPinState = sensor.getInterruptPinStatus(); //reset interrupt pin
    }
  }
  gfx->drawRect(190, 128, 75, 45, RED);
  gfx->setCursor(200, 160);
  sprintf(buf, "%4d", rotationCount);
  gfx->setCursor(200,160);
  gfx->println(buf);

  
  // DEBUG STUFF
   /*
    float magXx = sensor.getXData();
    float magY = sensor.getYData();
    float magZz = sensor.getZData();

    Serial.print("MagX:");
    Serial.print(magXx);
    Serial.print(",");
    Serial.print("MagY:");
    Serial.print(magY);
    Serial.print(",");
    Serial.print("MagZ:");
    Serial.println(magZz);
 
  if(motorSpeed > 70) {
    Serial.print("Analog: ");
    Serial.print(potAnalog);
    Serial.print(", Speed: ");
    Serial.println(motorSpeed);
    Serial.print("Count is: ");
    Serial.println(rotationCount);
  }
  else{
    Serial.println("Stopped");
  }
  */
  //if((myStopTime - myStartTime) > 60000 && (myStopTime-myStartTime <61000)) {
    //Serial.print("Count is: ");
    //Serial.println(rotationCount);
    //Serial.print("Analog: ");
    //Serial.print(potAnalog);
  //}

    
  //END DEBUG STUFF


}

// ISR for mag sensor threshold crossing 
void thresholdCross() 
{
  thresholdCrossed = true;
}
