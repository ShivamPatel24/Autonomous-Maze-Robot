//Contributers: Shivam Patel, Gianluca Basile


#include <Servo.h>
#include <LiquidCrystal.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <EEPROM.h>

Servo leftServo, rightServo, fanServo, scanServo;
//Analog pin A4,A5 are allocated to the gyro
int rightServoPin = A3;       // Right Servo Signal Pin
int leftServoPin = A2;        // Left Servo Signal Pin
int IRSensorPin  = A1; // scanServo - currently IR sensory
int scanServoPin = 4; //original name: fanLiftServo; current serving as Scan Servo
int flameSensor = A0;
int statusButtonPin = 3;
//int blueLedPin = ;
int greenLedPin = 5;          // DIO Red Led Pin
int buzzerPin = 6;
int fanControl = 7;
//int redLedPin = ; ///
int brightness = 0;            // Starting Brightness Value
int fadeValue = 5;             // Pulse Value
int turnTime = 610;           // Turn Delay
int scanTime = 650;            // Scan Delay
int leftWheelSpeed = 0;
int rightWheelSpeed = 0;
int stringLengthOld = 0;      //Used to compare length of text displayed on LCD
int valueLengthOld = 0;        //Used to compare length of text displayed on LCD
int status = 0;                // 0 -> Stanby, 1 -> Running
int EEPROMaddress = 0;     //Used to change the adress of the EEPROM
boolean mazeEnd = false;
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // initialize the library with the numbers of the interface pins

//GYRO
MPU6050 mpu;
float straightDriveAngle = 0;
#define OUTPUT_READABLE_EULER
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
float euler[3];
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//GYRO END



void setup() {

  lcd.begin(16, 2); // set up the LCD's number of columns and rows:
  pinMode(greenLedPin, OUTPUT);                      // DIO Pin 3 -> Output
  //  pinMode(blueLedPin, OUTPUT);                       // DIO Pin42 -> Output
  pinMode(statusButtonPin, INPUT_PULLUP);            // Button for ON/OFF Button
  //  pinMode(redLedPin, OUTPUT);
  rightServo.attach (rightServoPin);                 // Assign Right Servo Pin
  leftServo.attach (leftServoPin);                   // Assign Left Servo Pin
  scanServo.attach (scanServoPin);
  Serial.begin(9600);                               // Serial Monitor
  lcd.begin(16, 2);                                 // set up the LCD's number of columns and rows:
  driveControl (90, 90, 0);
  //Gyro Began
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  //GYRO END
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  int counter = 0;
  LCDwrite ("Stabalizing!", 12);
  while (counter != 2200)
  {
    gyro();
    delay(1);
    counter++;
  }
  playTone (365, 200, 1);
  LCDwrite ("Im Ready!", 9);
}

void loop() {
  if (digitalRead(statusButtonPin) == LOW && status == 0 )                         // Check for Button Press
  {
    playTone (600, 200, 0);
    playTone (440, 100, 3);
    delay (200);
    status = 1;
    straightDriveAngle = gyro();
  }
  if (digitalRead(statusButtonPin) == LOW && status == 1)                         // Check for Button Press
  {
    playTone (300, 300, 0);
    digitalWrite (greenLedPin, LOW);
    status = 0;
    playTone (100, 300, 0);
  }

  if (status == 1) {
    mainDrive(); // Run Main Drive Procedure
    if (int(straightDriveAngle) < int(gyro())) {
      int rCorrect = 0;
      int currentAngle = gyro();
      rCorrect = abs ((currentAngle - straightDriveAngle * 0.5));
      driveControl (180, 0 + rCorrect, 0);
    }
    else if (int(straightDriveAngle) > int(gyro())) {
      int lCorrect = 0;
      int currentAngle = abs(gyro());
      lCorrect = abs((currentAngle - straightDriveAngle * 2));

      driveControl (100, 0, 0);
    }
    else {
      driveControl (180, 0, 0);
    }
  }
  else {
    driveControl (90, 90, 0);                        // Stop Servo Motors
    gyro();
    LCDwrite ("I'm Ready!", 10);
  }
}

int mainDrive () {
  int leftDistance, rightDistance, sensorValue = 0;
  ledPulse (true, greenLedPin);                      // Begin Green Pulse
  LCDwrite("Driving...", 10);
  //  digitalWrite (blueLedPin, LOW);                    // Set Blue LED OFF
  sensorValue = analogRead (1); // Read Analog Distance Sensor
  if (sensorValue > 500 && status == 1 && mazeEnd == false) {
    playTone(800, 100, 0);
    LCDwrite ("I am scanning!", 14);
    digitalWrite (greenLedPin, LOW);                 // Turn OFF Green LED
    //    digitalWrite (blueLedPin, HIGH);                 // Turn ON Blue LED
    driveControl( 90, 90, turnTime);
    scanServo.write(0);
    delay (scanTime);
    leftDistance = analogRead (1);;
    LCDwriteValue ("LDist:", leftDistance, 9);
    scanServo.write(180);
    delay (scanTime);
    rightDistance = analogRead (1);
    LCDwriteValue ("RDist:", rightDistance, 9);
    scanServo.write(90);
    delay (scanTime);
    mazeSolver (leftDistance, rightDistance);// Run Obstalce Avoidance
    //                                                 Return Collected Distances
  }
  else if (sensorValue > 500 && status == 1 && mazeEnd == true)  {
    mazeSolver (0, 0);
  }
  return 0;                                         // End of Procedure, Send to Main
}
void mazeSolver (int leftValue, int rightValue ) {
  int startingAngle = gyro();
  int turnAngle = 0;
  int turnDirection = 0;
  if (mazeEnd == false) {
    if ((leftValue > 300) && (rightValue > 300)) {
      LCDwrite ("End of maze!", 12);
      turnAngle = angleCalculator(straightDriveAngle, 180);
      while ((gyro()) != turnAngle) {
        driveControl (75, 75, 0);   // Turn Left
      }
      mazeEnd = true;
      EEPROMaddress++;
    }
    else if (leftValue > rightValue) {
      turnAngle = angleCalculator(straightDriveAngle, 90);
      while (gyro() != turnAngle) {
        driveControl (75, 75, 0);              // Turn Right
      }
      EEPROMaddress++;
      EEPROM.write (EEPROMaddress, 1); //1 = right turn
    }
    else {
      turnAngle = angleCalculator(straightDriveAngle, -90);
      while ((gyro()) != turnAngle) {
        driveControl (100, 100, 0);   // Turn Left
      }
      EEPROMaddress++;
      EEPROM.write (EEPROMaddress, 2); //2 = left turn
    }
  }
  else {
    EEPROMaddress--;
    turnDirection = EEPROM.read (EEPROMaddress);
    if (turnDirection == 1) {
      turnAngle = angleCalculator(straightDriveAngle, -90);
      while ((gyro()) != turnAngle) {
        driveControl (100, 100, 0);   // Turn Left
      }
    }
    else if (turnDirection == 2) {
      turnAngle = angleCalculator(straightDriveAngle, 90);
      while (gyro() != turnAngle) {
        driveControl (75, 75, 0);              // Turn Right
      }
    }
    else{
    status = 0;
    mazeEnd = false;
    }
  }
  driveControl (90, 90, 0);
  straightDriveAngle = turnAngle;
}

float angleCalculator (float initial, float rotateAngle) {
  int finalAngle = initial + rotateAngle;
  if (finalAngle > 180) {
    finalAngle = (finalAngle - 360);
  }
  else if (finalAngle < (-180)) {
    finalAngle = (finalAngle + 360);
  }
  return finalAngle;
}
boolean ledPulse (boolean status, int ledPin) {
  if (status = 1) {
    analogWrite(ledPin, brightness);
    brightness = brightness + fadeValue;
    if (brightness == 0 || brightness == 255) {
      fadeValue = -fadeValue ;
    }
  }
}
void driveControl (int leftDegree, int rightDegree, int timeDelay) {
  leftServo.write(leftDegree);
  rightServo.write (rightDegree);
  delay (timeDelay);
}

void LCDwrite (String text, int stringLength)
{
  if (stringLengthOld > stringLength)
  {
    lcd.clear();
  }
  lcd.setCursor(0, 0);
  lcd.print(text);

  stringLengthOld = stringLength;
}

void LCDwriteValue (String label, float value, int valueLength)
{
  if (valueLengthOld > valueLength)
  {
    lcd.clear();
  }
  lcd.setCursor(0, 1);
  lcd.print(label);
  lcd.setCursor (valueLength - 3, 1);
  lcd.print(value);
  valueLengthOld = valueLength;
}

int gyro() {
reAccess:
  if (!dmpReady) {
    goto reAccess;
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
checkAngle:
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    float currentAngle = euler[0] * 180 / M_PI;
    LCDwriteValue ("Angle:", currentAngle, 9);
  }
  return (euler[0] * 180 / M_PI);
}
void playTone (int frequency, int duration, int repeat) {
  for (int counter = 0; counter <= repeat; counter++) {
    tone (buzzerPin, frequency + frequency * counter, duration);
    delay (duration);
  }
}

