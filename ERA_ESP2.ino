#include <AccelStepper.h>   
#include <Servo.h>

Servo wristPitchServo;  // create servo object to control a servo
Servo accesServo;  // create servo object to control a servo

const int button1Pin = 22;
const int button2Pin = 4;
const int switch1Pin = 33;
const int switch2Pin = 32;
const int wristPitchServoPin = 21;
const int accesServoPin = 16;

const int xAxisPin = 34; 
const int yAxisPin = 35; 

const int h1atPositionPin = 18;
const int h2atPositionPin = 26;

int j2StepperSpeed;
int wristRollStepperSpeed;
int wristServoPos;
int wristServoMove = 100;
int accesServoPos;
int accesServoMove = 10;
int maxSpeed = 1000;

int j2Positions[10];
int wristStepperPositions[10];
int accesServoPositions[10];
int wristServoPositions[10];
int posCount = 0;

int delayPositions[10];
int delayPositionsCount = 0;

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
int timeDelay = 0;

bool dontLoop = false;
bool saveButtonHit = false;
bool replay = true;

AccelStepper j2Stepper(1, 13, 14);   // (Type:driver(1 is default driver), STEP, DIR)   
AccelStepper wristRollStepper(1, 27, 25);   // (Type:driver(1 is default driver), STEP, DIR)   

void setup() 
{
  j2Stepper.setMaxSpeed(maxSpeed);  //400 pulse/rev
  wristRollStepper.setMaxSpeed(maxSpeed);  //200 pulse/rev

  wristPitchServo.attach(wristPitchServoPin);  //Attaches the servo on pin 9 to the servo object
  accesServo.attach(accesServoPin);  //Attaches the servo on pin 9 to the servo object

  pinMode(xAxisPin, INPUT_PULLDOWN);
  pinMode(yAxisPin, INPUT_PULLDOWN);

  pinMode(h1atPositionPin, INPUT_PULLDOWN);
  pinMode(h2atPositionPin, OUTPUT);

  pinMode(switch1Pin, INPUT_PULLDOWN);
  pinMode(switch2Pin, INPUT_PULLDOWN);

  pinMode(button1Pin, INPUT_PULLDOWN);
  pinMode(button2Pin, INPUT_PULLDOWN);

  Serial.begin(115200);
}

void loop() 
{
  unsigned long currentMillis = millis();
  
  
  if (digitalRead(switch1Pin) == HIGH)
  {
    if (digitalRead(switch2Pin) == LOW) //Switch to J2 Control
    {
      j2StepperJoystickControl(xAxisPin, j2StepperSpeed);
      wristRollStepperJoystickControl(yAxisPin, wristRollStepperSpeed);
      wristPitchServo.write(wristServoMove);
      accesServo.write(accesServoMove);
    }
    else //Switch to servo control
    {
      wristServoPos = analogRead(yAxisPin);
      wristServoPos = map(wristServoPos, 0, 3775, 30, 170);//!MAY NEED TO ADJUST THESE VALUES!
      if (currentMillis - previousMillis2 >= 25) //Only allowed to update wristServoMove every 25ms
      {
        previousMillis2 = currentMillis;
        if (wristServoPos > 140) //!MAY NEED TO ADJUST THESE VALUES!
          wristServoMove++;
        else if (wristServoPos < 95) 
          wristServoMove--;
      }
      wristPitchServo.write(wristServoMove);

      accesServoPos = analogRead(xAxisPin);
      accesServoPos = map(accesServoPos, 0, 3775, 30, 170); //!MAY NEED TO ADJUST THESE VALUES!
      if (currentMillis - previousMillis3 >= 25) //Only allowed to update wristServoMove every 25ms
      {
        previousMillis3 = currentMillis;
        if (accesServoPos > 140) //!MAY NEED TO ADJUST THESE VALUES!
          accesServoMove++;
        else if (accesServoPos < 95) 
          accesServoMove--;
      }
      accesServoMove = constrain(accesServoMove, 16, 80);
      accesServo.write(accesServoMove);
    }

    if (digitalRead(button2Pin) == HIGH) //Reset
    {
      memset(j2Positions, 0, sizeof(j2Positions));
      memset(wristStepperPositions, 0, sizeof(wristStepperPositions));
      memset(wristServoPositions, 0, sizeof(wristServoPositions));
      memset(accesServoPositions, 0, sizeof(accesServoPositions));
      memset(delayPositions, 0, sizeof(delayPositions));
      posCount = 0;
      delayPositionsCount = 0;
      timeDelay = 0;      
    }
    if (digitalRead(button1Pin) == HIGH && dontLoop == true) //Save positions
    {
      j2Positions[posCount] = j2Stepper.currentPosition(); 
      wristStepperPositions[posCount] = wristRollStepper.currentPosition(); 
      wristServoPositions[posCount] = wristServoMove;
      accesServoPositions[posCount] = accesServoMove;
      posCount++;
      delay(100);
      previousMillis = currentMillis; //Start clock
      saveButtonHit = true;
      dontLoop = false;
    }
    else if (digitalRead(button1Pin) == LOW)
    {
      dontLoop = true;
    }

    if (saveButtonHit) //So entire statement will execute
    {
      if (wristRollStepper.currentPosition() == wristStepperPositions[posCount-1]) //If still
      {
        timeDelay = currentMillis - previousMillis;
        delayPositions[delayPositionsCount] = timeDelay;
      }
      else
      {
        delayPositionsCount++;
        saveButtonHit = false;
      }
    }
  }
  else
  {
    while (digitalRead(switch1Pin) == LOW)
    {
      for (int i = 0; i <= sizeof(wristStepperPositions) / sizeof(wristStepperPositions[0]) && digitalRead(switch1Pin) == LOW; i++) //using wristStepperPositions as base array
      {
        if (wristStepperPositions[i] == 0) //Back to 0 at end of array
          i = 0;
        
        j2Stepper.moveTo(j2Positions[i]); //Set target positions for runSpeedToPosition function
        j2Stepper.setSpeed(500);
        wristRollStepper.moveTo(wristStepperPositions[i]);
        wristRollStepper.setSpeed(500);

        //Move all steppers at the same time/sync up
        while (wristRollStepper.currentPosition() != wristStepperPositions[i] || j2Stepper.currentPosition() != j2Positions[i]) // || accesServoPositions[i] != accesServoPos || wristServoPositions[i] != wristServoPos
        {
          digitalWrite(h2atPositionPin, LOW);
          wristPitchServo.write(wristServoPositions[i]);
          j2Stepper.runSpeedToPosition();
          wristRollStepper.runSpeedToPosition();
        }
        digitalWrite(h2atPositionPin, HIGH);//Notify other ESP that it's at its positions
                                                                        
        while (digitalRead(h1atPositionPin) != HIGH) //Wait for other steppers
        {
          //Do nothing
        }
        accesServo.write(accesServoPositions[i]);
        delay(delayPositions[i]);
      }
    }
  }
}

void j2StepperJoystickControl(int xAxisPin, int j2StepperSpeed)
{
  j2StepperSpeed = analogRead(yAxisPin);
  j2StepperSpeed = map(j2StepperSpeed, 0, 3775, 500, -500);
  if (j2StepperSpeed > -200 && j2StepperSpeed < 200) {
    j2StepperSpeed = 0;
  }
  else if (j2StepperSpeed < -380)
  {
    j2StepperSpeed = -500;    
  }
  else if (j2StepperSpeed > 500) {
    j2StepperSpeed = 500;
  }
  
  if (j2StepperSpeed == 0) {   
    j2StepperSpeed = 0;   
    j2Stepper.setSpeed(j2StepperSpeed); //steps per second 
    j2Stepper.stop();
  }   
  else {
    j2Stepper.setSpeed(j2StepperSpeed); //steps per second 
    j2Stepper.runSpeed();  
  }
}

void wristRollStepperJoystickControl(int yAxisPin, int wristRollStepperSpeed)
{
  wristRollStepperSpeed = analogRead(xAxisPin);
  wristRollStepperSpeed = map(wristRollStepperSpeed, 0, 3775, -500, 500);
  if (wristRollStepperSpeed > -200 && wristRollStepperSpeed < 200) {
    wristRollStepperSpeed = 0;
  }
  else if (wristRollStepperSpeed < -360)
  {
    wristRollStepperSpeed = -500;    
  }
  else if (wristRollStepperSpeed > 500) {
    wristRollStepperSpeed = 500;
  }
    
  if (wristRollStepperSpeed == 0) {   
    wristRollStepper.setSpeed(wristRollStepperSpeed); //steps per second 
    wristRollStepper.stop();
  }   
  else {
    wristRollStepper.setSpeed(wristRollStepperSpeed); //steps per second 
    wristRollStepper.runSpeed();  
  }
}
