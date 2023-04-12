#include <AccelStepper.h>   

const int button1Pin = 22;
const int button3Pin = 4;
const int switch1Pin = 33;

const int xAxisPin = 34; 
const int yAxisPin = 35; 

const int h1atPositionPin = 21;
const int h2atPositionPin = 17;

int baseStepperSpeed;
int j1StepperSpeed;
int maxSpeed = 1000;

int basePositions[10];
int j1Stepper1Positions[10];
int j1Stepper2Positions[10];
int posCount = 0;

int delayPositions[10];
int delayPositionsCount = 0;

unsigned long previousMillis = 0;
int timeDelay = 0;

bool dontLoop = false;
bool saveButtonHit = false;
bool replay = true;

AccelStepper baseStepper(1, 13, 14);   // (Type:driver(1 is default driver), STEP, DIR)   
AccelStepper j1Stepper1(1, 27, 26);   // (Type:driver(1 is default driver), STEP, DIR)   
AccelStepper j1Stepper2(1, 25, 32);   // (Type:driver(1 is default driver), STEP, DIR)   

void setup() 
{
  baseStepper.setMaxSpeed(maxSpeed);  //400 pulse/rev
  j1Stepper1.setMaxSpeed(maxSpeed);  //200 pulse/rev
  j1Stepper2.setMaxSpeed(maxSpeed);  //200 pulse/rev

  pinMode(xAxisPin, INPUT_PULLDOWN);
  pinMode(yAxisPin, INPUT_PULLDOWN);

  pinMode(h1atPositionPin, OUTPUT);
  pinMode(h2atPositionPin, INPUT_PULLDOWN);

  pinMode(switch1Pin, INPUT_PULLDOWN);

  pinMode(button1Pin, INPUT_PULLDOWN);
  pinMode(button3Pin, INPUT_PULLDOWN);

  previousMillis = millis();

  Serial.begin(115200);
}

void loop() 
{
  //Serial.println(baseStepper.currentPosition());
  unsigned long currentMillis = millis();
  
  if (digitalRead(switch1Pin) == HIGH)
  {
    baseStepperJoystickControl(xAxisPin, baseStepperSpeed);
    j1StepperJoystickControl(yAxisPin, j1StepperSpeed);

    if (digitalRead(button3Pin) == HIGH) //Reset
    {
      memset(basePositions, 0, sizeof(basePositions));
      memset(j1Stepper1Positions, 0, sizeof(j1Stepper1Positions));
      memset(j1Stepper2Positions, 0, sizeof(j1Stepper2Positions));
      memset(delayPositions, 0, sizeof(delayPositions));
      posCount = 0;
      delayPositionsCount = 0;
      timeDelay = 0;      
    }
    if (digitalRead(button1Pin) == HIGH && dontLoop == true)
    {
      basePositions[posCount] = baseStepper.currentPosition(); 
      j1Stepper1Positions[posCount] = j1Stepper1.currentPosition(); 
      j1Stepper2Positions[posCount] = j1Stepper2.currentPosition(); 
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
      if (baseStepper.currentPosition() == basePositions[posCount-1]) //If still
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
      for (int i = 0; i <= sizeof(basePositions) / sizeof(basePositions[0]) && digitalRead(switch1Pin) == LOW; i++) //using basePositions as base array
      {
        if (basePositions[i] == 0) //Back to 0 at end of array
          i = 0;
        
        baseStepper.moveTo(basePositions[i]); //Set target positions for runSpeedToPosition function
        baseStepper.setSpeed(500);
        j1Stepper1.moveTo(j1Stepper1Positions[i]);
        j1Stepper1.setSpeed(500);
        j1Stepper2.moveTo(j1Stepper2Positions[i]);
        j1Stepper2.setSpeed(-500);

        //Move all steppers at the same time/sync up
        while (j1Stepper1.currentPosition() != j1Stepper1Positions[i] || j1Stepper2.currentPosition() != j1Stepper2Positions[i] || baseStepper.currentPosition() != basePositions[i]) 
        {
          digitalWrite(h1atPositionPin, LOW);
          baseStepper.runSpeedToPosition();
          j1Stepper1.runSpeedToPosition();
          j1Stepper2.runSpeedToPosition();
        }
        digitalWrite(h1atPositionPin, HIGH); //Notify other ESP that it's at its positions
        while (digitalRead(h2atPositionPin) != HIGH) //Wait for other steppers
        {
          //Do nothing
        }
        delay(delayPositions[i]);
      }
    }
  }
}

void baseStepperJoystickControl(int xAxisPin, int baseStepperSpeed)
{
  baseStepperSpeed = analogRead(xAxisPin);
  baseStepperSpeed = map(baseStepperSpeed, 0, 3775, 500, -500);
  if (baseStepperSpeed > -200 && baseStepperSpeed < 200) {
    baseStepperSpeed = 0;
  }
  else if (baseStepperSpeed < -380)
  {
    baseStepperSpeed = -500;    
  }
  else if (baseStepperSpeed > 500) {
    baseStepperSpeed = 500;
  }
  
  if (baseStepperSpeed == 0) {   
    baseStepperSpeed = 0;   
    baseStepper.setSpeed(baseStepperSpeed); //steps per second 
    baseStepper.stop();
  }   
  else {
    baseStepper.setSpeed(baseStepperSpeed); //steps per second 
    baseStepper.runSpeed();  
  }
}

void j1StepperJoystickControl(int yAxisPin, int j1StepperSpeed)
{
  j1StepperSpeed = analogRead(yAxisPin);
  j1StepperSpeed = map(j1StepperSpeed, 0, 3775, -500, 500);
  if (j1StepperSpeed > -200 && j1StepperSpeed < 200) {
    j1StepperSpeed = 0;
  }
  else if (j1StepperSpeed < -360)
  {
    j1StepperSpeed = -500;    
  }
  else if (j1StepperSpeed > 500) {
    j1StepperSpeed = 500;
  }
    
  if (j1StepperSpeed == 0) {   
    j1Stepper1.setSpeed(j1StepperSpeed); //steps per second 
    j1Stepper2.setSpeed(-j1StepperSpeed); //steps per second 
    j1Stepper1.stop();
    j1Stepper2.stop();
  }   
  else {
    j1Stepper1.setSpeed(j1StepperSpeed); //steps per second 
    j1Stepper2.setSpeed(-j1StepperSpeed); //steps per second 
    j1Stepper1.runSpeed();  
    j1Stepper2.runSpeed();
  }
  //Serial.println(j1StepperSpeed);
}
