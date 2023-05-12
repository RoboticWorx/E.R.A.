#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

float theta1;
float theta2;

float x;
float y;
float z;

float delta;

float theta3;
float psi = 180; //Desired gripper orientation. 
const int wristPin = 2;

int j1Speed = 600;
int j2Speed = 800;
int baseSpeed = 400;
int previousMillis = 0;

Servo wrist;  //Create servo object to control a servo
AccelStepper baseStepper(1, 5, 4);   // (Type:driver(1 is default driver), STEP, DIR)
AccelStepper j1Stepper_L(1, 7, 6);   // (Type:driver(1 is default driver), STEP, DIR)
AccelStepper j1Stepper_R(1, 17, 15);   // (Type:driver(1 is default driver), STEP, DIR)
AccelStepper j2Stepper(1, 10, 9);   // (Type:driver(1 is default driver), STEP, DIR)

//WIFI
typedef struct struct_message {
    float x;
    float y;
    float z;
} struct_message;

// Create a struct_message called myData
struct_message myData;
 
void setup() {
  //Init steppers (Half steps)
  baseStepper.setMaxSpeed(baseSpeed); //400 pulse/rev
  j1Stepper_L.setMaxSpeed(j1Speed); //400 pulse/rev
  j1Stepper_R.setMaxSpeed(j1Speed); //400 pulse/rev
  j2Stepper.setMaxSpeed(j2Speed); //400 pulse/rev

  wrist.attach(wristPin);

  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //WIFI
  WiFi.mode(WIFI_STA);

  //WIFI
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  //WIFI
  esp_now_register_recv_cb(OnDataRecv);

  //Start position
  x = 400;
  y = 0; //Remember - down is positive!
  z = 0; 
}
 
void loop() 
{
  //Testing, used for moving it in a straight line. Comment out if using predetermined coordinates, otherwise will pick up random signals
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 20)
  {
    previousMillis = currentMillis;
    if (myData.x > 400)
      x++;
    else if (myData.x < 100)
      x--;

    if (myData.y > 400) 
      z++; //CHANGED FOR TESTING
    else if (myData.y < 100)
      z--;

    //if (myData.z > 400)
    //  z++;
    //else if (myData.z < 100)
    //  z--;
  }

  inverseKinematics(x, y, z); //x, y & z are the coordinates sent from the controller. ARM MUST BE COMPLETELY VERTICAL AT START TO WORK (home position, since all relative to y-axis)

  //These 3 ifs needs to be commented out if youre controling it in a straight line! 
  //If motors close, slow down (by 2)
  if (isClose(j1Stepper_L, 200)) //(Stepper, steps to be considered close)
    j1Speed /= 2;
  if (isClose(j2Stepper, 200)) //(Stepper, steps to be considered close)
    j2Speed /= 2;
  if (isClose(baseStepper, 200)) //(Stepper, steps to be considered close)
    baseSpeed /= 2;
  

  //Calc degrees to steps and move (assuming 400 steps/rev)
  //Tell the motors which way to get to theta if past 90deg. (Do the motors need to go left or right)
  if (theta1 > 90)
  {
    j1Stepper_L.moveTo((90 - theta1) * -37.037037); //Set target position (37.037037 steps/degree)
    j1Stepper_R.moveTo((90 - theta1) * 37.037037); //Set target position (37.037037 steps/degree)
    j1Stepper_L.setSpeed(j1Speed); //Neg. since it needs to move in the other direction
    j1Stepper_R.setSpeed(-j1Speed); //Neg. since it needs to move in the other direction
  }
  else
  {
    //90 - theta1 to make it relative to y-axis
    j1Stepper_L.moveTo((90 - theta1) * -37.037037); //Set target position (37.037037 steps/degree)
    j1Stepper_R.moveTo((90 - theta1) * 37.037037); //Set target position (37.037037 steps/degree)
    j1Stepper_L.setSpeed(-j1Speed); 
    j1Stepper_R.setSpeed(j1Speed); 
  }  

  j2Stepper.moveTo(theta2*50); //Set target position (50 steps/degree). 
  j2Stepper.setSpeed(j2Speed); //800 steps/sec

  baseStepper.moveTo(delta*16.666667); //Set target position (16.666667 steps/degree). 
  baseStepper.setSpeed(baseSpeed); // steps/sec

  if (theta3 > 0)
    wrist.write(theta3*.666667); //*.666667 because I had to scale the 0-180deg write value to 0-270. (Used 270deg servo). Also you need make sure gripper has access to full range of motion when mounting. (If max rotation 270deg, mount it in a way so it can rotate that)

  j1Stepper_L.runSpeedToPosition();
  j1Stepper_R.runSpeedToPosition();
  j2Stepper.runSpeedToPosition();
  baseStepper.runSpeedToPosition();
}

//Callback function that will be executed when data is received
//WIFI
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}

void inverseKinematics(float x, float y, float z) //x & y are desired position coordinates (mm)
{
  j1Speed = 600;
  j2Speed = 800;
  baseSpeed = 800;

  const float pi = 3.141593;
  float j1 = 255.68; //Length J1 in mm
  float j2 = 428.40; //Length J2 in mm

  //Z move
  delta = atan(z/x); //Angle base needs to move
  delta *= (180/pi); //Radians to degrees

  float x2 = sqrt(sq(z) + sq(x)) - x; //Difference added to x to stay alligned with x-coord as base moves

  if (z != 0)
    x += x2;

  theta2 = -acos((sq(x) + sq(y) -sq(j1) - sq(j2)) / (2 * j1 * j2)); //Calculate theta2 (in rads)

  theta1 = atan(y / x) + atan((j2 * sin(theta2)) / (j1 + j2 * cos(theta2))); //Calculate theta1 (in rads)

  //Radians to degrees
  theta2 *= (180/pi);
  theta1 *= (180/pi);
  
  //Adjust angles 
  if (theta2 < 0 && theta1 > 0) 
  {
    if (theta1 < 90)
    {
      theta1 += (180-(theta1 * 2)); //Mirror across y
      theta2 *= -1;
    }
    else if (theta1 > 90)
    {
      theta1 = theta1 - (2 * (theta1 - 90)); //Mirror across y
      theta2 *= -1;
    }
  }
  else if (theta1 < 0 && theta2 < 0)
  {
    theta1 *= -1;
    theta2 *= -1;
  }

  //Gripper orientation. 
  //Psi is desired orientation with respect to y-axis. (180 is straight down)
  theta3 = psi - theta2 - (90 - theta1); //(90 - theta1) so we can make it relative to y-axis. Did it here so it wouldn't be an abs(value)
  theta3 = 180 - theta3; //180 because that is the angle for gripper to be straight, & minus sign because down is negative for how I mounted the gripper  
}

bool isClose(AccelStepper s1, int threshold) //"s1" corresponds with the stepper motor declared in the function call, threshold is steps to be considered close
{
  if (abs(s1.distanceToGo()) < threshold)
  { 
    return true;
  }
  return false;
}
