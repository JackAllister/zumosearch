/*
 * Assignment 1
 * Zumo Search and Rescue Robot
 * 
 * Jack Allister - 23042098
 */
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

/* Module typedefs */
typedef enum
{
  GUIDED_NAVIGATE,
  SEARCH_ROOM,
  AUTONOMOUS_NAVIGATE
} OPERATING_MODE;

/* Module constants */
#define NUM_SENSORS 6

static const char CALIBRATE = 'C';
static const char FORWARD = 'W';
static const char LEFT = 'A';
static const char RIGHT = 'D';
static const char BACKWARD = 'S';
static const char STOP_ROBOT = 0x20;

static const int MAX_SPEED = 100;

/* Module variables */
ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;

/* Calibration variables used for left/right turns */
unsigned long calLeftTime = 0;
unsigned long calRightTime = 0;

OPERATING_MODE robotMode = GUIDED_NAVIGATE;

/* Module prototypes */
void parseGuidedNavigate();
void turnLeft();
void turnRight();
void calibrateSensors();
void calibrateTurns();

/* Module code */
void setup() 
{
  Serial.begin(9600);

  reflectanceSensors.init();

  /* Make sure the robot is not moving */
  motors.setSpeeds(0, 0);
}

void loop() 
{
  switch (robotMode)
  {
    case GUIDED_NAVIGATE:
    {
      parseGuidedNavigate();
      break;
    }

    case SEARCH_ROOM:
    {
      break;
    }

    case AUTONOMOUS_NAVIGATE:
    {
      break;
    }
  }
  
}

void parseGuidedNavigate()
{
  if (Serial.available() > 0)
  {
    char recvByte = toupper(Serial.read());

    switch (recvByte)
    {
      case CALIBRATE:
      {
        calibrateSensors();
        calibrateTurns();
        break;
      }

      case FORWARD:
      {
        motors.setSpeeds(MAX_SPEED, MAX_SPEED);
        break;
      }

      case BACKWARD:
      {
        motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
        break;
      }

      case LEFT:
      {
        turnLeft();
        break;
      }

      case RIGHT:
      {
        turnRight();
        break;
      }

      case STOP_ROBOT:
      {
        motors.setSpeeds(0, 0);
      }
    
    }
  }
}

void turnLeft()
{
  /* Turns the robot left 90 degress */
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  delay(calLeftTime);
  motors.setSpeeds(0, 0);
}

void turnRight()
{
  /* Turns the robot right 90 degress */
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
  delay(calRightTime);
  motors.setSpeeds(0, 0);
}

void calibrateSensors()
{
  char dummy;
  int i;

  Serial.println("Calibrating sensors");
  Serial.println("Please move the robot so that the sensors are on a line/wall");
  Serial.println("Press space to continue");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();
  
  for (i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
    else
      motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
    reflectanceSensors.calibrate();

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  motors.setSpeeds(0,0);

  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print("Minimum values: ");
    Serial.print(reflectanceSensors.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print("Maximum values: ");
    Serial.print(reflectanceSensors.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Sensor calibration complete!");
}

void calibrateTurns()
{
  static const unsigned long WAIT_TIME = 500;
  
  unsigned long startTime;
  unsigned long endTime;
  char dummy;

  /* 
   *  This section calculates how long it takes to turn 90 degress, this is used
   *  so that we have accurate turning for the arduino.
   */  
  Serial.println("Calibrating turns, space to continue");
  Serial.println("Send space once then again, when the robot has turned 90* left");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();
  delay(WAIT_TIME);
  
  startTime = millis();
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);

  /* Wait until the second press */
  while (Serial.available() == 0)
  {
    
  }
  dummy = Serial.read();
  endTime = millis();
  motors.setSpeeds(0, 0);

  /* Below is the time it takes to turn 90 degress left */
  calLeftTime = endTime - startTime;
  Serial.print("Turn left time: ");
  Serial.println(calLeftTime);
  
  /* Now we do the same again but for right turns */
  Serial.println("Send space once then again, when the robot has turned 90* right");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();
  delay(WAIT_TIME);
  
  startTime = millis();
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);

  /* Wait until the second press */
  while (Serial.available() == 0)
  {
    
  }
  dummy = Serial.read();
  endTime = millis();
  motors.setSpeeds(0, 0);

  /* Below is the time it takes to turn 90 degress left */
  calRightTime = endTime - startTime;
  Serial.print("Turn right time: ");
  Serial.println(calRightTime);

  Serial.println("Turn calibration complete!");
}

