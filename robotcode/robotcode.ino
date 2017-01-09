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
static const char BACKWARD = 'S';
static const char LEFT = 'A';
static const char RIGHT = 'D';
static const char STOP_ROBOT = 0x20;

static const int MAX_SPEED = 100;

/* Module variables */
ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;

/* Calibration variables used for left/right turns */
unsigned long calLeftTime = 0;
unsigned long calRightTime = 0;

bool isMoving = false;

OPERATING_MODE robotMode = GUIDED_NAVIGATE;

/* Module prototypes */
void parseGuidedNavigate();
bool isWallFound();
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

      if (isMoving && (isWallFound() == true))
      {
        robotStop();
        Serial.println("Wall found");
      }
        
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
        robotForward();
        break;
      }

      case BACKWARD:
      {
        robotBackward();
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
        robotStop();
        break;
      } 
    }
  }
}

bool isWallFound()
{
  static const int LINE_VALUE = 650;
  
  unsigned int sensorValues[NUM_SENSORS];
  int i;
  bool result = true;

  reflectanceSensors.readLine(sensorValues);

  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.println();

  /* Check sensors 1-4 to see if wall detected */
  for (i = 1; i < 4; i++)
  {
    if (sensorValues[i] <= LINE_VALUE)
    {
      result = false;
    }
  }

  return result;
}

void robotForward()
{
  Serial.println("Robot moving forwards");
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  isMoving = true;
}

void robotBackward()
{
  Serial.println("Robot moving backwards");
  motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
  isMoving = true;
}

void robotStop()
{
  Serial.println("Robot stopping");
  motors.setSpeeds(0, 0);
  isMoving = false;
}

void turnLeft()
{
  Serial.println("Robot turning left 90*");
  
  /* Turns the robot left 90 degress */
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  delay(calLeftTime);
  motors.setSpeeds(0, 0);
}

void turnRight()
{
  Serial.println("Robot turning right 90*");
  
  /* Turns the robot right 90 degress */
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
  delay(calRightTime);
  motors.setSpeeds(0, 0);
}

void calibrateSensors()
{
  static const unsigned long MOVE_TIME = 350;
  
  char dummy;
  int i;

  Serial.println("Calibrating sensors");
  Serial.println("Move the robot so that the sensors are just before a line/wall");
  Serial.println("Press space to continue");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();

  for (i = 0; i < 10; i++)
  {
    unsigned long startTime = millis();
    motors.setSpeeds(MAX_SPEED, MAX_SPEED);
    while (millis() - startTime < MOVE_TIME)
    {
      reflectanceSensors.calibrate();
    }

    startTime = millis();
    motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
    while (millis() - startTime < MOVE_TIME)
    {
      reflectanceSensors.calibrate();
    }
  }
  motors.setSpeeds(0, 0);

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

