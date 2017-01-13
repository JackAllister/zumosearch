/*
 * Assignment 1
 * Zumo Search and Rescue Robot
 * 
 * Jack Allister - 23042098
 */
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <NewPing.h>

/* Module typedefs */
typedef enum
{
  GUIDED_NAVIGATE,
  SEARCH_ROOM,
  AUTONOMOUS_NAVIGATE
} OPERATING_MODE;

typedef enum
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  NONE
} MOVEMENT;

/* Module constants */
#define NUM_SENSORS 6
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define MAX_DISTANCE 20 /* maximum 20cm */

static const char CHAR_CALIBRATE = '1';
static const char CHAR_CHECK_ROOM = '2';
static const char CHAR_WALL_DETECT = 'C';
static const char CHAR_ROOM = 'R';
static const char CHAR_FORWARD = 'W';
static const char CHAR_BACKWARD = 'S';
static const char CHAR_LEFT = 'A';
static const char CHAR_RIGHT = 'D';
static const char CHAR_STOP = 0x20;

static const int MAX_SPEED = 120;

/* Module variables */
ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

OPERATING_MODE robotMode = GUIDED_NAVIGATE;
MOVEMENT currMovement = NONE;
bool wallDetect = true;

/* Speeds used for path correction */
int lastLeftSpeed = 0;
int lastRightSpeed = 0;

int roomCount = 0;

/* Module prototypes */
void parseGuidedNavigate(char recv);
void parseSearchRoom(char recv);
bool parseMovement(char recv);

bool checkForObject();
bool correctPath();
bool isWallFound();
void robotForward();
void robotBackward();
void turnLeft();
void turnRight();
void robotStop();
void calibrateSensors();

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
  char recvByte = 0;

  /* Check serial to see if we have received any commands */
  if (Serial.available() != 0)
  {
    recvByte = toupper(Serial.read());
  }
  
  switch (robotMode)
  {
    case GUIDED_NAVIGATE:
    {
      if (recvByte != 0)
      {
        if (parseMovement(recvByte) == false)
        {
          parseGuidedNavigate(recvByte);
        }
      }

      if (currMovement == FORWARD)
      {
        if (wallDetect == true)
        {
          if (isWallFound() == true)
          {
            robotStop();
            Serial.println("Wall found, press C to re-enable when moved.");
            wallDetect = false;
          }
          else if (correctPath() == true)
          {
            /* Only correct the path if a wall has not been found */
            Serial.print("Path corrected, wheels speeds: ");
            Serial.print(lastLeftSpeed);
            Serial.print(' ');
            Serial.println(lastRightSpeed);
          } 
        }
      }        
      break;
    }

    case SEARCH_ROOM:
    {
      if (recvByte != 0)
      {
        if (parseMovement(recvByte) == false)
        {
          parseSearchRoom(recvByte);
        }
      }
      break;
    }

    case AUTONOMOUS_NAVIGATE:
    {
      break;
    }
  }
  
}

void parseGuidedNavigate(char recv)
{
  switch (recv)
  {
    case CHAR_WALL_DETECT:
    {
      if (wallDetect == false)
      {
        wallDetect = true;
        Serial.println("Wall detect enabled");
      }
      else
      {
        Serial.println("Wall detect already enabled");
      }
      break;
    }

    case CHAR_ROOM:
    {
      robotMode = SEARCH_ROOM;
      roomCount++;
      
      Serial.print("Room search: ");
      Serial.println(roomCount);
      break;
    }
  }
}

void parseSearchRoom(char recv)
{ 
  switch (recv)
  {
    case CHAR_CHECK_ROOM:
    {
      checkForObject();
      break;
    }

    case CHAR_ROOM:
    {
      robotMode = GUIDED_NAVIGATE;
      Serial.println("Leaving search mode, going back to guided");
      break;
    }
  } 
}

bool parseMovement(char recv)
{
  bool result = true;
  
  switch (recv)
  {
    case CHAR_CALIBRATE:
    {
      calibrateSensors();
      break;
    }

    case CHAR_FORWARD:
    {
      robotForward();
      break;
    }

    case CHAR_BACKWARD:
    {
      robotBackward();
      break;
    }

    case CHAR_LEFT:
    {
      turnLeft();
      break;
    }

    case CHAR_RIGHT:
    {
      turnRight();
      break;
    }

    case CHAR_STOP:
    {
      robotStop();
      break;
    } 

    default:
    {
      /* 
       * If character does not match a movement key 
       * this function should return false.
       */
       result = false;
    }
  }

  return result;
}

bool checkForObject()
{
  static const unsigned long TURN_TIME = 750;

  bool found = false;
  unsigned long startTime;
  unsigned long pingDist;
  
  Serial.println("Checking room for objects");
  /* Perform a quick scan of the room using US to find items */ 

  /* Check left direction */
  Serial.println("Checking left");
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    delay(50);
    pingDist = sonar.ping_cm();
    if ((pingDist != 0) && (found == false))
    {
      /* If we get here means object found in left of room */
      Serial.print("Object found in left of room ");
      Serial.print(pingDist);
      Serial.println(" cm away.");
      found = true;
    } 
  }

  /* Move right to midpoint */
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    /* Do nothing here as returning */
  }

  /* Check right direction */
  Serial.println("Checking right");
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    delay(50);
    pingDist = sonar.ping_cm();
    if ((pingDist != 0) && (found == false))
    {
      /* If we get here means object found in right of room */
      Serial.print("Object found in right of room ");
      Serial.print(pingDist);
      Serial.println(" cm away.");
      found = true;
    }
  }

  /* Move left to midpoint */
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    /* Do nothing here as returning */
  }
  
  motors.setSpeeds(0, 0);
  return found;
}

bool correctPath()
{
  static const unsigned long CORR_INTERVAL = 50;
  static const int LEFT_SENSOR = 0;
  static const int RIGHT_SENSOR = 5;
  static const int LINE_VALUE = 400;

  static unsigned long lastRun = 0;
  unsigned int sensorValues[NUM_SENSORS];
  bool result = false;

  /* Corrects the path to stop a side from accidentally going on a wall */

  /* Correction happens every 50ms */
  if ((lastRun == 0) || (millis() - lastRun > CORR_INTERVAL))
  {
    lastRun = millis();
    
    reflectanceSensors.readLine(sensorValues);
    
    if (sensorValues[LEFT_SENSOR] >= LINE_VALUE)
    {
      motors.setSpeeds(0, 0);
      while (sensorValues[LEFT_SENSOR] >= LINE_VALUE)
      {
        motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
        
        reflectanceSensors.readLine(sensorValues);
      }
      motors.setSpeeds(lastLeftSpeed, lastRightSpeed);
      
      /* If far left sensor on a line correct path a little */
      Serial.print("Corrected left: ");
      Serial.print(lastLeftSpeed);
      Serial.print(" ");
      Serial.println(lastRightSpeed);
      result = true;
    }
    else if (sensorValues[RIGHT_SENSOR] >= LINE_VALUE)
    {
      motors.setSpeeds(0, 0);
      while (sensorValues[RIGHT_SENSOR] >= LINE_VALUE)
      {
        motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
        
        reflectanceSensors.readLine(sensorValues);
      }
      motors.setSpeeds(lastLeftSpeed, lastRightSpeed);
  
      /* If far right sensor on a line correct path a little */
      Serial.print("Corrected right: ");
      Serial.print(lastLeftSpeed);
      Serial.print(" ");
      Serial.println(lastRightSpeed);
      result = true;
    }
  }

  return result;
}

bool isWallFound()
{
  static const int LINE_VALUE = 400;
  
  unsigned int sensorValues[NUM_SENSORS];
  int i;
  int detectCount = 0;

  reflectanceSensors.readLine(sensorValues);

//  for (i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print(' ');
//  }
//  Serial.println();

  /* Check sensors 0-6 to see if wall detected */
  for (i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] >= LINE_VALUE)
    {
      detectCount++;
    }
  }

  /* Return true if more than one sensor detects a line */
  return (detectCount>1);
}

void robotForward()
{
  Serial.println("Robot moving forwards");
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);

  /* Set values needed for collision detection */
  currMovement = FORWARD;
  lastLeftSpeed = MAX_SPEED;
  lastRightSpeed = MAX_SPEED; 
}

void robotBackward()
{
  Serial.println("Robot moving backwards");
  motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);

  /* Set values needed for collision detection */
  currMovement = BACKWARD;
  lastLeftSpeed = -MAX_SPEED;
  lastRightSpeed = -MAX_SPEED; 
}

void turnLeft()
{
  Serial.println("Robot turning left");
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);

  /* Set values needed for collision detection */
  currMovement = LEFT;
  lastLeftSpeed = -MAX_SPEED;
  lastRightSpeed = MAX_SPEED; 
}

void turnRight()
{
  Serial.println("Robot turning right");
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);

  /* Set values needed for collision detection */
    currMovement = RIGHT;
  lastLeftSpeed = MAX_SPEED;
  lastRightSpeed = -MAX_SPEED; 
}

void robotStop()
{
  Serial.println("Robot stopping");
  motors.setSpeeds(0, 0);

  /* Set values needed for collision detection */
  currMovement = NONE;
  lastLeftSpeed = 0;
  lastRightSpeed = 0; 
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

