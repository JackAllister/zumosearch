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

typedef struct
{
  MOVEMENT movement;
  unsigned long time;
} MOVEMENT_COORD;

/* Module constants */
#define NUM_SENSORS 6

/* Ultrasonic sensor */
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define MAX_DISTANCE 20

#define MAX_COORDINATES 60

static const char CHAR_CALIBRATE = '1';
static const char CHAR_CHECK_ROOM = '2';
static const char CHAR_START_AUTONOMOUS = '3';
static const char CHAR_WALL_DETECT = 'C';
static const char CHAR_ROOM = 'R';
static const char CHAR_AUTONOMOUS = 'E';
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

int roomCount = 0;

/* Array of directions used for autonomous mode */
MOVEMENT_COORD movLog[MAX_COORDINATES] = {NONE, 0};
int movLogCount = 0;

/* Module prototypes */
void parseGuidedNavigate(char recv);
void parseSearchRoom(char recv);
bool parseMovement(char recv);

void runAutonomousMode();
MOVEMENT findInverse(MOVEMENT movement);
void moveDirection(MOVEMENT movement);

bool checkForObject();
bool correctPath();
bool isWallFound();
int isSensorsOver(int startSensor, int endSensor);
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
            moveDirection(NONE);
            Serial.println("Wall found, press C to re-enable when moved.");
            wallDetect = false;
          }
          else
          {
            /* Only correct the path if a wall has not been found */
            correctPath();
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
      if (recvByte != 0)
      {
        if (parseMovement(recvByte) == false)
        {
          parseAutonomous(recvByte);
        }
      }
      break;
    }
  }
  
}

void parseGuidedNavigate(char recv)
{
  switch (recv)
  {
    case CHAR_CALIBRATE:
    {
      calibrateSensors();
      break;
    }

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

    case CHAR_AUTONOMOUS:
    {
      robotMode = AUTONOMOUS_NAVIGATE;
      Serial.println("Autonomous mode started, rotate 180* then press 3");
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

void parseAutonomous(char recv)
{
  switch (recv)
  {
    case CHAR_START_AUTONOMOUS:
    {
      runAutonomousMode();
      break;
    }

    case CHAR_AUTONOMOUS:
    {
      robotMode = GUIDED_NAVIGATE;
      Serial.println("Leaving autonomous mode, going back to guided");
      break;
    }
  }
}

bool parseMovement(char recv)
{
  bool result = true;
  
  switch (recv)
  {
    case CHAR_FORWARD:
    {
      moveDirection(FORWARD);
      break;
    }

    case CHAR_BACKWARD:
    {
      moveDirection(BACKWARD);
      break;
    }

    case CHAR_LEFT:
    {
      moveDirection(LEFT);
      break;
    }

    case CHAR_RIGHT:
    {
      moveDirection(RIGHT);
      break;
    }

    case CHAR_STOP:
    {
      moveDirection(NONE);
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

void runAutonomousMode()
{
  int i = 0;
  MOVEMENT inverse;
  unsigned long startTime;

  for (i = movLogCount; i != -1; i--)
  {
    Serial.print("Doing log: ");
    Serial.println(i);
    
    if (movLog[i].movement != NONE)
    {      
      inverse = findInverse(movLog[i].movement);
    
      /* Move in inverse direction to backtrace */
      startTime = millis();
      moveDirection(inverse);
      while (millis() - startTime < movLog[i].time)
      {
        if (inverse == FORWARD)
        {
          if (isWallFound() == true)
          {
            motors.setSpeeds(0, 0);
          }
          else
          {
            /* Only correct the path if a wall has not been found */
            correctPath();
          }  
        }
      }
    }
  }
  moveDirection(NONE);
}

MOVEMENT findInverse(MOVEMENT movement)
{
  MOVEMENT result = NONE;
  
  switch (movement)
  {
    case FORWARD:
    {
      result = FORWARD;
      break;
    }

    case BACKWARD:
    {
      result = BACKWARD;
      break;
    }

    case LEFT:
    {
      result = RIGHT;
      break;
    }

    case RIGHT:
    {
      result = LEFT;
      break;
    }
  }

  return result;
}


void moveDirection(MOVEMENT movement)
{
  /* Method for storing coordinates in guided mode */
  if (robotMode == GUIDED_NAVIGATE)
  {
    /* Set movement and start time */
    unsigned long currTime = millis();
    movLog[movLogCount].movement = movement;
    movLog[movLogCount].time = currTime;

    /* Here we correct previous log time so that it is delta/diff */
    if (movLogCount != 0)
    {
      movLog[movLogCount-1].time = currTime - movLog[movLogCount-1].time;
    }
    movLogCount++;
  }
  
  switch (movement)
  {
    case FORWARD:
    {
      /* Set values needed for collision detection */
      currMovement = FORWARD;
      
      Serial.println("Robot moving forwards");
      motors.setSpeeds(MAX_SPEED, MAX_SPEED);
      break; 
    }

    case BACKWARD:
    {
      /* Set values needed for collision detection */
      currMovement = BACKWARD;
      
      Serial.println("Robot moving backwards");
      motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
      break;
    }

    case LEFT:
    {
      /* Set values needed for collision detection */
      currMovement = LEFT; 
      
      Serial.println("Robot turning left");
      motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
      break;
    }

    case RIGHT:
    {
      /* Set values needed for collision detection */
      currMovement = RIGHT;
      
      Serial.println("Robot turning right");
      motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
      break;
    }

    case NONE:
    {
      Serial.println("Robot stopping");
      motors.setSpeeds(0, 0);
    
      /* Set values needed for collision detection */
      currMovement = NONE;
      break;
    } 
  }
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
  if (found == false)
    Serial.println("No objects found.");
  return found;
}

bool correctPath()
{
  static const unsigned long CORR_INTERVAL = 50;
  static const int LEFT_SENSOR_START = 0;
  static const int LEFT_SENSOR_END = 1;
  static const int RIGHT_SENSOR_START = 4;
  static const int RIGHT_SENSOR_END = 5;
  static const int LINE_VALUE = 400;

  static unsigned long lastRun = 0;
  bool result = false;
  MOVEMENT lastMovement;

  /* Corrects the path to stop a side from accidentally going on a wall */

  /* Correction happens every 50ms */
  if ((lastRun == 0) || (millis() - lastRun > CORR_INTERVAL))
  {
    lastRun = millis();
    
    
    if (isSensorsOver(LEFT_SENSOR_START, LEFT_SENSOR_END) > 0)
    {
      motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
      do
      {
        
      } while (isSensorsOver(LEFT_SENSOR_START, LEFT_SENSOR_END) > 0);
      motors.setSpeeds(MAX_SPEED, MAX_SPEED);
      
      /* If far left sensor on a line correct path a little */
      Serial.println("Corrected left");
      result = true;
    }
    else if (isSensorsOver(RIGHT_SENSOR_START, RIGHT_SENSOR_END) > 0)
    {
      motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
      do
      {
        
      } while (isSensorsOver(RIGHT_SENSOR_START, RIGHT_SENSOR_END) > 0);
      motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  
      /* If far right sensor on a line correct path a little */
      Serial.println("Corrected right.");
      result = true;
    }
  }

  return result;
}

bool isWallFound()
{
  static const int START_SENSOR = 2;
  static const int END_SENSOR = 3;

  int sensorCount = isSensorsOver(START_SENSOR, END_SENSOR);

  /* Return true if more than one sensor detects a line */
  return (sensorCount>1);
}

int isSensorsOver(int startSensor, int endSensor)
{
  static const int LINE_VALUE = 400;
  
  int sensorCount = 0;
  unsigned int sensorValues[NUM_SENSORS];
  int i;

  reflectanceSensors.readLine(sensorValues);
  
  for (i = startSensor; i <= endSensor; i++)
  {
//    Serial.print(sensorValues[i]);
//    Serial.print(' ');
    
    if (sensorValues[i] >= LINE_VALUE)
      sensorCount++;
  }
//  Serial.println();

  return sensorCount;
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

