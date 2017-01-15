# ZumoSearch
Zumo search and rescue robot using Arduino Uno.

Developed by Jack Allister (23042098)

### Doxygen
To access the doxygen output of the project please navigate to: 
*doxygen/html/index.html*

# Functionality
* Task 1 - Wireless guided navigation.
* Task 2 - Path correction using reflectance sensors.
* Task 3 - Wall detection to stop the robot just before.
* Task 4 - Room search movement, allows the robot to move into a room for search.
* Task 5 - Uses the ultrasonic sensor to search a room for objects
* Task 6 - Robot can move autonomously to the begin, only searching rooms where objects were found.
* Task 7 - Not completed/implemented.

## Wireless communication/movement.
The robot is able to move wirelessly using the XBee. This allows the robot
to be controlled via simple terminally such as uComm or PuTTY using the WASD keys.

Important information is also sent from the robot back to the terminal such as found objects, movement etc.

## Path correction
Path correction is completed by using the reflectance sensors at the front-bottom of the robot.

The algorithm for detecting a wall is simple.

The robot checks every 50ms to see if a wall is detected on one of the outer reflectance sensors.

If so the robot stops it current motion, rotates away from the line and then continues it prior movement.

## Wall detection
Wall detection is implemented by checking the reflectance sensors as well.

If more than one reflectance sensor detects a wall/line boundary then the robot is stopped. This then allows the user to navigate the robot away from the wall (unless in autonomous mode).

## Room Searching
Room searching is implemented by using the ultrasonic sensors. At the moment a maximum range of 20cm from the robot is able to be searched, however this can be changed easily by changing one variable. Refer to checkForObject procedure for more information.

## Autonomous navigation
Autonomous navigation is implemented by using a log of stored movements that
are taken while in GUIDED_NAVIGATE and SEARCH_ROOM mode. Wall sensing and path correction are both implemented within this mode so the robot should not be able to leave the specific boundaries of the course.

During the room searches in autonomous navigation, the robot will not search rooms that were found to be empty in user guided searching. This is a form of optimisation that would be crucial in a real world application of search and rescue.

### Credits/References
* NewPing library for ultrasonic sensor (Tim Eckel - teckel@leethost.com)
