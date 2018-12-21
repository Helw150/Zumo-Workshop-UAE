#include <Wire.h>
#include <ZumoShield.h>
#include "Adafruit_TCS34725.h"

/* This example uses the Zumo Reflectance Sensor Array
 * to navigate a black line maze with no loops. This program
 * is based off the 3pi maze solving example which can be
 * found here:
 *
 * http://www.pololu.com/docs/0J21/8.a
 * 
 * The Zumo first calibrates the sensors to account
 * for differences of the black line on white background.
 * Calibration is accomplished in setup().
 *
 * In loop(), the function solveMaze() is called and navigates
 * the Zumo until it finds the finish line which is defined as
 * a large black area that is thick and wide enough to
 * cover all six sensors at the same time.
 * 
 * Once the Zumo reaches the finishing line, it will stop and
 * wait for the user to place the Zumo back at the starting
 * line. The Zumo can then follow the shortest path to the finish
 * line.
 *
 * The macros SPEED, TURN_SPEED, ABOVE_LINE(), and LINE_THICKNESS 
 * might need to be adjusted on a case by case basis to give better 
 * line following results.
 */

// SENSOR_THRESHOLD is a value to compare reflectance sensor
// readings to to decide if the sensor is over a black line
#define SENSOR_THRESHOLD 300

// ABOVE_LINE is a helper macro that takes returns
// 1 if the sensor is over the line and 0 if otherwise
#define ABOVE_LINE(sensor)((sensor) > SENSOR_THRESHOLD)

// Motor speed when turning. TURN_SPEED should always
// have a positive value, otherwise the Zumo will turn
// in the wrong direction.
#define TURN_SPEED 115

// Motor speed when driving straight. SPEED should always
// have a positive value, otherwise the Zumo will travel in the
// wrong direction.
#define SPEED 115

// Thickness of your line in inches
#define LINE_THICKNESS .75 

// When the motor speed of the zumo is set by
// motors.setSpeeds(200,200), 200 is in ZUNITs/Second.
// A ZUNIT is a fictitious measurement of distance
// and only helps to approximate how far the Zumo has
// traveled. Experimentally it was observed that for 
// every inch, there were approximately 17142 ZUNITs.
// This value will differ depending on setup/battery
// life and may be adjusted accordingly. This value
// was found using a 75:1 HP Motors with batteries
// partially discharged.
#define INCHES_TO_ZUNITS 17142.0

// When the Zumo reaches the end of a segment it needs
// to find out three things: if it has reached the finish line,
// if there is a straight segment ahead of it, and which
// segment to take. OVERSHOOT tells the Zumo how far it needs
// to overshoot the segment to find out any of these things.
#define OVERSHOOT(line_thickness)(((INCHES_TO_ZUNITS * (line_thickness)) / SPEED))

ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_4X);
Pushbutton button(ZUMO_BUTTON);
uint16_t baseR, baseG, baseB;
int color = -1;
void setup()
{

  unsigned int sensors[6];
  unsigned short count = 0;
  unsigned short last_status = 0;
  int turn_direction = 1;  
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  baseR = r;
  baseG = g;
  baseB = b;
  buzzer.play(">g32>>c32");
  Serial.begin(9600);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  reflectanceSensors.init();
  
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);	 // turn on LED to indicate we are in calibration mode
   
  button.waitForButton();
  
  // Calibrate the Zumo by sweeping it from left to right
  for(int i = 0; i < 4; i ++)
  {
    // Zumo will turn clockwise if turn_direction = 1.
    // If turn_direction = -1 Zumo will turn counter-clockwise.
    turn_direction *= -1;
	
    // Turn direction.
    motors.setSpeeds(turn_direction * TURN_SPEED, -1*turn_direction * TURN_SPEED);
      
    // This while loop monitors line position
    // until the turn is complete. 
    while(count < 2)
    {
      reflectanceSensors.calibrate();
      reflectanceSensors.readLine(sensors);
      if(turn_direction < 0)
      {
	// If the right	 most sensor changes from (over white space -> over 
	// line or over line -> over white space) add 1 to count.
	count += ABOVE_LINE(sensors[5]) ^ last_status;
	last_status = ABOVE_LINE(sensors[5]);
      }
      else
      {
	// If the left most sensor changes from (over white space -> over 
	// line or over line -> over white space) add 1 to count.
	count += ABOVE_LINE(sensors[0]) ^ last_status;
	last_status = ABOVE_LINE(sensors[0]);	  
      }
    }
  
    count = 0;
    last_status = 0;
  }
  
  // Turn left.
  turn('L');
  
  motors.setSpeeds(0, 0);
  
  // Sound off buzzer to denote Zumo is finished calibrating
  buzzer.play("L16 cdegreg4");
  
  // Turn off LED to indicate we are through with calibration
  digitalWrite(13, LOW);
}

void loop()
{

  // solveMaze() explores every segment
  // of the maze until it finds the finish
  // line.
  solveMaze();
  
  // Sound off buzzer to denote Zumo has solved the maze
  buzzer.play(">>a32");
}

// Turns according to the parameter dir, which should be 
// 'L' (left), 'R' (right), 'S' (straight).
void turn(char dir)
{

  // count and last_status help
  // keep track of how much further
  // the Zumo needs to turn.
  unsigned short count = 0;
  unsigned short last_status = 0;
  unsigned int sensors[6];
  color = -1;
  // dir tests for which direction to turn
  switch(dir)
  {
  
  // Since we're using the sensors to coordinate turns instead of timing them, 
  // we can treat a left turn the same as a direction reversal: they differ only 
  // in whether the zumo will turn 90 degrees or 180 degrees before seeing the 
  // line under the sensor.
    case 'L':
      // Turn left.
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      
      // This while loop monitors line position
      // until the turn is complete. 
      while(count < 2)
      {
	reflectanceSensors.readLine(sensors);
		
	// Increment count whenever the state of the sensor changes 
		// (white->black and black->white) since the sensor should 
		// pass over 1 line while the robot is turning, the final 
		// count should be 2
	count += ABOVE_LINE(sensors[1]) ^ last_status; 
	last_status = ABOVE_LINE(sensors[1]);
      }
    
    break;
    
    case 'R':
      // Turn right.
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      
      // This while loop monitors line position
      // until the turn is complete. 
      while(count < 2)
      {
	reflectanceSensors.readLine(sensors);
	count += ABOVE_LINE(sensors[4]) ^ last_status;
	last_status = ABOVE_LINE(sensors[4]);
      }
    
    break;
	
    case 'S':
    // Don't do anything!
    break;
  }
}

int detectColor(){
    uint16_t r, g, b, c, colorTemp, lux;
    tcs.getRawData(&r, &g, &b, &c);
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print("\n");
    if(r < baseR*0.9 || b < baseB*0.9 || g < baseG*0.9)
    {
    	if(r > g && r > b){
    	    Serial.print("Red");
      	    return 0;
    	} else {
    	    Serial.print("Green");
    	    return 1;
    	}
    }
    return -1;
} 

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char selectTurn(unsigned char found_left,
		unsigned char found_right,
		int localColor)
{
  // Make a decision about how to turn.	 The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if(found_left && localColor == 1)
    return 'L';
  else if(found_right)
    return 'R';
  else
    return 'S';
}

// The maze is broken down into segments. Once the Zumo decides
// which segment to turn on, it will navigate until it finds another
// intersection. followSegment() will then return after the
// intersection is found.
void followSegment()
{
  unsigned int position;
  unsigned int sensors[6];
  int offset_from_center;
  int power_difference;
  
  while(1)
  {	
    // Get the position of the line.
    position = reflectanceSensors.readLine(sensors);
     
    // The offset_from_center should be 0 when we are on the line.
    offset_from_center = ((int)position) - 2500;
     
    // Compute the difference between the two motor power settings,
    // m1 - m2.	 If this is a positive number the robot will turn
    // to the left.  If it is a negative number, the robot will
    // turn to the right, and the magnitude of the number determines
    // the sharpness of the turn.
    power_difference = offset_from_center / 3;
     
    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    if(power_difference > SPEED)
      power_difference = SPEED;
    if(power_difference < -SPEED)
      power_difference = -SPEED;
     
    if(power_difference < 0)
      motors.setSpeeds(SPEED + power_difference, SPEED);
    else
      motors.setSpeeds(SPEED, SPEED - power_difference);
     
    // We use the inner four sensors (1, 2, 3, and 4) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 5 for detecting lines going to the left and
    // right.
     
    if(!ABOVE_LINE(sensors[0]) && !ABOVE_LINE(sensors[1]) && !ABOVE_LINE(sensors[2]) && !ABOVE_LINE(sensors[3]) && !ABOVE_LINE(sensors[4]) && !ABOVE_LINE(sensors[5]))
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.	Must be a dead end.	       
      return;
    }
    else if(ABOVE_LINE(sensors[0]) || ABOVE_LINE(sensors[5]))
    {
      // Found an intersection.
      return;
    }
   
  }
}

// The solveMaze() function works by applying a "left hand on the wall" strategy: 
// the robot follows a segment until it reaches an intersection, where it takes the 
// leftmost fork available to it. It records each turn it makes, and as long as the 
// maze has no loops, this strategy will eventually lead it to the finish. Afterwards, 
// the recorded path is simplified by removing dead ends. More information can be 
// found in the 3pi maze solving example.
void solveMaze()
{
    while(1)
    {
	// Navigate current line segment
	followSegment();

	// These variables record whether the robot has seen a line to the
	// left, straight ahead, and right, while examining the current
	// intersection.
	unsigned char found_left = 0;
	unsigned char found_right = 0;
  motors.setSpeeds(0,0);
	if(color == -1){
	    color = detectColor();
	}
	// Now read the sensors and check the intersection type.
	unsigned int sensors[6];
	reflectanceSensors.readLine(sensors);
	 
	// Check for left and right exits.
	if(ABOVE_LINE(sensors[0]))
	    found_left = 1;
	if(ABOVE_LINE(sensors[5]))
	    found_right = 1;
	    
	// Drive straight a bit more, until we are
	// approximately in the middle of intersection.
	// This should help us better detect if we
	// have left or right segments.
	motors.setSpeeds(SPEED, SPEED);
	delay(OVERSHOOT(LINE_THICKNESS)/2);

	reflectanceSensors.readLine(sensors);
	
	// Check for left and right exits.
	if(ABOVE_LINE(sensors[0]))
	    found_left = 1;
	if(ABOVE_LINE(sensors[5]))
	    found_right = 1;
	motors.setSpeeds(0,0);
	// After driving a little further, we
	// should have passed the intersection
	// and can check to see if we've hit the
	// finish line or if there is a straight segment
	// ahead.   
	delay(OVERSHOOT(LINE_THICKNESS)/2);
	if(color == -1){
	    color = detectColor();
	}
	// Check for a straight exit.
	reflectanceSensors.readLine(sensors);
	
	// Check again to see if left or right segment has been found
	if(ABOVE_LINE(sensors[0]))
	    found_left = 1;
	if(ABOVE_LINE(sensors[5]))
	    found_right = 1;
		 
// 	// Check for the ending spot.
// 	// If all four middle sensors are on dark black, we have
// 	// solved the maze.
// 	if(ABOVE_LINE(sensors[1]) && ABOVE_LINE(sensors[2]) && ABOVE_LINE(sensors[3]) && ABOVE_LINE(sensors[4]))
// 	{
// 	  motors.setSpeeds(0,0);
// 	  break;
// 	}
	 
	// Intersection identification is complete.
	unsigned char dir = selectTurn(found_left, found_right, color);
	
	// Make the turn indicated by the path.
	turn(dir);	   
    }
}
