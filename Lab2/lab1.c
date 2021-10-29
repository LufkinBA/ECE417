/* Lab 1 Labvolt robot movement basics
 * Blake Lufkin
 * In this lab, the goal is to complete movement basics
 * 1) Initialize and zero the robot.
 * 2) Rotate the base 30 degrees counter clockwise (when viewed from above),
 * 3) then rotate the shoulder 20 degrees down (forward),
 * 4) then rotate the elbow 15 degrees in (toward the base),
 * 5) then roll the wrist 20 degrees clockwise (from robot's viewpoint).
 * 6) then pitch the wrist 20 degrees up.
 * 7) then delay 5 seconds using the sleep() command,
 * 8) then close the gripper,
 * 9) and finally return the robot to the home position and shut it down.
 *
 * Some useful information:
 * Base      6500 steps/revolution + = CCW			~18.06 steps per degree 
 * Shoulder  8600 steps/revolution + = up/back		~23.9 steps per degree
 * Elbow     8600 steps/revolution + = down			~23.9 steps per degree
 * M4        6500 steps/revolution (see note below) ~18.06 steps per degree
 * M5        6500 setps/revolution					~18.06 steps per degree
 *
 * +M4 and -M5 will pitch the wrist down
 * +M4 and +M5 will roll the wrist right (CCW from the robot's view)
 *
 * Taken from Lab1 movement basics: 
 * I found that an initial move of 
 * 
 *    moverel(0,-910, -10, -200, 0); 

 * will place the robot which is connected to "einstein" (the robot on the right) 
 * in a position where the "upper arm" is vertical and the lower arm and wrist are horizontal. 
 */

#include "labvolt.h"

int main() 
{
	//Step 1
	init();
	zero(); 	

	//Step 2
	moverel(542, 0, 0, 0, 0);

	//Movements from here on will adjust for the angle at which the previous joint moves
	//Step 3
	moverel(0, -478, 478, 361, -361); 

	//Step 4
	moverel(0, 0, 359, 271, -271);

	//Step 5
	moverel(0, 0, 0, -361, -361); //not sure 

	//Step 6
	moverel(0, 0, 0, -361, 361); //fairly sure


	//Step 8 
	gripperClose();

	//Step 9 
	nest(); 

	//Cleanup
	shutdown();
	return 0; 
}






