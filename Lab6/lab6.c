/* Lab6 ECE417
 * Blake Lufkin
 * lab6.c
 * The purpose of this program is to move to a specified point, open the gripper,
 * move down, close the gripper, move back up, move in a straight line to another
 * specified point, move down, open gripper, move up, close the gripper then return home
 */
#include "labvolt.h"
#include <stdio.h>
#include <stdlib.h>
#include "matrix.c"

/* random values inserted to compile
 * actuall values will be computed and inserted in Lab
 */

//Values that are to be the initial movement location
#define init_x  (23)
#define init_y  (23)
#define init_z  (11)


int main()
{
	double target[3];
	int i;

	printf("Enter x, y, and z of the target position\n"); 
	for (i = 0; i < 3; i++) {
		scanf("%lf", &target[i]);
	}	
	
	//initialize the robot and move to 0,90,90,0,0 and set that as home
	init();
	moverel(0,-910,-10,-200,0); 
	zero();

	
	//move to initial position then open gripper 
	move_xyz(init_x, init_y, init_z); 
	gripperOpen();

	//move down and close gripper assuming 10 cm drop
	move_straight(init_x, init_y, init_z-10); 
	gripperClose();
	//TODO DELAY HERE?
	move_straight(init_x, init_y, init_z);

	//move straight to target position
	move_straight(target[0], target[1], target[2]); 

	//move down and release item assuming 10cm drop
	move_straight(target[0], target[1], target[2]-10); 
	gripperOpen();
	//TODO DELAY HERE?	

	//move back to target then return home
	move_straight(target[0], target[1], target[2]);
	nest();  
	
	shutdown();
	return 0; 
}
