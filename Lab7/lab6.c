/* Lab7 ECE417
 * Blake Lufkin
 * lab7.c
 * The purpose of this lab is to use the labvolt robot
 * to play Towers of Hanoi with a stack of 5 blocks
 */
#include "labvolt.h"
#include <stdio.h>
#include <stdlib.h>
#include "matrix.c"

/* random values inserted to compile
 * actuall values will be computed and inserted in Lab
 */

//Values that are to be the initial movement location
#define pos0_x  (23)
#define pos0_y  (23)

#define pos1_x (23)
#define pos1_y (-23)

#define pos2_x (23)
#define pos2_y (0)


int main()
{
    int current[5] = {0,0,0,0,0};   // initially all on pin 0
    int desired[5] = {1,1,1,1,1};   // only last really needs to be initialized
    int i, tmpz;
	unsigned int pos[3];
	
	//initialize the robot and move to 0,90,90,0,0 and set that as home
	init();
	moverel(0,-910,-10,-200,0); 
	zero();
	gripperOpen();
	
	//define the amount of blocks in each position
	pos[0] = 5;
	pos[1] = 0;
	pos[2] = 0; 
	
	//define position 0 for block spots spots. comment out after
	move_xyz(pos0_x, pos0_y,5);
	gripperClose();
	gripperOpen();

	//This while loop is from Richard Eason's Tower of Hanoi iterative program
    while (1) {
        for (i=4; i>0; i--) {                       // work up deciding desired positions
            if (current[i] == desired[i])
                desired[i-1] = desired[i];          // previous disk should be on top
            else
                desired[i-1] = 3 - desired[i] - current[i]; // previous disk out of way
        }

        for (i=0; i<5; i++) {                   // work down to first not in desired position
            if (desired[i] != current[i]) break;
        }
        if (i==5) break;                    // break out if all are in desired position

        printf("Moving Disk %d", i);		//print this move
        printf(" from Pin %d", current[i]);
        printf(" to Pin %d \n", desired[i]);
        
		//Labvolt Robot Code goes here
		/* determine which pile it is grabbing from
		 * and grab that block from the pile while
		 * always returning to the height of 15 to avoid the other blocks
		 */
		if (current[i] == 0) {
			tmpz = ((pos[0] - 1) * 2) + 1;
			move_xyz(pos0_x, pos0_y, 15);
			move_xyz(pos0_x, pos0_y, tmpz);
			gripperClose();
			move_xyz(pos0_x, pos0_y, 15);
			pos[0]--; 
		} else if (current[i] == 1) {
			tmpz = ((pos[1] - 1) * 2) + 1;
			move_xyz(pos1_x, pos1_y, 15);
			move_xyz(pos1_x, pos1_y, tmpz);
			gripperClose();
			move_xyz(pos1_x, pos1_y, 15);
			pos[1]--; 
		} else {
			tmpz = ((pos[2] - 1) * 2) + 1;
			move_xyz(pos2_x, pos2_y, 15);
			move_xyz(pos2_x, pos2_y, tmpz);
			gripperClose();
			move_xyz(pos2_x, pos2_y, 15);
			pos[2]--; 
		}
		
		//Place the block in the desired spot
		if (desired[i] == 0) {
			tmpz = ((pos[0]) * 2) + 1;
			move_xyz(pos0_x, pos0_y, 15);
			move_xyz(pos0_x, pos0_y, tmpz);
			gripperOpen();
			move_xyz(pos0_x, pos0_y, 15);
			pos[0]++; 
		} else if (desired[i] == 1) {
			tmpz = ((pos[1]) * 2) + 1;
			move_xyz(pos1_x, pos1_y, 15);
			move_xyz(pos1_x, pos1_y, tmpz);
			gripperOpen();
			move_xyz(pos1_x, pos1_y, 15);
			pos[1]++; 
		} else {
			tmpz = ((pos[2]) * 2) + 1;
			move_xyz(pos2_x, pos2_y, 15);
			move_xyz(pos2_x, pos2_y, tmpz);
			gripperOpen();
			move_xyz(pos2_x, pos2_y, 15);
			pos[2]++; 
		}

		current[i] = desired[i];
    }

	
	nest();  
	
	shutdown();
	return 0; 
}
