#include <stdio.h>
#include <stdlib.h>
// Tower of Hanoi, iterative version
// Rick Eason

void main(void) {
	int current[5] = {0,0,0,0,0};	// initially all on pin 0
	int desired[5] = {1,1,1,1,1};	// only last really needs to be initialized
	int i;

	while (1) {
		for (i=4; i>0; i--) {						// work up deciding desired positions
			if (current[i] == desired[i]) 
				desired[i-1] = desired[i];			// previous disk should be on top
			else
				desired[i-1] = 3 - desired[i] - current[i];	// previous disk out of way
		}

		for (i=0; i<5; i++) {					// work down to first not in desired position
			if (desired[i] != current[i]) break;
		}
		if (i==5) break;					// break out if all are in desired position

/*		cout << "Moving Disk " << i;		// print this move
		cout << " from Pin " << current[i] ;
		cout << " to Pin "   << desired[i] << endl;*/
		printf("Moving Disk %d", i);
		printf(" from Pin %d", current[i]);
		printf(" to Pin %d \n", desired[i]);
		current[i] = desired[i];
	}
}
