#include "labvolt.h"
#include <stdio.h>
#include <stdlib.h>
#include "matrix.c"


void main()
{
	double theta[5];
	char affirm='y';
	int x; 

	init();
	moverel(0,-910,-10,-200,0);
	zero();

	while(affirm!='n') {
		
		printf("Enter the joint angles \n");
		
		for (x = 0; x < 5; x++) {
			printf("Please enter theta %d: ",x+1);
			scanf("%lf", &theta[x]);
		}
		
		move_theta(theta);

		printf("Would you like to move again? y or n?\n");
		scanf("%s",&affirm);
	}
	
	shutdown();

	
	return;
}
