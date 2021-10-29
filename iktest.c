//Test function for inverse kinematics 
//ECE417 Blake Lufkin Lab 4

#include <stdio.h>
#include <stdlib.h>
#include "matrix.c"

int main()
{
	double T05[4][4];
	double thetai[5];
	
	forward_kin(T05);
	
	inverse_kin(T05, thetai);
	
	return 0; 
}
