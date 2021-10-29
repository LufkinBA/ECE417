#include <math.h>
#include <stdio.h>

void main()
{
	double a, b, ar, br;
	double result1, result2;
	a = 30;
	b = 60;

	ar = a * 3.14159/180;
	br = b * 3.14159/180;
	
	result1 = atan2(a,b);
	result2 = atan2(ar,br);

	printf("Non convert value is: %lf, convert value is: %lf \n", result1, result2);	
}
