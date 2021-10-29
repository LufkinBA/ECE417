//test functions

#include <stdio.h>
#include <stdlib.h>
#include "matrix.c"

int main()
{
	double mat1[4][4];
	double mat2[4][4];
	double r[4][4], r2[4][4], t[4][4];
	double Puvw[4], Pxyz[4];

	//input matrix A
	printf("Matrix A\n");
	populate(mat1);
	print_tmatrix(mat1);
	
	printf("\n");
	printf("\n");
	printf("\n");
	
	//input matrix B
	printf("Matrix B\n");
	populate(mat2);
	print_tmatrix(mat2);
	
	printf("\n");
	printf("\n");
	printf("\n");
	
	//input point Puvw
	printf("point Puvw\n");
	point_pop(Puvw);
	print_point(Puvw);

	printf("\n");
	printf("\n");
		
	//print A times B
	printf("A times B\n");
	t_mult(mat1,mat2,r);
	print_tmatrix(r);

	printf("\n");
	printf("\n");
	
	//print A transpose
	printf("Matrix A Transpose\n");
	t_transpose(mat1,t);
	print_tmatrix(t);

	printf("\n");
	printf("\n");
	
	//print A times Puvw
	printf("Matrix A times Puvw\n");
	t_point(mat1,Puvw,Pxyz);
	print_point(Pxyz);
	
	printf("\n");
	printf("\n");
	
	//print A times A transpose
	printf("Matrix A times A transpose\n");
	t_mult(mat1,t,r2);
	print_tmatrix(r2);

	return 0;
	
}
