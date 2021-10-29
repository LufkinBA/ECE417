/* ECE 417 Forward Kinematics Lab 3
 * Blake Lufkin 
 * Labvolt all in one forward kinematics subroutine
 *      di  thetai  ai  alphai
 * 1    27.2    t1  0       90
 * 2    0       t2 19.2     180
 * 3    0       t3  19.2    0
 * 4    0   t4+90   0       90
 * 5    10.5    t5  0       0
 * 
 * Uses the above joint parameter table in conjunction with 
 * the formula for finding the 4x4 homogeneous matrix from
 * a joint parameter table. The 5 resulting homogenous matrices
 * are then multiplied in order from 0 to 4 to gain the final
 * Translation matrix T 5 with respect to 0, stored in result. 
 */
#include <stdio.h>
#include "matrix.c"

void main() 
{

	double thetai[5]; 
    double T[5][4][4], result[4][4];
    double tmp1[4][4], tmp2[4][4], tmp3[4][4];
    int x; 
    double di[5]= {27.2, 0, 0, 0, 10.5};
    double ai[5]= {0, 19.2, 19.2, 0, 0};
    
    //alphai expressed in radians
    double alphai[5]= {1.5707963, 3.14159265, 0, 1.5707963, 0};
    
    for (x=0;x<5;x++) {
        printf("Please enter theta %d: ",x+1);
        scanf("%lf", &thetai[x]);
        //convert decimal value to radians
        if (x == 3) 
            thetai[x] = thetai[x] + 90;
        thetai[x] = thetai[x] * (3.14159265/180);
    }
	for (x = 0; x<5; x++ ) {
		joint_to_T(di[x],thetai[x], ai[x], alphai[x], T[x]);
	}

	t_mult(T[0], T[1], tmp1);

    t_mult(tmp1, T[2], tmp2);

    t_mult(tmp2, T[3], tmp3);

    t_mult(tmp3, T[4], result);

    print_tmatrix(result);

}
