/*ECE 417 Matrix multiplication subroutines 
 * matrix.c
 *Blake Lufkin
 *First subroutine is to multiply two 4x4 matrices together
 *Second subroutine is to multiply the R in a 4x4 by a point  
 *Puvw to gain Pxyz, the output will be a 3x1 matrix.
 *Third subroutine computes the inverse of a homogenous transformation 
 *matrix. 
 * The rest of the subroutines are for printing and populating the different kinds
 * of matricies. 
 * Also includes a forward kinematics subroutine as well as an inverse kinematics 
 * subroutine 
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"

#define init_x  (23)
#define init_y  (23)
#define init_z  (11)

#define e(a) (((a)/360.0) * 8600)
#define h(a) (((a)/360.0) * 6500)


/* Functions purpose is to multiply two 4x4 matricies. 
 * m1 is multiplied by m2 and stored in calc 
 */
void t_mult(double m1[4][4], double m2[4][4], double calc[4][4])
{
	int a, b, i;
	double tmp = 0;
		
	for (i=0;i<4;i++) {
		for (a=0;a<4;a++) {
			for (b=0;b<4;b++) {
				tmp = tmp + (m1[i][b] * m2[b][a]);
			}
			calc[i][a] = tmp;
			tmp = 0;
		}
	}
	return; 
}

/* Finds the point calc in relation to m2 by using m1 as the rotation
 * matrix.
 */

void t_point(double m1[4][4], double m2[4], double calc[4])
{
	
	int a, i;
	double tmp = 0;
	
	for (i=0;i<4;i++) {
		for (a=0;a<4;a++) {
			tmp = tmp + (m1[i][a] * m2[a]);
		}
		calc[i] = tmp;
		tmp = 0;
	}
	return; 
}

/* Finds the transpose of a 4x4 transformation matrix 
 * ra na sa ta
 * rb nb sb tb
 * rc nc sc tc
 * 0  0  0	1
 * finds the transpose of the 3x3 contained by r, n, and s
 * then multiples that transposed 3x3 by ta, tb, tc to get the transpose of
 * the position vector based on the formulas:
 * R'=R^-1  and   t'= -R't
 */
void t_transpose(double m1[4][4], double trans[4][4])
{
	int a, b;
	double tmp = 0;
	

	for(a=0;a<3;a++) {
		for(b=0;b<3;b++) {
			trans[b][a] = m1[a][b];
		}
	}
	
	for(a=0;a<4;a++){
		for(b=0;b<4;b++) {
			tmp = tmp + (-1 * trans[a][b] * m1[b][3]);
		}
		trans[a][3] = tmp;
		tmp = 0; 
	}
	
	for (a=0;a<4;a++) {
		trans[3][a] = m1[3][a];
	}
	
	return;
}

//Populates a 4x4 matrix with user prompts
void populate(double data[4][4])
{
	int a, b;
	double tmp;
	
	for (a=0;a<4;a++){
		for(b=0;b<4;b++) {
			printf("Enter value for matrix[%d][%d]: \n",a+1,b+1);
			scanf("%lf",&tmp);
			data[a][b] = tmp;
		}
	}
	return;
}

//Populates a point(3x1) with user prompts
void point_pop(double point[4])
{
	int a;
	double tmp;

	for (a=0;a<3;a++) {	
		printf("Enter value for matrix[%d][1]: \n",a+1);
		scanf("%lf",&tmp);
		point[a] = tmp;
	}
	point[3] = 1;

}

//print function for a 4x4 matrix
void print_tmatrix(double data[4][4])
{
	int a, b;
	for(a=0;a<4;a++) {
		for(b=0;b<4;b++) {
			printf("%f ",data[a][b]);
		}
		printf("\n");
	}
	return;
}

//print function for a 3x1 matrix
void print_point(double point[4]) {
	
	int i; 
	for (i=0;i<3;i++){
		printf("%f\n",point[i]);
	}
	return;	
}

//takes in joint parameters and creates/populates a 4x4 homogeneous matrix 
void joint_to_T(double d, double th, double a, double al, double T[4][4])
{
	//brute force each individual assignment	
	//not elegant but it works
	T[0][0] = cos(th); 
	T[0][1] = -sin(th) * cos(al);
	T[0][2] = sin(th) * sin(al);
	T[0][3] = a * cos(th);
	T[1][0] = sin(th);
	T[1][1] = cos(th) * cos(al); 
	T[1][2] = -cos(th) * sin(al);
	T[1][3] = a * sin(th); 
	T[2][0] = 0;
	T[2][1] = sin(al);
	T[2][2] = cos(al);
	T[2][3] = d;
	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][3] = 1;
}


/* Labvolt all in one forward kinematics subroutine
 *		di	thetai	ai	alphai
 * 1	27.2	t1	0		90
 * 2	0		t2 19.2		180
 * 3	0		t3	19.2	0
 * 4	0	t4+90	0		90
 * 5	10.5	t5	0		0
 */
void forward_kin(double result[4][4])
{
	double thetai[5]; 
	double T[5][4][4];
	double tmp1[4][4], tmp2[4][4], tmp3[4][4];
	int x; 
	double di[5]= {27.2, 0, 0, 0, 10.5};
	double ai[5]= {0, 19.2, 19.2, 0, 0};
	
	//al = alpha expressed in radians
	double alphai[5]= {1.5707963, 3.14159265, 0, 1.5707963, 0};
	
	for (x=0;x<5;x++) {
		printf("Please enter theta %d: ",x+1);
		scanf("%lf", &thetai[x]);
		//convert decimal value to radians
		if (x == 3)	
			thetai[x] = thetai[x] + 90;	
		thetai[x] = thetai[x] * (3.14159265/180); 
	}
	
	//populate matricies 
	for (x=0;x<5;x++) {
		joint_to_T(di[x], thetai[x], ai[x], alphai[x], T[x]);
	}

	t_mult(T[0], T[1], tmp1);
	
	t_mult(tmp1, T[2], tmp2);
	
	t_mult(tmp2, T[3], tmp3);
	
	t_mult(tmp3, T[4], result);

	print_tmatrix(result);
	

	
}

/* performs inverse kinematics to determine theta1-5 based on T 5 w.r.t. 0 
 * Takes a 4x4 matrix and computes the required 5 thetas for the Labvolt robot
 */
void inverse_kin(double T[4][4], double theta[5]) 
{
	double x, y, z, d1=27.2, d5 = 10.5, pi=3.14159265; 
	double a, g; 
	double a2 = 19.2 , a3 = 19.2, th3, th2, th1, tmp;
	double A30[4][4], A35[4][4];
	int i;

	//define x, y, and z based off known values
	x = T[0][3] - (d5*T[0][2]);
	y = T[1][3] - (d5*T[1][2]);
	z = T[2][3] - (d5*T[2][2]);

	//Compute theta1 based off equation
	theta[0] = atan2(y, x); 

	//define a and z, called g in this function as not to confuse 
	a = sqrt(pow(x,2) + pow(y,2));
	g = z - d1;
	
	tmp = (pow(a,2) + pow(g,2) - pow(a2,2) - pow(a3,2))/(2*a2*a3);
	
	//this is here because of weird rounding issues i.e. acos(1.0000000) would be nan
	if (tmp>1 || tmp < -1) tmp = round(tmp); 
	
	//convert the value for this equation to radians before taking the arc cos
	theta[2] = acos(tmp);

	th3 = (theta[2]);
	
	//compute theta2
	theta[1] = atan2(g, a) + atan2(a3*sin(th3),(a2 + a3 * cos(th3)));

	th2 = theta[1];
	th1 = theta[0]; 
	
	//populate A30
	A30[0][0] = cos(th1) * cos(th3-th2);
	A30[0][1] = sin(th1) * cos(th3-th2);
	A30[0][2] = -sin(th3-th2);
	A30[0][3] = -a3 - a2*cos(th3) + d1*sin(th3-th2);
	A30[1][0] = -cos(th1) * sin(th3-th2);
	A30[1][1] = -sin(th1) * sin(th3-th2);
	A30[1][2] = -cos(th3-th2);
	A30[1][3] = a2 * sin(th3) + d1*cos(th3-th2);
	A30[2][0] = -sin(th1);
	A30[2][1] = cos(th1);
	A30[2][2] = 0;
	A30[2][3] = 0;
	A30[3][0] = 0;
	A30[3][1] = 0;
	A30[3][2] = 0;
	A30[3][3] = 1;
	
	//compute 3A5
	t_mult(A30,T,A35);
	
	//compute theta 4 and theta 5 with atan2 of calcualted variables
	theta[3] = atan2(A35[1][2], A35[0][2]);
	theta[4] = atan2(A35[2][0], A35[2][1]);

	//print theta values
	for (i = 0; i < 5; i++) {
		theta[i] = theta[i] * (180/pi);
	//	printf("Theta %d is: %lf \n", i+1, theta[i]); 
	}
	return;
}

//Subroutine to move from one set of joint angles to another
/* Some useful information:
 * Base      6500 steps/revolution + = CCW          ~18.06 steps per degree 
 * Shoulder  8600 steps/revolution + = up/back      ~23.9 steps per degree
 * Elbow     8600 steps/revolution + = down         ~23.9 steps per degree
 * M4        6500 steps/revolution (see note below) ~18.06 steps per degree
 * M5        6500 setps/revolution                  ~18.06 steps per degree
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

void move_theta(double theta[5]) 
{
	//starts at (0,90,90,0,0)
	static int steps_from_home[5] = {0, e(90), e(90), 0, 0};
	int new_steps[5], new_steps_delta[5], x;

	//put theta[x] in terms of steps
	for (x = 0;x<5;x++) { 
		if (x==1 || x==2) {
			new_steps[x] = e(theta[x]);
		} else {
			new_steps[x] = h(theta[x]);
		}
	}
	
	//new_steps = new_steps - steps_from_home
	for (x=0;x<5;x++) {
		new_steps_delta[x] = new_steps[x] - steps_from_home[x]; 
	}
	
	
	moverel(new_steps_delta[0],new_steps_delta[1],new_steps_delta[2]-new_steps_delta[1],(new_steps_delta[3]-new_steps_delta[4]+(6500.0/8600.0)*(new_steps_delta[2]-new_steps_delta[1])),(-new_steps_delta[3]-new_steps_delta[4]+(6500.0/8600.0)*(new_steps_delta[1]-new_steps_delta[2]))); 
	
	for (x=0;x<5;x++) {
		steps_from_home[x]=new_steps[x]; 
	}
	
	return;
}

/* A subroutine that moves the robot to the desired position with the gripper
 * pointing downward
 */
void move_xyz(double x, double y, double z)
{
	double T[4][4];
	double theta[5]; 

	//convert x, y, z to a rotation matrix
	T[0][0] = -1;
	T[0][1] = 0;
	T[0][2] = 0;
	T[0][3] = x; 
	T[1][0] = 0;
	T[1][1] = 1;
	T[1][2] = 0;
	T[1][3] = y; 
	T[2][0] = 0;
	T[2][1] = 0;
	T[2][2] = -1;
	T[2][3] = z; 
	T[3][0] = 0;
	T[3][1] = 0;
	T[3][2] = 0;
	T[3][3] = 1;
	
	//calls inverse kinematic function to solve for theta values
	inverse_kin(T,theta); 

	//moves to theta values using move_theta()
	move_theta(theta); 
	
	return; 
}

/* A subroutine in which the robot moves from the current point
 * to a desired point in a straight line. Function gets passed the
 * coordinates of the new point and assumes that initial position values
 * have been established globally in main. i.e. #define init_x = ??;  
 */

void move_straight(double x, double y, double z)
{
	//Step is an xyz vector
	double step[3], diffsq[3], tmpx, tmpy, tmpz, tmpsq, length;
	static double current_x = init_x;
	static double current_y = init_y;
	static double current_z = init_z; 
	int i; 

	tmpx = x - current_x;
	tmpy = y - current_y; 
	tmpz = z - current_z; 

	diffsq[0] = tmpx * tmpx;
	diffsq[1] = tmpy * tmpy;
	diffsq[2] = tmpz * tmpz;
	
	tmpsq = diffsq[0] + diffsq[1] + diffsq[2]; 
	
	//find length of vector
	length = sqrt(tmpsq); 

	printf("Length of the distance is: %lf\n", length);

	//define the unit vector of the movement 
	step[0] = (tmpx/length);
	step[1] = (tmpy/length);
	step[2] = (tmpz/length); 
	
	printf("Step lengths are x: %lf, y: %lf, z: %lf \n", step[0], step[1], step[2]);

	//move the unit vector however many times is reqired defined by length
	for (i = 0; i < length; i++) {
		current_x += step[0];
		current_y += step[1];
		current_z += step[2]; 
		printf("current position is %lf, %lf, %lf \n", current_x, current_y, current_z);
		move_xyz(current_x, current_y, current_z); 
	}
	
	//finish moving any distance not a whole cm
	move_xyz(x, y, z); 

	//set current position to equal the target position
	current_x = x;
	current_y = y;
	current_z = z; 
}























