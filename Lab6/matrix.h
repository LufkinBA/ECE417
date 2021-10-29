/* Blake Lufkin
 * matrix.h
 * header file for all the matrix.c functions
 */
#ifndef MATRIX_H
#define MATRIX_H
#include <stdio.h>
#include <stdlib.h>



void t_mult(double m1[4][4], double m2[4][4], double calc[4][4]);

void t_point(double m1[4][4], double m2[4], double calc[4]);

void populate(double data[4][4]);

void point_pop(double point[4]);

void print_point(double point[4]);

void t_transpose(double m1[4][4], double trans[4][4]);

void print_tmatrix(double data[4][4]);

void joint_to_T(double d, double th, double a, double al, double T[4][4]);

void forward_kin(double result[4][4]);

void inverse_kin(double T[4][4], double theta[5]);

void move_theta(double theta[5]);

void move_xyz(double x, double y, double z);

void move_straight(double x, double y, double z);
#endif
