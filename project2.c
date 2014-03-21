/*************************************************************************/
/* File:        Kinematics.c                                             */
/* Description: Kinematic functions for Cartesian control:               */
/*               inv_arm_kinematics() - writes arm cspace setpoints      */
/*               stereo_observation() - (vision.c) computes Cartesian    */
/*               observations (mean, cov), visualize procedure draws     */
/*               observation to canvas                                   */
/* FUNCTION LIBRARY (appended at bottom):                                */
/*    construct_wTb(roger->base_position, wTb) computes the homogeneous  */
/*          transform (double wTb[4][4]) relating the base frame to the  */
/*          to the world frame given input robot state (double x,y,theta)*/
/*    inverse_homogeneous_xform(aTb, bTa) - inverts a homogeneous xform  */
/*    matrix_times_vector(A,x,y) - multiplies double A[4][4] and input   */
/*          double x[4] to yield output double y[4] (y=Ax) - can be used */
/*         to apply homogeneous transforms to homogeneous vectors        */
/* Date:        5-10-2013                                                */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


void draw_observation(), inv_HTransform(), matXvec4441();


/**********************************************************************/
/************* LIBRARY CODE *******************************************/
/**********************************************************************/
void construct_wTb(base_pos, wTb)
double base_pos[3]; // (x,y,theta)
double wTb[4][4];
{
	double s0, c0;
	s0 = sin(base_pos[2]);
	c0 = cos(base_pos[2]);

	wTb[0][0] = c0;  wTb[0][1] = -s0; wTb[0][2] = 0.0; wTb[0][3] = base_pos[X];
	wTb[1][0] = s0;  wTb[1][1] = c0;  wTb[1][2] = 0.0; wTb[1][3] = base_pos[Y];
	wTb[2][0] = 0.0; wTb[2][1] = 0.0; wTb[2][2] = 1.0; wTb[2][3] = 0.0;
	wTb[3][0] = 0.0; wTb[3][1] = 0.0; wTb[3][2] = 0.0; wTb[3][3] = 1.0;
}

/******************* HOMOGENEOUS TRANSFORMS **************************/
void inverse_homogeneous_xform(in, out)
double in[4][4], out[4][4];
{
	int i,j;
	for (i=0; i<3; ++i) {
		for (j=0; j<3; ++j) {
			out[i][j] = in[j][i];
		}
	}
	out[3][0] = out[3][1] = out[3][2] = 0.0; out[3][3] = 1.0;

	out[0][3] = -in[0][3]*in[0][0] - in[1][3]*in[1][0] - in[2][3]*in[2][0];
	out[1][3] = -in[0][3]*in[0][1] - in[1][3]*in[1][1] - in[2][3]*in[2][1];
	out[2][3] = -in[0][3]*in[0][2] - in[1][3]*in[1][2] - in[2][3]*in[2][2];
}

/*********************************************************************/
void matrix_times_vector(mat, in, out)    // out = mat x in
double mat[4][4], in[4], out[4];
{
	int i,j;
	double sum;
	for (i=0; i<4; ++i) {
		sum = 0.0;
		for (j=0; j<4; ++j) {
			sum += mat[i][j] * in[j];
		}
		out[i] = sum;
	}
}

/**********************************************************************/
/************* CARTESIAN ARM CONTROL **********************************/
/**********************************************************************/

int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
	double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

	double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
	double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
	double theta1_plus, theta1_minus;

	//IMPLEMENTED

		//Input position from GUI is reference position in world coordinates
		ref_w[X] = x;
		ref_w[Y] = y;

		//Contruct wTb transform and invert to get bTw transform
		construct_wTb(roger->base_position,wTb);
			
		//Inverse homogenous transform to reverse map
		inverse_homogeneous_xform(wTb,bTw);

		//Transform world coordinate reference to base coordinates
		matXvec4441(bTw, ref_w, ref_b);


		if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
		else ref_b[Y] += ARM_OFFSET;


		//Inverse kinematics procedure
		r2 = (SQR(ref_b[X]) + SQR(ref_b[Y]));
		c2 = (r2-SQR(LARM_1)-SQR(LARM_2))/(2*LARM_1*LARM_2);

		//If goal is in reachable workspace
		if(-1 <= c2 <= 1){
			s2_plus  =      sqrt(1-SQR(c2));
			s2_minus =   -1*sqrt(1-SQR(c2));
			theta2_plus  =  atan2(s2_plus, c2);
			theta2_minus =  atan2(s2_minus, c2);
			k1           =  LARM_1 + LARM_2*c2;
			k2_plus      =  LARM_2*s2_plus;
			k2_minus     =  LARM_2*s2_minus;
			alpha_plus   =  atan2(k2_plus, k1);
			alpha_minus  =  atan2(k2_minus, k1);
			theta1_plus  =  atan2(ref_b[Y], ref_b[X]) - alpha_plus;
			theta1_minus =  atan2(ref_b[Y], ref_b[X]) - alpha_minus;

			//Given the limb, choose the more biological configuration
			//for that arm frame.
			if(limb != LEFT){
				roger->arm_setpoint[limb][0] = theta1_plus;
				roger->arm_setpoint[limb][1] = theta2_plus;
			}
			else{
				roger->arm_setpoint[limb][0] = theta1_minus;
				roger->arm_setpoint[limb][1] = theta2_minus;
			}

			return TRUE;
		}

	//END IMPLEMENTED

	return FALSE;
}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
Observation obs;

/* executed automatically when                                          */
/* control mode = PROJECT2; input mode = ball inputs                    */
void project2_control(roger, time)
Robot * roger;
double time;
{
	int stereo_observation();

	printf("INSIDE PROJECT2_CONTROL():\n");
	// check if ball is in view
	// write Observation "obs" = mean and cov in world coordinates
	if (stereo_observation(roger, &obs)) {          // in vision.c
		printf("stereo_observation(): x=%6.4lf y=%6.4lf\n", obs.pos[X], obs.pos[Y]);
		printf("                        %lf %lf\n", obs.cov[0][0], obs.cov[0][1]);
		printf("                        %lf %lf\n\n",obs.cov[1][0],obs.cov[1][1]);

	}
	else {
		printf("no valid stereo observation!\n");
	}
}

void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
}

void project2_visualize(roger)
Robot* roger;
{ 
	void draw_observations();
	draw_observation(obs);
}

/* === DOESN'T need to be done in Project 2 === */
void arm_Jacobian(theta1,theta2, Jacobian)
double theta1,theta2;
double Jacobian[2][2];
{
}




