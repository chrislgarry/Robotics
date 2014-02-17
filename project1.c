/*************************************************************************/
/* File:        MotorUnits.c                                             */
/* Description: motor units execute every simulated millisecond and are  */
/*              never disengaged, applications control Roger by altering */
/*              setpoints for each degree of freedom                     */
/* Date:        11-2012                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include <sys/time.h>

void update_setpoints();

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
/*************************************************************************/

//EYES-----------------------------------------------------------------------------------------------
	// gains for the PD controllers for eyes
	double kp_eye = 1.0; //KP_EYE
	double kd_eye = (sqrt(4.0*1.0*I_EYE)); //KD_EYE;

	/* PROJECT #1.1 - PD CONTROLLER FOR THE EYES                             */
	/* setpoints are joint angle values in radians for the eyes              */
	void PDController_eyes(roger, time)
	Robot * roger;
	double time;
	{
		int i;
		double theta_error, theta_ddot_des;

		for (i = 0; i < NEYES; i++) {

			theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];

			//IMPLEMENTED---------------------

				roger->eye_torque[i] = (kp_eye*theta_error) - (kd_eye*(roger->eye_theta_dot[i]));
				//Print some datapoints
				// if(i == 0){
				// 	printf("%f,%f,%f\n", theta_error, roger->eyes_setpoint[i], roger->eye_theta[i]);
				// }
				
			//END IMPLEMENTED-----------------
		}
	}

//ARMS-----------------------------------------------------------------------------------------------

	// gains for the PD controllers for arms 
	double kp_arm = 80.0; //KP_ARM;
	double kd_arm = 10.0; //KD_ARM;

	/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
	/* setpoints - joint angles in radians for the shoulders and elbows      */
	void PDController_arms(roger, time)
	Robot * roger;
	double time;
	{
		int i;
		double theta_error[2], acc[2], M[2][2], V[2], G[2];

		for (i=LEFT; i<=RIGHT; ++i) {
			// PDcontrol - desired accelerations
			theta_error[0] = roger->arm_setpoint[i][0] - roger->arm_theta[i][0];
			theta_error[1] = roger->arm_setpoint[i][1] - roger->arm_theta[i][1];

			//shoulder - bound error -pi < error < +pi
			while (theta_error[0] > M_PI) theta_error[0] -= 2.0 * M_PI;
			while (theta_error[0] < -M_PI) theta_error[0] += 2.0 * M_PI;
			//elbow - bound error -pi < error < +pi
			while (theta_error[1] > M_PI) theta_error[1] -= 2.0 * M_PI;
			while (theta_error[1] < -M_PI) theta_error[1] += 2.0 * M_PI;

			// tune kp_arm and kd_arm by changing their value using "Enter Params" button

			//IMPLEMENTED--------------------
				roger->arm_torque[i][0] = (kp_arm*theta_error[0])-(kd_arm*(roger->arm_theta_dot[i][0]));
				roger->arm_torque[i][1] = (kp_arm*theta_error[1])-(kd_arm*(roger->arm_theta_dot[i][1]));
				if(i==0) {
					printf("%6.4lf\n", theta_error[i]);
				}
			//END IMPLEMENTED----------------

		}
	}

//BASE TRANS-----------------------------------------------------------------------------------------
	// gains for base translational controller
	double kp_base_trans = 3050.0; //3050.0 KP_BASE
	double kd_base_trans = 102.0;  //102.0 KD_BASE

	/* Base PD controller, Cartesian reference */
	double PDBase_translate(roger, time) 
	Robot * roger;
	double time;
	{ 
		double Fx;

		//IMPLEMENTED

		double error[2], trans_error, veloc_error;

		//Compute error in x and y reference coordinates
		error[X] = roger->base_setpoint[X] - roger->base_position[X];
         	error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];

         	//Compute coordinates of base in world coordinates
           	double x_b[2] = {cos(roger->base_position[THETA]), sin(roger->base_position[THETA])};
         	
         	//Project translational and velocity errors onto world coordinates
         	trans_error = (error[X]*x_b[X])+(error[Y]*x_b[Y]);
         	veloc_error = (roger->base_velocity[X]*x_b[X])+(roger->base_velocity[Y]*x_b[Y]);

         	Fx = kp_base_trans*trans_error - kd_base_trans*veloc_error;

         	//DEBUG
         		//printf("%6.4lf,%6.4lf,", error[X],error[Y]);

		//END IMPLEMENTED

		return(Fx);
	}

//BASE ROT-------------------------------------------------------------------------------------------
	// gains for base rotational controller
	double kp_base_rot = 230.0; //230.0 KP_BASE
	double kd_base_rot = 5.0;   //5.0 KD_BASE

	/* Base PD controller, Cartesian reference */
	double PDBase_rotate(roger, time) 
	Robot * roger;
	double time;
	{
		double Mz;

		//IMPLEMENTED
			//Compute rotational error 
			double rot_error = roger->base_setpoint[2]-roger->base_position[2];

			//Bound rotational error 
			while (rot_error > M_PI) rot_error -= 2.0 * M_PI;
			while (rot_error < -M_PI) rot_error += 2.0 * M_PI;

			//Controller for rotation
			Mz = (kp_base_rot*rot_error) - (kd_base_rot*(roger->base_velocity[2]));

			//DEBUG
				//printf("%6.4lf\n", rot_error);

		//END IMPLEMENTED

		return(Mz);
	}

//BASE-----------------------------------------------------------------------------------------------
	/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
	/* setpoints - (xy) location for translation heading in radians          */

	//the base differential drive Jacobian:
	double baseJT[2][2] = {{0.5, (-1.0/(2.0*R_AXLE))},{0.5, (1.0/(2.0*R_AXLE))}};

	void PDController_base(roger, time)
	Robot * roger;
	double time;
	{ 
		double Fx, Mz, PDBase_translate(), PDBase_rotate();

		Fx = PDBase_translate(roger,time);
		Mz = PDBase_rotate(roger,time);

		//IMPLEMENTED

			// integrated wheel torque control
			roger->wheel_torque[LEFT]  = ((Fx*baseJT[0][0])+(Mz*baseJT[0][1]));
			roger->wheel_torque[RIGHT] = ((Fx*baseJT[1][0])+(Mz*baseJT[1][1]));

			//DEBUG
				//printf("%6.4lf  %6.4lf\n %6.4lf  %6.4lf", baseJT[0][0],baseJT[0][1],baseJT[1][0],baseJT[1][1]);
				//printf("Fx=%6.4lf  Mz=%6.4lf\n", Fx, Mz);

		//END IMPLEMENTED
	}

//MISC-----------------------------------------------------------------------------------------------
	/*************************************************************************/
	/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
	/*                        *** DO NOT ALTER ***                           */
	/*************************************************************************/
	void control_roger(roger, time)
	Robot * roger;
	double time;
	{
		update_setpoints(roger);

		// turn setpoint references into torques
		PDController_eyes(roger, time);
		PDController_arms(roger, time);
		PDController_base(roger,time);

	}

	/*************************************************************************/
	void project1_reset(roger)
	Robot* roger;
	{ }

	/*************************************************************************/
	// prompt for and read user customized input values                             
	/*************************************************************************/
	void project1_enter_params()
	{
		// put anything in here that you would like to change at run-time
		// without re-compiling user project codes
		//fclose(f);
		printf("EYE: K=%6.4lf  B=%6.4lf\n", kp_base_trans, kd_base_trans);
		printf("EYE: enter 'K B'\n"); fflush(stdout);
		scanf("%lf %lf", &kp_base_trans, &kd_base_trans);
	}

	/*************************************************************************/
	// function called when the 'visualize' button on the gui is pressed            
	void project1_visualize(roger)
	Robot* roger;
	{ }

