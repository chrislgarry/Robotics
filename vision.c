/*************************************************************************/
/* File:        vision.c                                                 */
/* Description: User project #2                                          */
/* Date:        1-1-14                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void construct_wTb(), matXvec4441(), matXmat2222(), matrix_transpose22();

void stereoJJT(roger, ur, ul, JJT)
Robot * roger;
double ur, ul, JJT[2][2]; /* observation covariance only assigns 2x2 */
{ }

// triangulate the position of the red ball, transform from the base frame to
// to world frame and write into Observation obs
int stereo_observation(roger, obs)
Robot * roger;
Observation * obs;
{
  double ur, ul;
  double wTb[4][4], bTw[4][4], wRb[2][2], bRw[2][2], ref_b[4], ref_w[4];
  double lambdaL, gammaL, gammaR;
  double cov_w[2][2], jacobian_b[2][2], jacobian_w[2][2], jacobian_wT[2][2];

  int compute_average_red_pixel();
  int no_red_blue_transitions = TRUE;
  int check_stereo_FOV();

  /************************************************************************/
  // PROJECT2: triangulate to localize the red ball in the base frame
  //           convert to world frame, and
  //           write into Observation "obs"
  //
  // build a cascade of filters to identify "valid" stereo observations
  // is there "red" on both image planes?
  // are there any red-blue transitions?
  // are there any FOV issues?
  if ((compute_average_red_pixel(roger, &ur, &ul) == TRUE) &&
      (no_red_blue_transitions) && (check_stereo_FOV() == TRUE) ) {
    
    // get angle from base to object
    gammaL = roger->eye_theta[LEFT] + atan2((ul - 63.5), 64.0);
    gammaR = roger->eye_theta[RIGHT] + atan2((ur - 63.5), 64.0);

    //IMPLEMENTED
      
      //COMPUTE OBS POS ****************************************/

        // calculate x,y coordinates of average red pixel location in base frame  
        ref_b[X] = ((2*BASELINE)*((cos(gammaR)*cos(gammaL))/sin(gammaR-gammaL)));
        ref_b[Y] = ((2*BASELINE)*((cos(gammaR)*sin(gammaL))/sin(gammaR-gammaL))) + BASELINE;

        //Compute world-to-base (wTb) transform
        construct_wTb(roger->base_position,wTb);

        //Transform computed base coordinates (ref_b) to world coordinates (ref_w) using world-to-base (wTb)
        matXvec4441(wTb,ref_b,ref_w);

        //Store world coordinates in observation
        obs->pos[X] = ref_w[X];
        obs->pos[Y] = ref_w[Y];

      //COMPUTE OBS COV ****************************************/

        //Compute localizability Jacobian in base coordinates
        jacobian_b[0][0] = ((2*BASELINE)/(sin(gammaR-gammaL)*sin(gammaR-gammaL))) * (cos(gammaR)*cos(gammaR));
        jacobian_b[0][1] = ((2*BASELINE)/(sin(gammaR-gammaL)*sin(gammaR-gammaL))) * (-1*cos(gammaL)*cos(gammaL));
        jacobian_b[1][0] = ((2*BASELINE)/(sin(gammaR-gammaL)*sin(gammaR-gammaL))) * (sin(gammaR)*cos(gammaR));
        jacobian_b[1][1] = ((2*BASELINE)/(sin(gammaR-gammaL)*sin(gammaR-gammaL))) * (-1*sin(gammaL)*cos(gammaL));

        //Extract rotation matrix from homogenous transform wTb
        wRb[0][0] = wTb[0][0];
        wRb[0][1] = wTb[0][1];
        wRb[1][0] = wTb[1][0];
        wRb[1][1] = wTb[1][1];

        //Compute the localizability jacobian in world coordinates by rotating
        matXmat2222(wRb,jacobian_b, jacobian_w);

        //Compute the jacobian transpose
        matrix_transpose22(jacobian_w, jacobian_wT);

        //Compute quadratic form by jacobian*vacobian_transpose (JJT) to approximate covariance matrix
        matXmat2222(jacobian_w, jacobian_wT, cov_w);

        //Store quadratic form as covariance matrix into observation
        obs->cov[0][0] = cov_w[0][0];
        obs->cov[0][1] = cov_w[0][1];
        obs->cov[1][0] = cov_w[1][0];
        obs->cov[1][1] = cov_w[1][1];
      
    //END IMPLEMENTED

    return(TRUE);
  }
  else return(FALSE);
}

/*************************************************************************/
// compute the (0<=position<128) of the average red pixel on both image planes
// if red is detected on both images:
//       write the index of the mean pixel into ul and ur;
//       return TRUE;
// else return FALSE;
/*************************************************************************/
int compute_average_red_pixel(roger, ur, ul)
Robot* roger;
double *ur, *ul;
{
  int i, nr, nl;
  double r,g,b,I,m,S,H;

  nr = nl = 0;     //Number of pixels with red color
  *ul = *ur = 0.0; //Pointers to mean pixel location in left and right eye

  for (i=0;i<NPIXELS;++i) {

    /* PIXEL i IN LEFT EYE */
    r = (double) roger->image[LEFT][i][RED_CHANNEL];
    g = (double) roger->image[LEFT][i][GREEN_CHANNEL];
    b = (double) roger->image[LEFT][i][BLUE_CHANNEL];

    //IMPLEMENTED
      //Compute average "red pixel index" for left eye
      if(r==255.0){
        //SUM LEFT EYE INDICIES CONTAINING RED AND STORE TEMPORARILY IN UL
        *ul = (((*ul)*nl)+i)/(nl+1);
        nl++;
      }

    //END IMPLEMENTED

    /* PIXEL i IN RIGHT EYE */
    r = (double) roger->image[RIGHT][i][RED_CHANNEL];
    g = (double) roger->image[RIGHT][i][GREEN_CHANNEL];
    b = (double) roger->image[RIGHT][i][BLUE_CHANNEL];

    //IMPLEMENTED
      //Compute average "red pixel index" for right eye
      if(r==255.0){
        //SUM RIGHT EYE INDICIES CONTAINING RED AND STORE TEMPORARILY IN UR
        *ur = (((*ur)*nr)+i)/(nr+1);
        nr++;
      }

    //END IMPLEMENTED

  }

  //RETURN TRUE OR FALSE
  
  //IMPLEMENTED

    //If red is detected on both images
    if(nr > 0 && nl > 0){
      return(TRUE);
    }

  //END IMPLEMENTED

  return(FALSE);
}

/*************************************************************************/
// if red is detected in pixel 0 and 127 on either image:
//       return FALSE
// else  return TRUE;
/*************************************************************************/
int check_stereo_FOV(roger)
Robot* roger;
{
  int i, p[2][2]={{FALSE, FALSE},{FALSE, FALSE}};

  double r, g, b, I, m, S, H;

  /* PIXEL 0 IN LEFT EYE */
  r = (double) roger->image[LEFT][0][RED_CHANNEL];
  g = (double) roger->image[LEFT][0][GREEN_CHANNEL];
  b = (double) roger->image[LEFT][0][BLUE_CHANNEL];

  //  I = (r + g +  b)/3.0;
  //  m = MIN(MIN(r,g),b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;
  
  // this is "red"
  if ( (H<=10.0) || H>=350.0) { p[LEFT][0]=TRUE; }

  /* PIXEL 127 IN LEFT EYE */
  r = (double) roger->image[LEFT][127][RED_CHANNEL];
  g = (double) roger->image[LEFT][127][GREEN_CHANNEL];
  b = (double) roger->image[LEFT][127][BLUE_CHANNEL];

  //  I = (r + g +  b)/3.0;
  //  m = MIN(MIN(r,g),b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;

  // this is "red"
  if ( (H<=10.0) || H>=350.0) { p[LEFT][1]=TRUE; }

  /* PIXEL 0 IN RIGHT EYE */
  r = (double) roger->image[RIGHT][0][RED_CHANNEL];
  g = (double) roger->image[RIGHT][0][GREEN_CHANNEL];
  b = (double) roger->image[RIGHT][0][BLUE_CHANNEL];

  //  I = (r + g + b)/3.0;
  //  m = MIN(MIN(r,g), b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;

  if ( (H<=10.0) || H>=350.0) { p[RIGHT][0]=TRUE; }
  
  /* PIXEL 127 IN RIGHT EYE */
  r = (double) roger->image[RIGHT][127][RED_CHANNEL];
  g = (double) roger->image[RIGHT][127][GREEN_CHANNEL];
  b = (double) roger->image[RIGHT][127][BLUE_CHANNEL];

  //  I = (r + g + b)/3.0;
  //  m = MIN(MIN(r,g), b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;

  if ( (H<=10.0) || H>=350.0) { p[RIGHT][1]=TRUE; }

  /***********************************************************************/

  if (p[LEFT][0] || p[LEFT][1] || p[RIGHT][0] || p[RIGHT][1]) return(FALSE);
  else return(TRUE);
}

