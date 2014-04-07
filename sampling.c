/*************************************************************************/
/* File:        sampling.c                                               */
/* Description: Methods for sampling from distributions                  */
/* Date:        10-30-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "models.h"

double time();
int init_random_seed = FALSE; // int init_random_seed = TRUE;

#define MAX_SAMPLES 10 // number of sample repetition before giving up
int sample_count = 0;

// pr.bin_size, pr.area, pr.dist[NHEADINGS]
Pr_dist Pr_red_prior = {(2.0*M_PI/NHEADINGS), 1.0,
			{1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			 1.0/NH}};

int sample_gaze_direction(heading)
double *heading;
{
  int i;
  double sum, random;

  if (init_random_seed == TRUE) {
    srand(time(NULL));
    init_random_seed = FALSE;
  }
  // uniformly sample from -PI to PI
  random = ((double)rand()/(double)RAND_MAX)*Pr_red_prior.area;
  i=sum=0;
  while (sum < random) {
    sum += Pr_red_prior.dist[i++]; 
  }
  *heading = ((double)i+0.5)*Pr_red_prior.bin_size - M_PI;

  if (Pr_red_prior.area < 0.02) {
    sample_count = 0;
    return FALSE;
  }
  return TRUE;
}
