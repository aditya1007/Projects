#include <MKL25Z4.H>
#include <stdio.h>
#include <math.h>

#include "Drift_Calculation.h"
#include "trig_approx.h"

#define PI (3.14159265f)
#define Two_PI (6.2831853f)
#define PI_over_180 (0.017453f)
#define Oneeighty_over_PI (57.2958f)
 
void Compute_Current(float speed_water, float angle_heading, float speed_ground, float angle_track, 
	float * speed_current, float * angle_current){
	float angle_drift_rad = 0.0;
	float local_speed_current, local_angle_current;
	float temp;
	
	angle_drift_rad = (angle_track - angle_heading);
	if (fabs(speed_ground) < MIN_SPEED_TOLERANCE) { 		// Check for special cases - no relative motion
		// not moving relative to ground, so current direction is opposite of heading through water
		local_angle_current = angle_heading-PI;
		local_speed_current = speed_water;
	}	else if (fabs(speed_water) < MIN_SPEED_TOLERANCE) { 
		// not moving relative to water, so current direction is track over ground
		local_angle_current = angle_heading;
		local_speed_current = speed_ground;
	}	else if (fabs(angle_drift_rad) < MIN_ANGLE_TOLERANCE) { 	// Check for special cases - no angular difference
		// track and heading are same: case IV
		local_angle_current = angle_track;
		local_speed_current = speed_ground - speed_water;
	} else if (fabs(angle_drift_rad - PI) < MIN_ANGLE_TOLERANCE) { 
		// track and heading are opposite: case III
		local_angle_current = angle_heading-PI;
		local_speed_current = speed_ground + speed_water;
	} else {
		
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%  Modified Aditya  %%%%%%%%%%%%%%%%%%%%%
		angle_drift_rad=fmodf(angle_drift_rad, Two_PI);	
		
		local_speed_current = sqrtf((speed_ground*speed_ground) + (speed_water*speed_water) - (2*speed_water*speed_ground*cos_32(angle_drift_rad)));
		temp = sin_32(angle_drift_rad)*(speed_ground/local_speed_current);
		
		if (temp > 1)
		{
		temp = 1;
		local_angle_current= 1.5708f+ angle_heading; //%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%
		}
		else if (temp < -1)
		{
			temp = -1;
		local_angle_current= 4.712389f+ angle_heading; //%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%
		}
		else 
		{
		local_angle_current = (PI-(asinf(temp))) + angle_heading;//%%%%%%%%%%%%%%%%%%%--Modified by Aditya--%%%%%%%%%%%%%%%%%
		}
	}	
	if (local_angle_current < 0)
		local_angle_current += Two_PI;
	if (local_angle_current >= Two_PI)
		local_angle_current -= Two_PI;

	*speed_current = local_speed_current;
	*angle_current = local_angle_current;
}
	
