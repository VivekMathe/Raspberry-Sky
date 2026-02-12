#include "guidance.h"
#include <Eigen/Dense>
#include "math_utils.h"
#include <cmath>
#include <iostream>

Guidance::Guidance(const Vector12d& x, const Vector2d& n_bounds, const Vector2d& e_bounds, int numpasses, double cruise_speed, double yaw_rate, double takeoff, double pgain, double vgain)
{
	x0 = x;
	phase = 1;
	found = false;
	at_end = false;
	nbounds = n_bounds;
	ebounds = e_bounds;
	passes = numpasses;
	takeoff_height = takeoff;
	cruise = cruise_speed;
	yaw_lim = yaw_rate;
	fov = 6 * .3048; //6 foot
	buffer = fov / 4;
	lawnmower_stripes.resize(passes);
	lawnmower_stripes.setZero();
	delta_e = (ebounds(1) - ebounds(0) - fov/2) / (passes - 1); //distance between stripes
	for (int i = 0; i < passes; i++)
	{
		lawnmower_stripes(i) = ebounds(0) + fov / 4 + i * delta_e; 
	}
	std::cout << "All planned stripes: " << "\n" << lawnmower_stripes << "\n";
	((lawnmower_stripes.array() - x(7)).cwiseAbs()).minCoeff(&stripe_index); //initializing stripe 
	Kp = pgain;
	Kd = vgain;
	edir = 1;
	ndir = 1;
	//north is to the right, east is up. Stripes will be along east. 
}

Vector10d Guidance::getTarget(const Vector12d& x)
{
	Vector10d commands;
	double psides;
	Vector3d omegades;
	Vector3d pdes;
	Vector3d vdes;
	switch (phase)
	{
	case 1: //takeoff
	{
		Vector2d ne_des;
		ne_des << x0(0) + std::copysign(buffer + .1, (nbounds.mean() - x0(0))), lawnmower_stripes(stripe_index);

		psides = x(2);
		omegades = Vector3d::Zero();
		pdes << ne_des(0), ne_des(1), -takeoff_height;
		vdes = Vector3d::Zero();
		double res = (pdes - x.block(6, 0, 3, 1)).norm();
		if (res < .02)
		{

			//done with takeoff, this is now initializing the search pattern
			phase += 1;
			std::cout << "Takeoff completed, beginning search" << "\n";
			Eigen::Index minIndex;
			((ebounds.array() - x(7)).cwiseAbs()).minCoeff(&minIndex); //finding nearest stripe
			if (minIndex == 0)
			{
				edir = 1; //going left to right
			}
			else
			{
				edir = -1; //going right to left
			}

			((nbounds.array() - x(6)).cwiseAbs()).minCoeff(&minIndex);
			if (minIndex == 0)
			{
				ndir = 1; //going up
			}
			else
			{
				ndir = -1; //going down
			}

		}
		break;
	}
	case 2: //searching
	{
		double target_e = lawnmower_stripes(stripe_index);
		double ve;
		double vn;
		if (x(6) > nbounds(1) - buffer || x(6) < nbounds(0) + buffer) //!at_end prevents this from triggering constantly outside of bounds
		{
			if (!at_end)
			{
				at_end = true;
				ndir *= -1; //switch direction along stripe
				if ((stripe_index + edir) >= passes || (stripe_index + edir) < 0) //if we're at the end of the lawnmower, restart by going the other way
				{
					edir = -edir;
				}
				std::cout << "OLD stripe:" << "\n" << lawnmower_stripes(stripe_index) << "\n";
				stripe_index += edir; //move to next stripe
				target_e = lawnmower_stripes(stripe_index);
				std::cout << "NEW stripe:" << "\n" << lawnmower_stripes(stripe_index) << "\n";
				std::cout << "POS" << "\n" << x.block(6, 0, 2, 1) << "\n";

			}
		}
		else
		{
			at_end = false;
		}
		ve = Kp * (target_e - x(7)) - Kd * x(10);
		
		ve = saturate(ve, cruise);
		vn = ndir * cruise;
		pdes << x(6), x(7), -takeoff_height; //not commanding n and e: Error is 0, velocity field should keep these in touch

		//extra nudge to push in bounds if we're going outside of the true bounds, as this can be catastrophic.
		
		if (x(6) > nbounds(1))
		{
			pdes(0) = x(6) - fov / 8;
		}
		else if (x(6) < nbounds(0))
		{
			pdes(0) = x(6) + fov / 8;
		}
		if (x(7) > ebounds(1))
		{
			pdes(1) = x(7) - fov / 8;
		}
		else if (x(7) < ebounds(0))
		{
			pdes(1) = x(7) + fov / 8;
		}
		
		psides = x(2);
		omegades = Vector3d::Zero();
		vdes << vn, ve, 0;
		break;
	}
	case 3: //logic from fire detection would go into these cases
	{
		std::exit(0);
		break;
	}
	case 4:
	{
		break;
	}
	}
	commands << psides, omegades, pdes, vdes;
	//returns psides, omegades, pdes, vdes
	return commands;
}
Vector4d Guidance::manualCommands(const Eigen::Matrix<double, 6, 1>& pwms)
{
	Vector4d motor_pwms; //psirate vn ve vd 
	motor_pwms << pwms(3), pwms(1), pwms(0), pwms(2);
	Vector4d commands; //psirate vn ve vd
	commands = (motor_pwms.array() - 1500) / 500; //scaling to -1 to 1
	commands(0) = commands(0) * yaw_lim; //scaling yaw rate
	commands.block(1, 0, 3, 1) = commands.block(1, 0, 3, 1) * cruise; //scaling to crusie speed
	{
		//two ramps with a deadzone in the middle at 0 throttle
		if (motor_pwms(3) <= 1400)
		{
			commands(3) = cruise * 1 / 400 * (commands(3) - 1400);
		}
		else if (motor_pwms(3) < 1600)
		{
			commands(3) = 0;
		}
		else
		{
			commands(3) = cruise * 1 / 400 * (commands(3) - 1600);
		}
	}

	

	return commands;//vn ve vd psi
}