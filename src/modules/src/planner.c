/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
implementation of planning state machine
*/
#include <stddef.h>
#include "planner.h"
#include "arm_math.h"
#include "debug.h"

static struct traj_eval plan_eval(struct planner *p, float t);

static void plan_takeoff_or_landing(struct planner *p, struct vec curr_pos, float curr_yaw, float hover_height, float hover_yaw, float duration)
{
	struct vec hover_pos = curr_pos;
	hover_pos.z = hover_height;

	// compute the shortest possible rotation towards 0
	hover_yaw = normalize_radians(hover_yaw);
	curr_yaw = normalize_radians(curr_yaw);
	float goal_yaw = curr_yaw + shortest_signed_angle_radians(curr_yaw, hover_yaw);

	piecewise_plan_7th_order_no_jerk(&p->planned_trajectory, duration,
		curr_pos,  curr_yaw,  vzero(), 0, vzero(),
		hover_pos, goal_yaw, vzero(), 0, vzero());
}

// ----------------- //
// public functions. //
// ----------------- //

void plan_init(struct planner *p)
{
	p->state = TRAJECTORY_STATE_IDLE;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->reversed = false;
	p->trajectory = NULL;
	p->compressed_trajectory = NULL;
	p->planned_trajectory.pieces = p->pieces;
}

void plan_stop(struct planner *p)
{
	p->state = TRAJECTORY_STATE_IDLE;
}

bool plan_is_finished(struct planner *p, float t)
{
	if (p->trajectory == NULL) {
		return 1;
	}
	switch (p->type) {
		case TRAJECTORY_TYPE_PIECEWISE:
			return piecewise_is_finished(p->trajectory, t);

		case TRAJECTORY_TYPE_PIECEWISE_COMPRESSED:
		  return piecewise_compressed_is_finished(p->compressed_trajectory, t);

		default:
		  return 1;
	}
}

bool plan_is_stopped(struct planner *p)
{
	return p->state == TRAJECTORY_STATE_IDLE;
}

void plan_disable(struct planner *p)
{
	p->state = TRAJECTORY_STATE_DISABLED;
}

bool plan_is_disabled(struct planner *p)
{
	return p->state == TRAJECTORY_STATE_DISABLED;
}

struct traj_eval plan_current_goal(struct planner *p, float t)
{
	switch (p->state) {
		case TRAJECTORY_STATE_LANDING:
			if (plan_is_finished(p, t)) {
				p->state = TRAJECTORY_STATE_IDLE;
			}
			// intentional fall-thru
		case TRAJECTORY_STATE_FLYING:
			return plan_eval(p, t);

		default:
			return traj_eval_invalid();
	}
}

struct traj_eval plan_eval(struct planner *p, float t)
{
	switch (p->type) {
		case TRAJECTORY_TYPE_PIECEWISE:
			if (p->reversed) {
				return piecewise_eval_reversed(p->trajectory, t);
			}
			else {
				return piecewise_eval(p->trajectory, t);
			}
			break;

		case TRAJECTORY_TYPE_PIECEWISE_COMPRESSED:
			if (p->reversed) {
				/* not supported */
				return traj_eval_invalid();
			}
			else {
				return piecewise_compressed_eval(p->compressed_trajectory, t);
			}
			break;

		default:
			return traj_eval_invalid();
	}
}

int plan_takeoff(struct planner *p, struct vec curr_pos, float curr_yaw, float hover_height, float hover_yaw, float duration, float t)
{
	if (p->state != TRAJECTORY_STATE_IDLE) {
		return 1;
	}

	plan_takeoff_or_landing(p, curr_pos, curr_yaw, hover_height, hover_yaw, duration);
	p->reversed = false;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_land(struct planner *p, struct vec curr_pos, float curr_yaw, float hover_height, float hover_yaw, float duration, float t)
{
	if (p->state == TRAJECTORY_STATE_LANDING) {
		return 1;
	}

	plan_takeoff_or_landing(p, curr_pos, curr_yaw, hover_height, hover_yaw, duration);
	p->reversed = false;
	p->state = TRAJECTORY_STATE_LANDING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_go_to_from(struct planner *p, const struct traj_eval *curr_eval, bool relative, bool linear, struct vec hover_pos, float hover_yaw, float duration, float t)
{
	if (relative) {
		hover_pos = vadd(hover_pos, curr_eval->pos);
		hover_yaw += curr_eval->yaw;
	}

	// compute the shortest possible rotation towards 0
	float curr_yaw = normalize_radians(curr_eval->yaw);
	hover_yaw = normalize_radians(hover_yaw);
	float delta_yaw = shortest_signed_angle_radians(curr_yaw, hover_yaw);
	float goal_yaw = curr_yaw + delta_yaw;

	if (linear) {
		struct vec vel = vdiv(vsub(hover_pos,curr_eval->pos), duration);
		float omz = delta_yaw/duration;
		
		piecewise_plan_7th_order_no_jerk(&p->planned_trajectory, duration,
		curr_eval->pos, curr_yaw, vel, omz, vzero(),
		hover_pos,      goal_yaw, vel, omz, vzero());
	}
	else {
		piecewise_plan_7th_order_no_jerk(&p->planned_trajectory, duration,
		curr_eval->pos, curr_yaw, curr_eval->vel, curr_eval->omega.z, curr_eval->acc,
		hover_pos,      goal_yaw,      vzero(),        0,                  vzero());
	}

	p->reversed = false;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_go_to(struct planner *p, bool relative, bool linear, struct vec hover_pos, float hover_yaw, float duration, float t)
{
	struct traj_eval setpoint = plan_current_goal(p, t);
	return plan_go_to_from(p, &setpoint, relative, linear, hover_pos, hover_yaw, duration, t);
}

int plan_spiral_from(struct planner *p, const struct traj_eval *curr_eval, bool sideways, bool clockwise, float spiral_angle, float radius0, float radiusF, float ascent, float duration, float t)
{
	// Limitting the inputs
  if (spiral_angle > 2*PI) {
    spiral_angle = 2*PI;
    DEBUG_PRINT("Warning: spiral angle saturated at 2pi\n");
  }
  else if (spiral_angle < -2*PI) {
    spiral_angle = -2*PI;
    DEBUG_PRINT("Warning: spiral angle saturated at -2pi\n");
  }
  if (radius0 < 0) {
    radius0 = 0;
    DEBUG_PRINT("Warning: radius set to 0 (was negative) \n");
  }
  if (radiusF < 0) {
    radiusF = 0;
    DEBUG_PRINT("Warning: radius set to 0 (was negative) \n");
  }
    
  int sense = -1 + 2*clockwise; // -1 counter-clockwise, +1 clockwise
	float omz =  -sense*spiral_angle/duration;
	float dz = ascent/duration;
	float dr = (radiusF - radius0)/duration;
	
	struct vec pos0 = curr_eval->pos;
	
	float phi0 = normalize_radians(curr_eval->yaw);
	
	float yawF = phi0 - sense*spiral_angle;
	float phiF = phi0 - sense*spiral_angle;

	float c0 = cosf(phi0);
	float s0 = sinf(phi0);
	float cF = cosf(phiF);
	float sF = sinf(phiF);

	float z0 = pos0.z;
	float xCenter, yCenter, dx0, ddx0, xF, dxF, ddxF, dy0, ddy0, yF, dyF, ddyF;

	if (sideways)
	{
		// center position
		xCenter = pos0.x + sense*radius0*c0;
		yCenter = pos0.y + sense*radius0*s0;
		
		// x0 = -sense*radius0*c0;
		dx0 = sense*(radius0*s0*omz - dr*c0);
		ddx0 = sense*(radius0*c0*omz*omz + 2*dr*s0*omz);

		xF = -sense*radiusF*cF;
		dxF = sense*(radiusF*sF*omz - dr*cF);
		ddxF = sense*(radiusF*cF*omz*omz + 2*dr*sF*omz);
		
		// y0 = -sense*radius0*s0;
		dy0 = sense*(-radius0*c0*omz - dr*s0);
		ddy0 = sense*(radius0*s0*omz*omz - 2*dr*c0*omz);

		yF = -sense*radiusF*sF;
		dyF = sense*(-radiusF*cF*omz - dr*sF);
		ddyF = sense*(radiusF*sF*omz*omz - 2*dr*cF*omz);
	}
	else
	{
		// center position
		xCenter = pos0.x + sense*radius0*s0;
		yCenter = pos0.y - sense*radius0*c0;
		
		// x0 = -sense*radius0*s0;
		dx0 = -sense*(radius0*c0*omz + dr*s0);
		ddx0 = sense*(radius0*s0*omz*omz - 2*dr*c0*omz);

		xF = -sense*radiusF*sF;
		dxF = -sense*(radiusF*cF*omz + dr*sF);
		ddxF = sense*(radiusF*sF*omz*omz - 2*dr*cF*omz);
		
		// y0 = sense*radius0*c0;
		dy0 = sense*(-radius0*s0*omz + dr*c0);
		ddy0 = sense*(-radius0*c0*omz*omz - 2*dr*s0*omz);

		yF = sense*radiusF*cF;
		dyF = sense*(-radiusF*sF*omz + dr*cF);
		ddyF = sense*(-radiusF*cF*omz*omz - 2*dr*sF*omz);
	}

	struct vec posF = mkvec(xCenter + xF, yCenter + yF, z0 + ascent);
	struct vec vel0 = mkvec(dx0, dy0, dz);
	struct vec velF = mkvec(dxF, dyF, dz);
	struct vec acc0 = mkvec(ddx0, ddy0, 0);
	struct vec accF = mkvec(ddxF, ddyF, 0);
	
	piecewise_plan_7th_order_no_jerk(&p->planned_trajectory, duration,
		pos0, phi0, vel0, omz, acc0,
		posF, yawF, velF, omz, accF);
	
	p->reversed = false;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->planned_trajectory.t_begin = t;
	p->trajectory = &p->planned_trajectory;
	return 0;
}

int plan_start_trajectory(struct planner *p, struct piecewise_traj* trajectory, bool reversed, bool relative_position, bool relative_yaw, struct vec start_from, float start_yaw)
{
	p->reversed = reversed;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE;
	p->trajectory = trajectory;

	if (relative_position) {
		struct traj_eval traj_init;
		trajectory->shift = vzero();
		trajectory->shift_yaw = 0;
		if (reversed) {
			traj_init = piecewise_eval_reversed(trajectory, trajectory->t_begin);
		}
		else {
			traj_init = piecewise_eval(trajectory, trajectory->t_begin);
		}

		// translate trajectory to starting point
		struct vec shift_pos = vsub(start_from, traj_init.pos);
		trajectory->shift = shift_pos;
		
		if (relative_yaw) {
			// compute the shortest possible rotation towards trajectory start yaw to current yaw 
			float traj_yaw = normalize_radians(traj_init.yaw);
			start_yaw = normalize_radians(start_yaw);
			trajectory->shift_yaw = shortest_signed_angle_radians(traj_yaw, start_yaw);
		}
	}
	else {
		trajectory->shift = vzero();
		trajectory->shift_yaw = 0;
	}

	return 0;
}

int plan_start_compressed_trajectory( struct planner *p, struct piecewise_traj_compressed* trajectory, bool relative, struct vec start_from)
{
	p->reversed = 0;
	p->state = TRAJECTORY_STATE_FLYING;
	p->type = TRAJECTORY_TYPE_PIECEWISE_COMPRESSED;
	p->compressed_trajectory = trajectory;

	if (relative) {
		trajectory->shift = vzero();
		struct traj_eval traj_init = piecewise_compressed_eval(
			trajectory, trajectory->t_begin
		);

		// translate trajectory to current position
		struct vec shift_pos = vsub(start_from, traj_init.pos);
		trajectory->shift = shift_pos;
	} else {
		trajectory->shift = vzero();
	}

	return 0;
}