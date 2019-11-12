/*

Compressed piecewise polynomial trajectories for Crazyflie.

Copyright (c) 2019 CollMot Robotics. All rights reserved.

*/

/*
Header file for compressed piecewise polynomial trajectories
*/

#pragma once

#include "pptraj.h"
#include <stdio.h>

enum piecewise_traj_storage_type {
	PPTRAJ_STORAGE_CONSTANT = 0,
	PPTRAJ_STORAGE_LINEAR = 1,
	PPTRAJ_STORAGE_BEZIER = 2,
	PPTRAJ_STORAGE_FULL = 3
};

// ---------------------------------------------//
// compressed piecewise polynomial trajectories //
// ---------------------------------------------//

struct piecewise_traj_compressed
{
	float t_begin;
	float duration;
	float timescale;
	struct vec shift;
	const void* data;

	// mutable part of the data structure. We plan to mess around with this part
	// but keep the rest untouched (i.e. supplied by the user)
	struct {
		// raw representation of the current piece
		const void* data;

		// start time of the current piece, relative to the "global" start time of
		// the entire trajectory
		float t_begin_relative;

		// poly4d representation of the current piece
		struct poly4d poly4d;
	} current_piece;
};

// Returns the total duration of a compressed trajectory. The total duration
// is pre-calculated and cached in the trajectory itself.
static float piecewise_compressed_duration(struct piecewise_traj_compressed const *traj) {
	return traj->duration;
}

// Returns whether we have finished flying the trajectory
static inline bool piecewise_compressed_is_finished(
	struct piecewise_traj_compressed const *traj, float t)
{
	return (t - traj->t_begin) >= piecewise_compressed_duration(traj);
}

// Evaluates the trajectory at the given time instant.
struct traj_eval piecewise_compressed_eval(
	struct piecewise_traj_compressed *traj, float t);

// Loads the compressed trajectory at the given pointer
void piecewise_compressed_load(
	struct piecewise_traj_compressed *traj, const void* data);
