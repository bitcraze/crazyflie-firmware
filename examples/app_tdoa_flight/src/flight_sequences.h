/**
 * flight_sequences.h - Flight patterns for the link-free TDoA capture app.
 *
 * Sequences are data, not code: each is an array of steps run by the small
 * interpreter in tdoa_flight.c. Add one here, bump nothing else -- the table at
 * the bottom is what tdoaFlight.seq indexes into.
 *
 * COORDINATES ARE NORMALISED, deliberately. x and y are fractions of
 * tdoaFlight.workXY (so +-1.0 is the edge of the working box) and z is a
 * fraction between tdoaFlight.zLo and tdoaFlight.zHi. Re-measuring the usable
 * Lighthouse volume is then a matter of setting three params: every sequence
 * rescales itself and no table below has to be touched.
 *
 * Spiral geometry, from plan_spiral_from() in src/modules/src/planner.c:
 *   - crtpCommanderHighLevelSpiral() never sets the planner's `clockwise`
 *     field, so it is always zero -> sense = -1 -> counter-clockwise.
 *   - In forward mode the centre is derived from the CURRENT pose:
 *       xCentre = x - r0*sin(yaw),  yCentre = y + r0*cos(yaw)
 *     so at yaw = 0 the centre is r0 to the LEFT (+y). To circle the origin at
 *     radius r, be at (0, -r) with yaw 0 before the first segment.
 *   - phi saturates at +-2*pi, and the docstring calls it a spline
 *     approximation "for <= 90-degree segments", so full circles are chained
 *     90-degree steps rather than one call.
 *   - Chaining only stays concentric if each segment's r0 equals the previous
 *     segment's rf. Keep that invariant when editing.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TDOA_FLIGHT_HALF_PI 1.5707963f

typedef enum {
  STEP_END = 0,   // terminator, required as the last entry
  STEP_GOTO,      // a = x_n, b = y_n, c = z_n, d = yaw (rad), e = duration (s)
  STEP_SPIRAL,    // a = phi (rad), b = r0_n, c = rf_n, d = dz (m), e = duration (s), flag = sideways
  STEP_HOVER,     // e = duration (s); holds the current setpoint
} stepKind_t;

typedef struct {
  stepKind_t kind;
  float a, b, c, d, e;
  bool flag;
} flightStep_t;

#define GOTO(x, y, z, yaw, dur) \
  { .kind = STEP_GOTO, .a = (x), .b = (y), .c = (z), .d = (yaw), .e = (dur), .flag = false }
#define SPIRAL(phi, r0, rf, dz, dur, sideways) \
  { .kind = STEP_SPIRAL, .a = (phi), .b = (r0), .c = (rf), .d = (dz), .e = (dur), .flag = (sideways) }
#define HOVER(dur) \
  { .kind = STEP_HOVER, .a = 0, .b = 0, .c = 0, .d = 0, .e = (dur), .flag = false }
#define END() \
  { .kind = STEP_END, .a = 0, .b = 0, .c = 0, .d = 0, .e = 0, .flag = false }

typedef struct {
  const char* name;
  const flightStep_t* steps;
} flightSequence_t;

/* 0 - Static hover at the centre.
 *
 * The reference capture. A drone at rest has no antenna shadowing and no
 * estimator excitation, so replay divergence here is close to a floor: if a
 * fly-away shows up in this sequence it is not motion-induced. Also the right
 * capture for calibrating tdoa-std against the observed measurement noise.
 */
static const flightStep_t seqHover[] = {
  GOTO(0.0f, 0.0f, 0.5f, 0.0f, 4.0f),
  HOVER(90.0f),
  END(),
};

/* 1 - Waypoint box.
 *
 * Two laps of a square at constant height, with a pause at each corner. Stops
 * and starts excite the estimator hard, and the corners put the drone at four
 * distinct anchor geometries, which is what makes a location-dependent
 * fly-away attributable.
 */
static const flightStep_t seqBox[] = {
  GOTO( 0.0f,  0.0f, 0.5f, 0.0f, 4.0f),
  GOTO( 1.0f,  1.0f, 0.5f, 0.0f, 5.0f),
  HOVER(3.0f),
  GOTO(-1.0f,  1.0f, 0.5f, 0.0f, 6.0f),
  HOVER(3.0f),
  GOTO(-1.0f, -1.0f, 0.5f, 0.0f, 6.0f),
  HOVER(3.0f),
  GOTO( 1.0f, -1.0f, 0.5f, 0.0f, 6.0f),
  HOVER(3.0f),
  GOTO( 1.0f,  1.0f, 0.5f, 0.0f, 6.0f),
  GOTO(-1.0f,  1.0f, 0.5f, 0.0f, 6.0f),
  GOTO(-1.0f, -1.0f, 0.5f, 0.0f, 6.0f),
  GOTO( 1.0f, -1.0f, 0.5f, 0.0f, 6.0f),
  GOTO( 0.0f,  0.0f, 0.5f, 0.0f, 5.0f),
  END(),
};

/* 2 - Chained spirals, expanding then contracting while climbing.
 *
 * Continuous motion with no stops: the estimator is never allowed to settle,
 * and the anchor geometry sweeps smoothly instead of jumping. Complements the
 * box, which is all transients and no steady turning.
 *
 * Starts at (0, -0.40) so the first segment's centre lands on the origin.
 * Each r0 matches the previous rf to stay concentric.
 */
static const flightStep_t seqSpiral[] = {
  GOTO(0.0f, -0.40f, 0.35f, 0.0f, 5.0f),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.40f, 0.55f, 0.00f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.55f, 0.70f, 0.00f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.70f, 0.85f, 0.00f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.85f, 1.00f, 0.00f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 1.00f, 1.00f, 0.15f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 1.00f, 1.00f, 0.15f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 1.00f, 0.85f, 0.15f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.85f, 0.70f, 0.15f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.70f, 0.55f, 0.00f, 5.0f, false),
  SPIRAL(TDOA_FLIGHT_HALF_PI, 0.55f, 0.40f, 0.00f, 5.0f, false),
  GOTO(0.0f, 0.0f, 0.5f, 0.0f, 5.0f),
  END(),
};

/* 3 - Vertical sweep.
 *
 * z is TDoA's weak axis: with anchors near the floor and ceiling the vertical
 * dilution of precision is worst in the middle of the room, and it changes with
 * horizontal position too. So this steps through the full height range twice,
 * once at the centre and once off-centre, holding at each level long enough for
 * a steady-state error estimate rather than a transient.
 */
static const flightStep_t seqVertical[] = {
  GOTO(0.0f, 0.0f, 0.00f, 0.0f, 4.0f),
  HOVER(10.0f),
  GOTO(0.0f, 0.0f, 0.33f, 0.0f, 4.0f),
  HOVER(10.0f),
  GOTO(0.0f, 0.0f, 0.66f, 0.0f, 4.0f),
  HOVER(10.0f),
  GOTO(0.0f, 0.0f, 1.00f, 0.0f, 4.0f),
  HOVER(10.0f),
  GOTO(0.7f, 0.7f, 1.00f, 0.0f, 6.0f),
  HOVER(10.0f),
  GOTO(0.7f, 0.7f, 0.50f, 0.0f, 4.0f),
  HOVER(10.0f),
  GOTO(0.7f, 0.7f, 0.00f, 0.0f, 4.0f),
  HOVER(10.0f),
  GOTO(0.0f, 0.0f, 0.50f, 0.0f, 6.0f),
  END(),
};

/* Table order defines tdoaFlight.seq. Appending is safe; reordering renames
 * every capture you have already logged, since the log records the index.
 *
 * The seeded random walk is not a table -- it is generated step by step from
 * tdoaFlight.seed, at index TDOA_FLIGHT_SEQ_RANDOM (one past the last entry
 * here). See tdoa_flight.c.
 */
static const flightSequence_t flightSequences[] = {
  { .name = "hover",    .steps = seqHover },
  { .name = "box",      .steps = seqBox },
  { .name = "spiral",   .steps = seqSpiral },
  { .name = "vertical", .steps = seqVertical },
};

#define TDOA_FLIGHT_N_TABLE_SEQ \
  ((uint8_t)(sizeof(flightSequences) / sizeof(flightSequences[0])))
#define TDOA_FLIGHT_SEQ_RANDOM  TDOA_FLIGHT_N_TABLE_SEQ
