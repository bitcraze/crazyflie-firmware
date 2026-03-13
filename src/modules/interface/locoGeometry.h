/**
 * locoGeometry.h - Loco anchor geometry estimation support
 *
 * Exposes inter-anchor ToF data and TDoA snapshots so a host-side script can
 * reconstruct anchor positions automatically (no manual tape-measure needed).
 */

#pragma once

/**
 * Initialize the loco geometry module.
 * Registers the MEM_TYPE_LOCO_GEOMETRY memory handler and starts the
 * background task that maintains the ToF matrix and TDoA snapshots.
 * Must be called from the loco deck init function.
 */
void locoGeometryInit(void);
