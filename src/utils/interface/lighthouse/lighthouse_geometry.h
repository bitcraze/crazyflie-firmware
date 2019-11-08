#pragma once

#include <stdbool.h>
#include "arm_math.h"

// Naive 3d vector type.
#define vec3d_size 3
typedef float vec3d[vec3d_size];

typedef struct baseStationGeometry_s {
  float origin[3];
  float mat[3][3];
} __attribute__((packed)) baseStationGeometry_t;

typedef struct baseStationEulerAngles_s {
  float roll;
  float pitch;
  float yaw;
} baseStationEulerAngles_t;

bool lighthouseGeometryGetPositionFromRayIntersection(baseStationGeometry_t baseStations[2], float angles[4], vec3d position, float *position_delta);
void lighthouseGeometryGetBaseStationPosition(baseStationGeometry_t* baseStationGeometry, vec3d baseStationPos);
void lighthouseGeometryGetRay(const baseStationGeometry_t* baseStationGeometry, const float angle1, const float angle2, vec3d ray);
bool lighthouseGeometryIntersectionPlaneVector(const vec3d linePoint, const vec3d lineVec, const vec3d planePoint, const vec3d PlaneNormal, vec3d intersectionPoint);
void lighthouseGeometryGetSensorPosition(const vec3d cfPos, const arm_matrix_instance_f32 *R, const int sensor, vec3d pos);
bool lighthouseGeometryYawDelta(const vec3d ipv, const vec3d spv, const vec3d n, float* yawDelta);
void lighthouseGeometryCalculateAnglesFromRotationMatrix(baseStationGeometry_t* baseStationGeometry, baseStationEulerAngles_t* baseStationEulerAngles);
