%module cffirmware
%include <stdint.i>

// ignore GNU specific compiler attributes
#define __attribute__(x)

%{
#define SWIG_FILE_WITH_INIT
#include "math3d.h"
#include "pptraj.h"
#include "planner.h"
#include "stabilizer_types.h"
#include "collision_avoidance.h"
#include "imu_types.h"
#include "controller_pid.h"
#include "position_controller.h"
#include "pid.h"
#include "filter.h"
#include "num.h"
#include "controller_mellinger.h"
#include "controller_brescianini.h"
#include "power_distribution.h"
#include "axis3fSubSampler.h"
#include "outlierFilterTdoa.h"
#include "kalman_core.h"
#include "mm_tdoa.h"
#include "mm_sweep_angles.h"
#include "outlierFilterLighthouse.h"
#include "mm_yaw_error.h"
#include "lighthouse_types.h"
#include "lighthouse_geometry.h"
#include "ootx_decoder.h"
#include "crc32.h"
#include "lighthouse_calibration.h"
%}

%include "math3d.h"
%include "pptraj.h"
%include "planner.h"
%include "stabilizer_types.h"
%include "collision_avoidance.h"
%include "controller_pid.h"
%include "imu_types.h"
%include "controller_mellinger.h"
%include "controller_brescianini.h"
%include "power_distribution.h"
%include "axis3fSubSampler.h"
%include "outlierFilterTdoa.h"
%include "kalman_core.h"
%include "mm_tdoa.h"
%include "mm_sweep_angles.h"
%include "outlierFilterLighthouse.h"
%include "mm_yaw_error.h"
%include "lighthouse_types.h"
%include "lighthouse_geometry.h"
%include "ootx_decoder.h"
%include "crc32.h"
%include "lighthouse_calibration.h"



%inline %{

float get_state(kalmanCoreData_t *data, int i)
{
    return data->S[i];
}

float get_mat_index(kalmanCoreData_t *data, int i, int j)
{
    return data->R[i][j];
}

float set_calibration_model(sweepAngleMeasurement_t *sweep, lighthouseCalibrationSweep_t *calib_in)
{
    sweep->calibrationMeasurementModel = lighthouseCalibrationMeasurementModelLh2;
    sweep->calib = calib_in;

}

void set_sensor_pos(sweepAngleMeasurement_t *sweep,  struct vec3_s sensorpos)
{
    vec3d sensorPos = {sensorpos.x, sensorpos.y, sensorpos.z};
    sweep->sensorPos = sensorPos;
}

void set_pose_origin_mat(sweepAngleMeasurement_t *sweep, vec3d  origin, mat3d mat)
{
    sweep->rotorPos = origin;
    sweep->rotorRot = mat;
}



void set_inv_mat(sweepAngleMeasurement_t *sweep, mat3d rotorRot)
{

    mat3d rotorRotInv;
    //printf("rotorRot: %f, %f, %f\n", rotorRot[0][0], rotorRot[0][1], rotorRot[0][2]);

    // Copy the transposed matrix to sweep->rotorRotInv
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rotorRotInv[i][j] = rotorRot[j][i];
    sweep->rotorRotInv = rotorRotInv;
    //printf("rotorRot: %f, %f, %f\n", sweep->rotorRot[0][0], sweep->rotorRot[0][1], sweep->rotorRot[0][2]);

}

void print_sweep_angle(sweepAngleMeasurement_t *sweep)
{
    //print all variables of struct
    printf("uint32_t timestamp: %d\n", sweep->timestamp);
    printf("sensorPos: %f, %f, %f\n", sweep->sensorPos[0], sweep->sensorPos[1], sweep->sensorPos[2]);
    printf("rotorPos: %f, %f, %f\n", sweep->rotorPos[0], sweep->rotorPos[1], sweep->rotorPos[2]);
    printf("rotorRot: %f, %f, %f\n", sweep->rotorRot[0][0], sweep->rotorRot[0][1], sweep->rotorRot[0][2]);
    printf("rotorRot: %f, %f, %f\n", sweep->rotorRot[1][0], sweep->rotorRot[1][1], sweep->rotorRot[1][2]);
    printf("rotorRot: %f, %f, %f\n", sweep->rotorRot[2][0], sweep->rotorRot[2][1], sweep->rotorRot[2][2]);
    printf("rotorRotInv: %f, %f, %f\n", sweep->rotorRotInv[0][0], sweep->rotorRotInv[0][1], sweep->rotorRotInv[0][2]);
    printf("rotorRotInv: %f, %f, %f\n", sweep->rotorRotInv[1][0], sweep->rotorRotInv[1][1], sweep->rotorRotInv[1][2]);
    printf("rotorRotInv: %f, %f, %f\n", sweep->rotorRotInv[2][0], sweep->rotorRotInv[2][1], sweep->rotorRotInv[2][2]);
    printf("sensorID: %d\n", sweep->sensorId);
    printf("baseStationID: %d\n", sweep->baseStationId);
    printf("sweepID: %d\n", sweep->sweepId);
    printf("t: %f\n", sweep->t);
    printf("measuredSweepAngle: %f\n", sweep->measuredSweepAngle);
    printf("stdDev: %f\n", sweep->stdDev);
    printf("calib: %f, %f, %f, %f, %f\n", sweep->calib->phase, sweep->calib->tilt, sweep->calib->curve, sweep->calib->gibmag, sweep->calib->gibphase);
    printf("calibrationMeasurementModel: %f\n", sweep->calibrationMeasurementModel);
}


void set_origin_mat(baseStationGeometry_t *geo, struct vec3_s *origin, struct vec3_s *mat1, struct vec3_s *mat2, struct vec3_s *mat3)
{
    geo->origin[0] = origin->x;
    geo->origin[1] = origin->y;
    geo->origin[2] = origin->z;
    geo->mat[0][0] = mat1->x;
    geo->mat[0][1] = mat1->y;
    geo->mat[0][2] = mat1->z;
    geo->mat[1][0] = mat2->x;
    geo->mat[1][1] = mat2->y;
    geo->mat[1][2] = mat2->z;
    geo->mat[2][0] = mat3->x;
    geo->mat[2][1] = mat3->y;
    geo->mat[2][2] = mat3->z;

    //print origin and mat
    printf("origin: %f, %f, %f\n", geo->origin[0], geo->origin[1], geo->origin[2]);
    printf("mat: %f, %f, %f\n", geo->mat[0][0], geo->mat[0][1], geo->mat[0][2]);
    printf("mat: %f, %f, %f\n", geo->mat[1][0], geo->mat[1][1], geo->mat[1][2]);
    printf("mat: %f, %f, %f\n", geo->mat[2][0], geo->mat[2][1], geo->mat[2][2]);
}

void set_sweep(lighthouseCalibration_t *calib, lighthouseCalibrationSweep_t sweep, int i)
{
    calib->sweep[i] = sweep;
}
void print_sweeps(lighthouseCalibration_t calib)
{
    printf("Sweep 0: phase: %f, tilt: %f, curve: %f, gibmag: %f, gibphase: %f\n", calib.sweep[0].phase, calib.sweep[0].tilt, calib.sweep[0].curve, calib.sweep[0].gibmag, calib.sweep[0].gibphase);
    printf("Sweep 1: phase: %f, tilt: %f, curve: %f, gibmag: %f, gibphase: %f\n", calib.sweep[1].phase, calib.sweep[1].tilt, calib.sweep[1].curve, calib.sweep[1].gibmag, calib.sweep[1].gibphase);
}
struct poly4d* piecewise_get(struct piecewise_traj *pp, int i)
{
    return &pp->pieces[i];
}
void poly4d_set(struct poly4d *poly, int dim, int coef, float val)
{
    poly->p[dim][coef] = val;
}
float poly4d_get(struct poly4d *poly, int dim, int coef)
{
    return poly->p[dim][coef];
}
struct poly4d* poly4d_malloc(int size)
{
    return (struct poly4d*)malloc(sizeof(struct poly4d) * size);
}
void poly4d_free(struct poly4d *p)
{
    free(p);
}

struct vec vec2svec(struct vec3_s v)
{
    return mkvec(v.x, v.y, v.z);
}

struct vec3_s svec2vec(struct vec v)
{
    struct vec3_s vv = {
        .x = v.x,
        .y = v.y,
        .z = v.z,
    };
    return vv;
}

void collisionAvoidanceUpdateSetpointWrap(
    collision_avoidance_params_t const *params,
    collision_avoidance_state_t *collisionState,
    int nOthers,
    float const *otherPositions,
    setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state)
{
    nOthers /= 3;
    float *workspace = malloc(sizeof(float) * 7 * (nOthers + 6));
    collisionAvoidanceUpdateSetpointCore(
        params,
        collisionState,
        nOthers,
        otherPositions,
        workspace,
        setpoint, sensorData, state);
    free(workspace);
}

void assertFail(char *exp, char *file, int line) {
    char buf[150];
    sprintf(buf, "%s in File: \"%s\", line %d\n", exp, file, line);

    PyErr_SetString(PyExc_AssertionError, buf);
}
%}

%pythoncode %{
import numpy as np
%}

#define COPY_CTOR(structname) \
structname(struct structname const *x) { \
    struct structname *y = malloc(sizeof(struct structname)); \
    *y = *x; \
    return y; \
} \
~structname() { \
    free($self); \
} \

%extend vec {
    COPY_CTOR(vec)

    %pythoncode %{
        def __repr__(self):
            return "({}, {}, {})".format(self.x, self.y, self.z)

        def __array__(self):
            return np.array([self.x, self.y, self.z])

        def __len__(self):
            return 3

        def __getitem__(self, i):
            if 0 <= i and i < 3:
                return _cffirmware.vindex(self, i)
            else:
                raise IndexError("vec index must be in {0, 1, 2}.")

        # Unary operator overloads.
        def __neg__(self):
            return _cffirmware.vneg(self)

        # Vector-scalar binary operator overloads.
        def __rmul__(self, s):
            return _cffirmware.vscl(s, self)

        def __div__(self, s):
            return self.__truediv__(s)

        def __truediv__(self, s):
            return _cffirmware.vdiv(self, s)

        # Vector-vector binary operator overloads.
        def __add__(self, other):
            return _cffirmware.vadd(self, other)

        def __sub__(self, other):
            return _cffirmware.vsub(self, other)
    %}
};

%extend traj_eval {
    COPY_CTOR(traj_eval)
};
