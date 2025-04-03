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
#include "controller_lee.h"
#include "power_distribution.h"
#include "axis3fSubSampler.h"
#include "outlierFilterTdoa.h"
#include "kalman_core.h"
#include "mm_tdoa.h"
#include "mm_sweep_angles.h"
#include "mm_yaw_error.h"
#include "outlierFilterLighthouse.h"
#include "lighthouse_calibration.h"
#include "lighthouse_types.h"
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
%include "controller_lee.h"
%include "power_distribution.h"
%include "axis3fSubSampler.h"
%include "outlierFilterTdoa.h"
%include "kalman_core.h"
%include "mm_tdoa.h"
%include "mm_sweep_angles.h"
%include "mm_yaw_error.h"
%include "outlierFilterLighthouse.h"
%include "lighthouse_calibration.h"
%include "lighthouse_types.h"


%inline %{
void set_calibration_model(sweepAngleMeasurement_t *sweep, lighthouseCalibrationSweep_t *calib_in)
{
    sweep->calibrationMeasurementModel = lighthouseCalibrationMeasurementModelLh2;
    sweep->calib = calib_in;
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

vec3d* make_vec3d(float x, float y, float z) {
    vec3d* v = (vec3d*)malloc(sizeof(vec3d));
    (*v)[0] = x;
    (*v)[1] = y;
    (*v)[2] = z;
    return v;
}

void free_vec3d(vec3d* v) {
    free(v);
}

mat3d* make_mat3d(float m00, float m01, float m02,
                  float m10, float m11, float m12,
                  float m20, float m21, float m22) {
    mat3d* m = (mat3d*)malloc(sizeof(mat3d));
    (*m)[0][0] = m00; (*m)[0][1] = m01; (*m)[0][2] = m02;
    (*m)[1][0] = m10; (*m)[1][1] = m11; (*m)[1][2] = m12;
    (*m)[2][0] = m20; (*m)[2][1] = m21; (*m)[2][2] = m22;
    return m;
}

void free_mat3d(mat3d* m) {
    free(m);
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