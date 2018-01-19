#pragma once

/*
The MIT License (MIT)

cmath3d
Copyright (c) 2016 James Alan Preiss

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

#include <math.h>
#include <stdbool.h>

#ifndef M_PI_F
#define M_PI_F (3.14159265358979323846f)
#endif


// ----------------------------- scalars --------------------------------

static inline float fsqr(float x) { return x * x; }
static inline float radians(float degrees) { return (M_PI_F / 180.0f) * degrees; }
static inline float degrees(float radians) { return (180.0f / M_PI_F) * radians; }


// ---------------------------- 3d vectors ------------------------------

struct vec {
	float x; float y; float z;
};

//
// constructors
//

// construct a vector from 3 floats.
static inline struct vec mkvec(float x, float y, float z) {
	struct vec v;
	v.x = x; v.y = y; v.z = z;
	return v;
}
// construct a vector with the same value repeated for x, y, and z.
static inline struct vec vrepeat(float x) {
	return mkvec(x, x, x);
}
// construct a zero-vector.
static inline struct vec vzero(void) {
	return vrepeat(0.0f);
}

//
// operators
//

// multiply a vector by a scalar.
static inline struct vec vscl(float s, struct vec v) {
	return mkvec(s * v.x , s * v.y, s * v.z);
}
// negate a vector.
static inline struct vec vneg(struct vec v) {
	return mkvec(-v.x, -v.y, -v.z);
}
// divide a vector by a scalar.
// does not perform divide-by-zero check.
static inline struct vec vdiv(struct vec v, float s) {
	return vscl(1.0f/s, v);
}
// add two vectors.
static inline struct vec vadd(struct vec a, struct vec b) {
	return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}
// subtract a vector from another vector.
static inline struct vec vsub(struct vec a, struct vec b) {
	return vadd(a, vneg(b));
}
// vector dot product.
static inline float vdot(struct vec a, struct vec b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
// vector magnitude squared.
static inline float vmag2(struct vec v) {
	return vdot(v, v);
}
// vector magnitude.
static inline float vmag(struct vec v) {
	return sqrt(vmag2(v));
}
// vector Euclidean distance squared.
static inline float vdist2(struct vec a, struct vec b) {
  return vmag2(vsub(a, b));
}
// vector Euclidean distance.
static inline float vdist(struct vec a, struct vec b) {
  return sqrt(vdist2(a, b));
}
// normalize a vector (make a unit vector).
static inline struct vec vnormalize(struct vec v) {
	return vdiv(v, vmag(v));
}
// vector cross product.
static inline struct vec vcross(struct vec a, struct vec b) {
	return mkvec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
// projection of a onto b, where b is a unit vector.
static inline struct vec vprojectunit(struct vec a, struct vec b_unit) {
	return vscl(vdot(a, b_unit), b_unit);
}
// component of a orthogonal to b, where b is a unit vector.
static inline struct vec vorthunit(struct vec a, struct vec b_unit) {
	return vsub(a, vprojectunit(a, b_unit));
}
// element-wise absolute value of vector.
static inline struct vec vabs(struct vec v) {
	return mkvec(fabs(v.x), fabs(v.y), fabs(v.z));
}
// element-wise minimum of vector.
static inline struct vec vmin(struct vec a, struct vec b) {
	return mkvec(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
// element-wise maximum of vector.
static inline struct vec vmax(struct vec a, struct vec b) {
	return mkvec(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}
static inline float vminkowski(struct vec v) {
	return fabs(v.x) + fabs(v.y) + fabs(v.z);
}

//
// comparisons
//

// compare two vectors for exact equality.
static inline bool veq(struct vec a, struct vec b) {
	return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}
// compare two vectors for exact inequality.
static inline bool vneq(struct vec a, struct vec b) {
	return !veq(a, b);
}
// element-wise less-than
static inline bool vless(struct vec a, struct vec b) {
	return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}
// element-wise less-than-or-equal
static inline bool vleq(struct vec a, struct vec b) {
	return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
}
// element-wise greater-than
static inline bool vgreater(struct vec a, struct vec b) {
	return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}
// element-wise greater-than-or-equal
static inline bool vgeq(struct vec a, struct vec b) {
	return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
}
// test if any element of a vector is NaN.
static inline bool visnan(struct vec v) {
	return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

//
// special functions to ease the pain of writing vector math in C.
//

// add 3 vectors.
static inline struct vec vadd3(struct vec a, struct vec b, struct vec c) {
	return vadd(vadd(a, b), c);
}
// subtract b and c from a.
static inline struct vec vsub2(struct vec a, struct vec b, struct vec c) {
	return vadd3(a, vneg(b), vneg(c));
}

//
// conversion to/from raw float and double arrays.
//

// load a vector from a double array.
static inline struct vec vload(double const *d) {
	return mkvec(d[0], d[1], d[2]);
}
// store a vector into a double array.
static inline void vstore(struct vec v, double *d) {
	d[0] = v.x; d[1] = v.y; d[2] = v.z;
}
// load a vector from a float array.
static inline struct vec vloadf(float const *f) {
	return mkvec(f[0], f[1], f[2]);
}
// store a vector into a float array.
static inline void vstoref(struct vec v, float *f) {
	f[0] = v.x; f[1] = v.y; f[2] = v.z;
}
// Vector TODO: fuzzy comparison


// ---------------------------- 3x3 matrices ------------------------------

struct mat33 {
	float m[3][3];
};

//
// constructors
//

// construct a zero matrix.
static inline struct mat33 mzero(void) {
	struct mat33 m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m.m[i][j] = 0;
		}
	}
	return m;
}
// construct a matrix with the given diagonal.
static inline struct mat33 diag(float a, float b, float c) {
	struct mat33 m = mzero();
	m.m[0][0] = a;
	m.m[1][1] = b;
	m.m[2][2] = c;
	return m;
}
// construct the matrix a * I for scalar a.
static inline struct mat33 eyescl(float a) {
	return diag(a, a, a);
}
// construct an identity matrix.
static inline struct mat33 eye(void) {
	return eyescl(1.0f);
}
// construct a matrix from three column vectors.
static inline struct mat33 mcolumns(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	m.m[0][0] = a.x;
	m.m[1][0] = a.y;
	m.m[2][0] = a.z;

	m.m[0][1] = b.x;
	m.m[1][1] = b.y;
	m.m[2][1] = b.z;

	m.m[0][2] = c.x;
	m.m[1][2] = c.y;
	m.m[2][2] = c.z;

	return m;
}
// construct a matrix from three row vectors.
static inline struct mat33 mrows(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	vstoref(a, m.m[0]);
	vstoref(b, m.m[1]);
	vstoref(c, m.m[2]);
	return m;
}
// construct the matrix A from vector v such that Ax = cross(v, x)
static inline struct mat33 mcrossmat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}

//
// accessors
//

// return one column of a matrix as a vector.
static inline struct vec mcolumn(struct mat33 m, int col) {
	return mkvec(m.m[0][col], m.m[1][col], m.m[2][col]);
}
// return one row of a matrix as a vector.
static inline struct vec mrow(struct mat33 m, int row) {
	return vloadf(m.m[row]);
}

//
// operators
//

// matrix transpose.
static inline struct mat33 mtranspose(struct mat33 m) {
	struct mat33 mt;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			mt.m[i][j] = m.m[j][i];
		}
	}
	return mt;
}
// multiply a matrix by a scalar.
static inline struct mat33 mscale(float s, struct mat33 a) {
	struct mat33 sa;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sa.m[i][j] = s * a.m[i][j];
		}
	}
	return sa;
}
// negate a matrix.
static inline struct mat33 mneg(struct mat33 a) {
	return mscale(-1.0, a);
}
// add two matrices.
static inline struct mat33 madd(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] + b.m[i][j];
		}
	}
	return c;
}
// subtract two matrices.
static inline struct mat33 msub(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] - b.m[i][j];
		}
	}
	return c;
}
// multiply a matrix by a vector.
static inline struct vec mvmult(struct mat33 a, struct vec v) {
	float x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z;
	float y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z;
	float z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z;
	return mkvec(x, y, z);
}
// multiply two matrices.
static inline struct mat33 mmult(struct mat33 a, struct mat33 b) {
	struct mat33 ab;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float accum = 0;
			for (int k = 0; k < 3; ++k) {
				accum += a.m[i][k] * b.m[k][j];
			}
			ab.m[i][j] = accum;
		}
	}
	return ab;
}
// add a scalar along the diagonal, i.e. a + dI.
static inline struct mat33 maddridge(struct mat33 a, float d) {
	a.m[0][0] += d;
	a.m[1][1] += d;
	a.m[2][2] += d;
	return a;
}
// test if any element of a matrix is NaN.
static inline bool misnan(struct mat33 m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (isnan(m.m[i][j])) {
				return true;
			}
		}
	}
	return false;
}
// set a 3x3 block within a big row-major matrix.
// block: pointer to the upper-left element of the block in the big matrix.
// stride: the number of columns in the big matrix.
static inline void set_block33_rowmaj(float *block, int stride, struct mat33 const *m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			block[j] = m->m[i][j];
		}
		block += stride;
	}
}

// Matrix TODO: inv, solve, eig, raw floats, axis-aligned rotations


// ---------------------------- quaternions ------------------------------

struct quat {
	float x;
	float y;
	float z;
	float w;
};

//
// constructors
//

// construct a quaternion from its x, y, z, w elements.
static inline struct quat mkquat(float x, float y, float z, float w) {
	struct quat q;
	q.x = x; q.y = y; q.z = z; q.w = w;
	return q;
}
// construct a quaternion from a vector containing (x, y, z) and a scalar w.
// note that this is NOT an axis-angle constructor.
static inline struct quat quatvw(struct vec v, float w) {
	struct quat q;
	q.x = v.x; q.y = v.y; q.z = v.z;
	q.w = w;
	return q;
}
// construct an identity quaternion.
static inline struct quat qeye(void) {
	return mkquat(0, 0, 0, 1);
}
// construct a quaternion from an axis and angle of rotation.
static inline struct quat qaxisangle(struct vec axis, float angle) {
	float scale = sinf(angle / 2) / vmag(axis);
	struct quat q;
	q.x = scale * axis.x;
	q.y = scale * axis.y;
	q.z = scale * axis.z;
	q.w = cos(angle/2);
	return q;
}
static inline struct vec quataxis(struct quat q) {
	// TODO this is not numerically stable for tiny rotations
	float s = 1.0f / sqrtf(1 - q.w * q.w);
	return vscl(s, mkvec(q.x, q.y, q.z));
}
static inline float quatangle(struct quat q) {
	return 2 * acosf(q.w);
}
// APPROXIMATE conversion of small (roll, pitch, yaw) Euler angles
// into a quaternion without computing any trig functions.
// only produces useful result for small angles.
// Example application is integrating a gyroscope when the angular velocity
// of the object is small compared to the sampling frequency.
static inline struct quat rpy2quat_small(struct vec rpy) {
	float q2 = vmag2(rpy) / 4.0f;
	if (q2 < 1) {
		return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
	}
	else {
		float w = 1.0f / sqrtf(1.0f + q2);
		return quatvw(vscl(w/2, rpy), w);
	}
}
static inline struct vec quatimagpart(struct quat q) {
	return mkvec(q.x, q.y, q.z);
}

//
// conversions to other parameterizations of 3D rotations
//

// convert quaternion to (roll, pitch, yaw) Euler angles
static inline struct vec quat2rpy(struct quat q) {
	struct vec v;
	v.x = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (fsqr(q.x) + fsqr(q.y))); // roll
	v.y = asin(2 * (q.w * q.y - q.x * q.z)); // pitch
	v.z = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (fsqr(q.y) + fsqr(q.z))); // yaw
	return v;
}
// compute the axis of a quaternion's axis-angle decomposition.
static inline struct vec quat2axis(struct quat q) {
	// TODO this is not numerically stable for tiny rotations
	float s = 1.0f / sqrtf(1 - q.w * q.w);
	return vscl(s, mkvec(q.x, q.y, q.z));
}
// compute the angle of a quaternion's axis-angle decomposition.
static inline float quat2angle(struct quat q) {
	return 2 * acos(q.w);
}
// convert a quaternion into a 3x3 rotation matrix.
static inline struct mat33 quat2rotmat(struct quat q) {
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;

	struct mat33 m;
	m.m[0][0] = 1 - 2*y*y - 2*z*z;
	m.m[0][1] = 2*x*y - 2*z*w;
	m.m[0][2] = 2*x*z + 2*y*w,
	m.m[1][0] = 2*x*y + 2*z*w;
	m.m[1][1] = 1 - 2*x*x - 2*z*z;
	m.m[1][2] = 2*y*z - 2*x*w,
	m.m[2][0] = 2*x*z - 2*y*w;
	m.m[2][1] = 2*y*z + 2*x*w;
	m.m[2][2] = 1 - 2*x*x - 2*y*y;
	return m;
}

//
// operators
//

// rotate a vector by a quaternion.
static inline struct vec qvrot(struct quat q, struct vec v) {
	// from http://gamedev.stackexchange.com/a/50545 - TODO find real citation
	struct vec qv = mkvec(q.x, q.y, q.z);
	return vadd3(
		vscl(2.0f * vdot(qv, v), qv),
		vscl(q.w * q.w - vmag2(qv), v),
		vscl(2.0f * q.w, vcross(qv, v))
	);
}
// multiply (compose) two quaternions
// such that qvrot(qqmul(q, p), v) == qvrot(q, qvrot(p, v)).
static inline struct quat qqmul(struct quat q, struct quat p) {
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
	return mkquat(x, y, z, w);
}
// invert a quaternion.
static inline struct quat qinv(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, q.w);
}
// negate a quaternion.
// this represents the same rotation, but is still sometimes useful.
static inline struct quat qneg(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, -q.w);
}
// quaternion dot product. is cosine of angle between them.
static inline float qdot(struct quat a, struct quat b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
// normalize a quaternion.
// typically used to mitigate precision errors.
static inline struct quat qnormalize(struct quat q) {
	float s = 1.0f / sqrtf(qdot(q, q));
	return mkquat(s*q.x, s*q.y, s*q.z, s*q.w);
}
// update an attitude estimate quaternion with a reading from a gyroscope
// over the timespan dt. Gyroscope is assumed (roll, pitch, yaw)
// angular velocities in radians per second.
static inline struct quat quat_gyro_update(struct quat quat, struct vec gyro, float const dt) {
	// from "Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005
	struct quat q1;
	float const r = (dt / 2) * gyro.x;
	float const p = (dt / 2) * gyro.y;
	float const y = (dt / 2) * gyro.z;

	q1.x =    quat.x + y*quat.y - p*quat.z + r*quat.w;
	q1.y = -y*quat.x +   quat.y + r*quat.z + p*quat.w;
	q1.z =  p*quat.x - r*quat.y +   quat.z + y*quat.w;
	q1.w = -r*quat.x - p*quat.y - y*quat.z +   quat.w;
	return q1;
}
// normalized linear interpolation. s should be between 0 and 1.
static inline struct quat qnlerp(struct quat a, struct quat b, float t) {
	float s = 1.0f - t;
	return qnormalize(mkquat(
		s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w));
}
// spherical linear interpolation. s should be between 0 and 1.
static inline struct quat qslerp(struct quat a, struct quat b, float t)
{
	float dp = qdot(a, b);
	if (dp < 0) {
		dp = -dp;
		b = qneg(b);
	}

	if (dp > 0.99f) {
		// fall back to linear interpolation to avoid div-by-zero
		return qnlerp(a, b, t);
	}
	else {
		float theta = acos(dp);
		float s = sin(theta * (1 - t)) / sin(theta);
		t = sin(theta * t) / sin(theta);
		return mkquat(
			s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w);
	}
}

//
// conversion to/from raw float and double arrays.
//

// load a quaternion from a raw double array.
static inline struct quat qload(double const *d) {
	return mkquat(d[0], d[1], d[2], d[3]);
}
// store a quaternion into a raw double array.
static inline void qstore(struct quat q, double *d) {
	d[0] = q.x; d[1] = q.y; d[2] = q.z; d[3] = q.w;
}
// load a quaternion from a raw float array.
static inline struct quat qloadf(float const *f) {
	return mkquat(f[0], f[1], f[2], f[3]);
}
// store a quaternion into a raw float array.
static inline void qstoref(struct quat q, float *f) {
	f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}

// Quaternion TODO: rpy2quat

// Overall TODO: lines? segments? planes? axis-aligned boxes? spheres?
