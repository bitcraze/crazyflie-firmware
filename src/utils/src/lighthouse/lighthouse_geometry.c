/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Code addapted from ashtuchkin's Vive DIY position sensor
 * see https://github.com/ashtuchkin/vive-diy-position-sensor/
 * 
 * Copyright (c) 2016 Alexander Shtuchkin
 * Copyright (C) 2018 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouseGeometry.c: lighthouse tracking system geometry functions
 */

#include "lighthouse_geometry.h"

#include <arm_math.h>
#include "singular_value_decomposition.h"

static void vec_cross_product(const vec3d a, const vec3d b, vec3d res) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

static float vec_length(vec3d vec) {
    float pow, res;

    arm_power_f32(vec, vec3d_size, &pow); // returns sum of squares
    arm_sqrt_f32(pow, &res);

    return res;
}

void calc_ray_vec(baseStationGeometry_t *bs, float angle1, float angle2, vec3d res, vec3d origin) {


    vec3d a = {arm_cos_f32(angle1), 0, -arm_sin_f32(angle1)};  // Normal vector to X plane
    vec3d b = {0, arm_cos_f32(angle2), arm_sin_f32(angle2)};   // Normal vector to Y plane

    vec3d ray = {};
    vec_cross_product(b, a, ray); // Intersection of two planes -> ray vector.
    float len = vec_length(ray);
    arm_scale_f32(ray, 1/len, ray, vec3d_size); // Normalize ray length.

    arm_matrix_instance_f32 source_rotation_matrix = {3, 3, (float32_t *)bs->mat};
    arm_matrix_instance_f32 ray_vec = {3, 1, ray};
    arm_matrix_instance_f32 ray_rotated_vec = {3, 1, res};
    arm_mat_mult_f32(&source_rotation_matrix, &ray_vec, &ray_rotated_vec);

    // TODO: Make geometry adjustments within base station.
    vec3d rotated_origin_delta = {};
    //vec3d base_origin_delta = {-0.025f, -0.025f, 0.f};  // Rotors are slightly off center in base station.
    // arm_matrix_instance_f32 origin_vec = {3, 1, base_origin_delta};
    // arm_matrix_instance_f32 origin_rotated_vec = {3, 1, rotated_origin_delta};
    // arm_mat_mult_f32(&source_rotation_matrix, &origin_vec, &origin_rotated_vec);
    arm_add_f32(bs->origin, rotated_origin_delta, origin, vec3d_size);
}


static bool intersect_lines(vec3d orig1, vec3d vec1, vec3d orig2, vec3d vec2, vec3d res, float *dist) {
    // Algoritm: http://geomalgorithms.com/a07-_distance.html#Distance-between-Lines

    vec3d w0 = {};
    arm_sub_f32(orig1, orig2, w0, vec3d_size);

    float a, b, c, d, e;
    arm_dot_prod_f32(vec1, vec1, vec3d_size, &a);
    arm_dot_prod_f32(vec1, vec2, vec3d_size, &b);
    arm_dot_prod_f32(vec2, vec2, vec3d_size, &c);
    arm_dot_prod_f32(vec1, w0, vec3d_size, &d);
    arm_dot_prod_f32(vec2, w0, vec3d_size, &e);

    float denom = a * c - b * b;
    if (fabsf(denom) < 1e-5f)
        return false;

    // Closest point to 2nd line on 1st line
    float t1 = (b * e - c * d) / denom;
    vec3d pt1 = {};
    arm_scale_f32(vec1, t1, pt1, vec3d_size);
    arm_add_f32(pt1, orig1, pt1, vec3d_size);

    // Closest point to 1st line on 2nd line
    float t2 = (a * e - b * d) / denom;
    vec3d pt2 = {};
    arm_scale_f32(vec2, t2, pt2, vec3d_size);
    arm_add_f32(pt2, orig2, pt2, vec3d_size);

    // Result is in the middle
    vec3d tmp = {};
    arm_add_f32(pt1, pt2, tmp, vec3d_size);
    arm_scale_f32(tmp, 0.5f, res, vec3d_size);

    // Dist is distance between pt1 and pt2
    arm_sub_f32(pt1, pt2, tmp, vec3d_size);
    *dist = vec_length(tmp);

    return true;
}

bool lighthouseGeometryGetPosition(baseStationGeometry_t baseStations[2], float angles[4], vec3d position, float *position_delta)
{
  static vec3d ray1, ray2, origin1, origin2;

  calc_ray_vec(&baseStations[0], angles[0], angles[1], ray1, origin1);
  calc_ray_vec(&baseStations[1], angles[2], angles[3], ray2, origin2);

  return intersect_lines(origin1, ray1, origin2, ray2, position, position_delta);
}

bool lighthouseGeometryBestFitBetweenRays(vec3d _orig0, vec3d _orig1, vec3d _u, vec3d _v, vec3d _D, vec3d pt0, vec3d pt1)
{


	vec3d orig1;
	memcpy(orig1, _orig1, sizeof(vec3d));

	vec3d orig0;
	memcpy(orig0, _orig0, sizeof(vec3d));

	vec3d u;
	memcpy(u, _u, sizeof(vec3d));

	vec3d v;
	memcpy(v, _v, sizeof(vec3d));

	vec3d D;
	memcpy(D, _D, sizeof(vec3d));


//	vec3d orig0;
//	memcpy(orig0, rays[0].origin, sizeof(vec3d));
//			vec3d orig0 = {-2.611738, 2.682860, -1.736229};


//	vec3d orig1;
//	memcpy(orig1, rays[1].origin, sizeof(vec3d));
//			vec3d orig1 = {2.375781, 2.739366, 1.372339};


	vec3d w;
	arm_sub_f32(orig1, orig0, w, vec3d_size);


	vec3d wD;
	arm_matrix_instance_f32 wD_mat = {3, 1, wD};
	arm_sub_f32(w, D, wD, vec3d_size);


	vec3d Dw;
	arm_matrix_instance_f32 Dw_mat = {3, 1, Dw};
	arm_sub_f32(D, w, Dw, vec3d_size);


//	vec3d u;
//	vec3d u = {0.79735142, -0.581848264, 0.160261855};
//	memcpy(u, rays[0].direction, sizeof(vec3d));
	arm_matrix_instance_f32 u_mat = {3, 1, u};
	// arm_matrix_instance_f32 u_mat = {3, 1, rays[0].direction}; //can work


	vec3d u_t;
	arm_matrix_instance_f32 u_t_mat = {1, 3, u_t};
	arm_mat_trans_f32(&u_mat, &u_t_mat);


//	vec3d v;
//	vec3d v = {-0.47880125, -0.564612567, -0.672280788};
//	memcpy(v, rays[1].direction, sizeof(vec3d));
	arm_matrix_instance_f32 v_mat = {3, 1, v};
// arm_matrix_instance_f32 v_mat = {3, 1, rays[1].direction}; //cannot work


	float32_t m[1];
	arm_matrix_instance_f32 m_mat = {1, 1, m};
	arm_mat_mult_f32(&u_t_mat, &v_mat, &m_mat);


	vec3d Dw_t;
	arm_matrix_instance_f32 Dw_t_mat = {1, 3, Dw_t};
	arm_mat_trans_f32(&Dw_mat, &Dw_t_mat);


	float32_t c[1];
	arm_matrix_instance_f32 c_mat = {1, 1, c};
	arm_mat_mult_f32(&Dw_t_mat, &v_mat, &c_mat);


	vec3d vc;
	arm_matrix_instance_f32 vc_mat = {3, 1, vc};
	arm_mat_mult_f32(&v_mat, &c_mat, &vc_mat);


	vec3d vm;
	arm_matrix_instance_f32 vm_mat = {3, 1, vm};
	arm_mat_mult_f32(&v_mat, &m_mat, &vm_mat);


	vec3d A;
	arm_matrix_instance_f32 A_mat = {3, 1, A};
	arm_sub_f32(u, vm, A, vec3d_size);


	vec3d B;
	arm_matrix_instance_f32 B_mat = {3, 1, B};
	arm_add_f32(wD, vc, B, vec3d_size);


	float x0[1];
	arm_matrix_instance_f32 x0_mat = {1, 1, x0};






//	linsolve(A_mat, B_mat, x0_mat);
//
//
//}
//
//
//void linsolve(const arm_matrix_instance_f32 * A_mat, const arm_matrix_instance_f32 * B_mat, arm_matrix_instance_f32 * x0_mat){
//
//	float32_t *A = A_mat->pData;

	#define N_ROWS 3
	#define N_COLS 1


	float U[N_ROWS][N_COLS];
	float singular_values[N_COLS];
	float V[N_COLS][N_COLS];
//	float dummy_array[N_COLS];
//  int svdError = Singular_Value_Decomposition(&A, N_ROWS, N_COLS, &U, &singular_values, &V, &dummy_array);


	float* dummy_array;
	dummy_array = (float*) malloc(N_COLS * sizeof(float));
	if (dummy_array == NULL) {
		//	 printf(" No memory available");
		return false;
	}
//	size_t freerambytes = xPortGetFreeHeapSize();
	int svdError = Singular_Value_Decomposition((float*)A, N_ROWS, N_COLS, (float*)U, singular_values, (float*)V, dummy_array);
	if (svdError != 0) {
 		return false;
	}


	arm_matrix_instance_f32 U_mat = {N_ROWS, N_COLS, U};


  float tolerance = 0.0001f;
	float D_inv[N_COLS];
  for ( uint8_t i = 0; i < N_COLS; i++ ) {
  	if(singular_values[i] >= tolerance){
    	D_inv[i] = 1/singular_values[i]; // conditional element-wise invert (this case only first element), ref: https://www.cse.unr.edu/~bebis/CS791E/Notes/SVD.pdf (page 2)
  	}
  }
	arm_matrix_instance_f32 D_inv_mat = {N_COLS, N_COLS, D_inv};


	arm_matrix_instance_f32 V_mat = {N_COLS, N_COLS, V};


	float VD_inv[N_COLS];
	arm_matrix_instance_f32 VD_inv_mat = {N_COLS, N_COLS, VD_inv};
	arm_mat_mult_f32(&V_mat, &D_inv_mat, &VD_inv_mat);


	vec3d U_t;
	arm_matrix_instance_f32 U_t_mat = {N_COLS, N_ROWS, U_t};
	arm_mat_trans_f32(&U_mat, &U_t_mat);


	float A_inv[N_COLS][N_ROWS];
	arm_matrix_instance_f32 A_inv_mat = {N_COLS, N_ROWS, A_inv};
	arm_mat_mult_f32(&VD_inv_mat, &U_t_mat, &A_inv_mat);


//	float x0[N_COLS];
//	arm_matrix_instance_f32 x0_mat = {N_COLS, N_COLS, x0};


	arm_mat_mult_f32(&A_inv_mat, &B_mat, &x0_mat);







	float mx0[N_COLS];
	arm_matrix_instance_f32 mx0_mat = {1, 1, mx0};
	arm_mat_mult_f32(&m_mat, &x0_mat, &mx0_mat);


	float x1[N_COLS];
	arm_matrix_instance_f32 x1_mat = {1, 1, x1};
	arm_mat_add_f32(&mx0_mat, &c_mat, &x1_mat);


	vec3d ux0;
	arm_matrix_instance_f32 ux0_mat = {3, 1, ux0};
	arm_mat_mult_f32(&u_mat, &x0_mat, &ux0_mat);


	vec3d vx1;
	arm_matrix_instance_f32 vx1_mat = {3, 1, vx1};
	arm_mat_mult_f32(&v_mat, &x1_mat, &vx1_mat);


	arm_matrix_instance_f32 orig0_mat = {3, 1, orig0};


	arm_matrix_instance_f32 orig1_mat = {3, 1, orig1};


	arm_matrix_instance_f32 pt0_mat = {3, 1, pt0};
	arm_mat_add_f32(&orig0_mat, &ux0_mat, &pt0_mat);
//	vec3d pt0;
//  arm_add_f32(orig0, ux0, pt0, vec3d_size);
//	memcpy(_pt0, pt0, sizeof(vec3d));


	arm_matrix_instance_f32 pt1_mat = {3, 1, pt1};
	arm_mat_add_f32(&orig1_mat, &vx1_mat, &pt1_mat);
//	vec3d pt1;
//  arm_add_f32(orig1, vx1, pt1, vec3d_size);
//	memcpy(_pt1, pt1, sizeof(vec3d));


	return true;
}
