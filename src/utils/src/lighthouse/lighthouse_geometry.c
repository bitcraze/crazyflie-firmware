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

float vec_length(vec3d vec) {
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

bool lighthouseGeometryBestFitBetweenRays(vec3d orig0, vec3d orig1, vec3d u, vec3d v, vec3d D, vec3d pt0, vec3d pt1)
{

	//calculate 'm'
	float32_t m[1];
	{
		arm_matrix_instance_f32 u_t_mat = {1, 3, u}; //fast transpose
		arm_matrix_instance_f32 v_mat = {3, 1, v};
		arm_matrix_instance_f32 m_mat = {1, 1, m};
		arm_mat_mult_f32(&u_t_mat, &v_mat, &m_mat);
	}


	//calculate 'c'
	float32_t c[1];
	{
		vec3d Dw;
		{
			vec3d w;
			arm_sub_f32(orig1, orig0, w, vec3d_size);
			arm_sub_f32(D, w, Dw, vec3d_size);
		}

		arm_matrix_instance_f32 Dw_t_mat = {1, 3, Dw}; //faster way to create a transpose
		arm_matrix_instance_f32 c_mat = {1, 1, c};
		arm_matrix_instance_f32 v_mat = {3, 1, v};
		arm_mat_mult_f32(&Dw_t_mat, &v_mat, &c_mat);
	}


	#define N_ROWS 3
	#define N_COLS 1

	//solve for 'x0'
	float x0[N_COLS];
	{
		// A*x0 = B
		// equivalents in MATLAB:
		// x0 = A\B
		// x0 = linsolve(A,B)
		// x0 = pinv(A)*B


		//obtain SVD of A
		float U[N_ROWS][N_COLS];
		float V[N_COLS][N_COLS];
		float singular_values[N_COLS];
		{
			//calculate 'A'
			vec3d A;
			{
				vec3d vm;
				{
					arm_matrix_instance_f32 vm_mat = {3, 1, vm};
					arm_matrix_instance_f32 v_mat = {3, 1, v};
					arm_matrix_instance_f32 m_mat = {1, 1, m};
					arm_mat_mult_f32(&v_mat, &m_mat, &vm_mat);
				}

				arm_sub_f32(u, vm, A, vec3d_size);
			}

			float dummy_array[N_COLS];
			int8_t svdError = Singular_Value_Decomposition((float*)A, N_ROWS, N_COLS, (float*)U, singular_values, (float*)V, dummy_array);
			if (svdError != 0) { //cannot converge
					return false;
			}
		}
		//found SVD of A

		//calculate 'B'
		vec3d B;
		{
			vec3d vc;
			{
				arm_matrix_instance_f32 vc_mat = {3, 1, vc};
				arm_matrix_instance_f32 v_mat = {3, 1, v};
				arm_matrix_instance_f32 c_mat = {1, 1, c};
				arm_mat_mult_f32(&v_mat, &c_mat, &vc_mat);
			}

			{
				vec3d wD;
				{
					vec3d w;
					arm_sub_f32(orig1, orig0, w, vec3d_size);
					arm_sub_f32(w, D, wD, vec3d_size);
				}

				arm_add_f32(wD, vc, B, vec3d_size);
			}
		}
		//found B

		//solve for 'x0' using SVD of A and calculated B
		#define tolerance 0.0001f
		Singular_Value_Decomposition_Solve((float*) U, singular_values, (float*) V, tolerance, N_ROWS, N_COLS, B, x0);
	}


	//obtain the 2 points on each of the 2 rays

	//calculate 'pt0' using 'x0'
	{
		vec3d ux0;
		{
			arm_matrix_instance_f32 ux0_mat = {3, 1, ux0};
			arm_matrix_instance_f32 u_mat = {3, 1, u};
			arm_matrix_instance_f32 x0_mat = {N_COLS, N_COLS, x0};
			arm_mat_mult_f32(&u_mat, &x0_mat, &ux0_mat);
		}
		arm_add_f32(orig0, ux0, pt0, vec3d_size);
	}

	//calculate 'x1'
	float x1[N_COLS];
	{
		float mx0[N_COLS];
		arm_matrix_instance_f32 mx0_mat = {1, 1, mx0};
		{
			arm_matrix_instance_f32 m_mat = {1, 1, m};
			arm_matrix_instance_f32 x0_mat = {N_COLS, N_COLS, x0};
			arm_mat_mult_f32(&m_mat, &x0_mat, &mx0_mat);
		}
		{
			arm_matrix_instance_f32 x1_mat = {1, 1, x1};
			arm_matrix_instance_f32 c_mat = {1, 1, c};
			arm_mat_add_f32(&mx0_mat, &c_mat, &x1_mat);
		}
	}

	//calculate 'pt1' using 'x1'
	{
		vec3d vx1;
		{
			arm_matrix_instance_f32 vx1_mat = {3, 1, vx1};
			arm_matrix_instance_f32 v_mat = {3, 1, v};
			arm_matrix_instance_f32 x1_mat = {1, 1, x1};
			arm_mat_mult_f32(&v_mat, &x1_mat, &vx1_mat);
		}
			arm_add_f32(orig1, vx1, pt1, vec3d_size);
	}

	return true;
}
