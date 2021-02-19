/** 
 * The robust M-estimation-based Kalman filter is originally implemented as a part of the work in
 * the below-cited paper. 
 * 
 * "Learning-based Bias Correction for Time Difference of Arrival Ultra-wideband Localization of 
 *  Resource-constrained Mobile Robots"
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @ARTICLE{WendaBiasLearning2021,
      author={Wenda Zhao, Jacopo Panerati, and Angela P. Schoellig},
      title={Learning-based Bias Correction for Time Difference of Arrival Ultra-wideband Localization of 
             Resource-constrained Mobile Robots},
      journal={IEEE Robotics and Automation Letters},
      year={2021},
      publisher={IEEE}
 *
 * The authors are with the Dynamic Systems Lab, Institute for Aerospace Studies,
 * University of Toronto, Canada, and affiliated with the Vector Institute for Artificial
 * Intelligence in Toronto. 
 * ============================================================================
 */
#include "mm_tdoa_robust.h"
#include "test_support.h"
     
#define MAX_ITER (2) // maximum iteration is set to 2. 

// Cholesky Decomposition for a nxn psd matrix (from scratch)
// Reference: https://www.geeksforgeeks.org/cholesky-decomposition-matrix-decomposition/
static void Cholesky_Decomposition(int n, float matrix[n][n],  float lower[n][n]){
    // Decomposing a matrix into Lower Triangular 
    for (int i = 0; i < n; i++) { 
        for (int j = 0; j <= i; j++) { 
            float sum = 0.0; 
            if (j == i) // summation for diagnols 
            { 
                for (int k = 0; k < j; k++) 
                    sum += powf(lower[j][k], 2); 
                lower[j][j] = sqrtf(matrix[j][j] - sum); 
            } else { 
                for (int k = 0; k < j; k++) 
                    sum += (lower[i][k] * lower[j][k]); 
                lower[i][j] = (matrix[i][j] - sum) / lower[j][j]; 
            } 
        } 
    }
} 

// Weight function for GM Robust cost function
static void GM_UWB(float e, float * GM_e){
    float sigma = 2.0;                        
    float GM_dn = sigma + e*e;
    *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
}

static void GM_state(float e, float * GM_e){
    float sigma = 1.5;                        
    float GM_dn = sigma + e*e;
    *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
}

// robsut update function
void kalmanCoreRobustUpdateWithTDOA(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa)
{
    // Measurement equation:
    // dR = dT + d1 - d0

	float measurement = 0.0f;
    float x = this->S[KC_STATE_X];   
    float y = this->S[KC_STATE_Y];   
    float z = this->S[KC_STATE_Z];

    float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
    float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;

    float dx1 = x - x1;   float  dy1 = y - y1;   float dz1 = z - z1;
    float dx0 = x - x0;   float  dy0 = y - y0;   float dz0 = z - z0;

    float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
    float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
    // if measurements make sense 
    if ((d0 != 0.0f) && (d1 != 0.0f)) {
        float predicted = d1 - d0;
        measurement = tdoa->distanceDiff;
        
        // innovation term based on x_check
        float error_check = measurement - predicted;    // error_check
        // ---------------------- matrix defination ----------------------------- //
        static float P_chol[KC_STATE_DIM][KC_STATE_DIM]; 
        static arm_matrix_instance_f32 Pc_m = {KC_STATE_DIM, KC_STATE_DIM, (float *)P_chol};
        static float Pc_tran[KC_STATE_DIM][KC_STATE_DIM];        
        static arm_matrix_instance_f32 Pc_tran_m = {KC_STATE_DIM, KC_STATE_DIM, (float *)Pc_tran};

        float h[KC_STATE_DIM] = {0};
        arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};    
        // The Kalman gain as a column vector
        static float Kw[KC_STATE_DIM];                           
        static arm_matrix_instance_f32 Kwm = {KC_STATE_DIM, 1, (float *)Kw};

        // float error_x[STATE_DIM]={0};          // error state
        // arm_matrix_instance_f32 error_x_mat = {STATE_DIM, 1, error_x};
        
        static float e_x[KC_STATE_DIM];
        static arm_matrix_instance_f32 e_x_m = {KC_STATE_DIM, 1, e_x};
        
        static float Pc_inv[KC_STATE_DIM][KC_STATE_DIM];
        static arm_matrix_instance_f32 Pc_inv_m = {KC_STATE_DIM, KC_STATE_DIM, (float *)Pc_inv};
        
        // rescale matrix
        static float wx_inv[KC_STATE_DIM][KC_STATE_DIM];
        static arm_matrix_instance_f32 wx_invm = {KC_STATE_DIM, KC_STATE_DIM, (float *)wx_inv};
        // tmp matrix for P_chol inverse
        static float tmp1[KC_STATE_DIM][KC_STATE_DIM];
        static arm_matrix_instance_f32 tmp1m = {KC_STATE_DIM, KC_STATE_DIM, (float *)tmp1};

        static float Pc_w_inv[KC_STATE_DIM][KC_STATE_DIM];
        static arm_matrix_instance_f32 Pc_w_invm = {KC_STATE_DIM, KC_STATE_DIM, (float *)Pc_w_inv};

        static float P_w[KC_STATE_DIM][KC_STATE_DIM];
        static arm_matrix_instance_f32 P_w_m = {KC_STATE_DIM, KC_STATE_DIM, (float *)P_w};

        static float HTd[KC_STATE_DIM];
        static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

        static float PHTd[KC_STATE_DIM];
        static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};
        // ------------------- Some initialization -----------------------//
        // float xpr[STATE_DIM] = {0.0};                   // x prior (error state), set to be zeros 
        // x_err comes from the KF update is the state of error state Kalman filter, set to be zero initially
        static float x_err[KC_STATE_DIM] = {0.0};          
        static arm_matrix_instance_f32 x_errm = {KC_STATE_DIM, 1, x_err};
        static float X_state[KC_STATE_DIM] = {0.0};
        float P_iter[KC_STATE_DIM][KC_STATE_DIM];
        matrixcopy(KC_STATE_DIM, KC_STATE_DIM, P_iter,this->P);   // init P_iter as P_prior
        
        float R_iter = tdoa->stdDev * tdoa->stdDev;               // measurement covariance
        vectorcopy(KC_STATE_DIM, X_state, this->S);               // copy Xpr to X_State and then update in each iterations

        // ---------------------- Start iteration ----------------------- //
        for (int iter = 0; iter < MAX_ITER; iter++){
            // cholesky decomposition for the prior covariance matrix 
            Cholesky_Decomposition(KC_STATE_DIM, P_iter, P_chol);          // P_chol is a lower triangular matrix
            mat_trans(&Pc_m, &Pc_tran_m);

            // decomposition for measurement covariance (scalar case)
            float R_chol = sqrtf(R_iter);       
            // construct H matrix
            // X_state updates in each iteration
            float x_iter = X_state[KC_STATE_X],  y_iter = X_state[KC_STATE_Y], z_iter = X_state[KC_STATE_Z];   

            float dx1 = x_iter - x1;  float dy1 = y_iter - y1;   float dz1 = z_iter - z1;
            float dx0 = x_iter - x0;  float dy0 = y_iter - y0;   float dz0 = z_iter - z0;

            float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
            float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
            
            float predicted_iter = d1 - d0;                         // predicted measurements in each iteration based on X_state
            float error_iter = measurement - predicted_iter;        // innovation term based on X_state

            float e_y = error_iter;
            if ((d0 != 0.0f) && (d1 != 0.0f)){
                // measurement Jacobian changes in each iteration w.r.t linearization point [x_iter, y_iter, z_iter]
                h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);  
                h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0); 
                h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);

                if (fabsf(R_chol - 0.0f) < 0.0001f){
                    e_y = error_iter / 0.0001f;
                }
                else{ 
                    e_y = error_iter / R_chol;
                }
                 
                // keep P_chol
                matrixcopy(KC_STATE_DIM, KC_STATE_DIM, tmp1, P_chol);
                mat_inv(&tmp1m, &Pc_inv_m);                            // Pc_inv_m = inv(Pc_m) = inv(P_chol)
                mat_mult(&Pc_inv_m, &x_errm, &e_x_m);                  // e_x_m = Pc_inv_m.dot(x_errm) 

                // compute w_x, w_y --> weighting matrix
                // Since w_x is diagnal matrix, directly compute the inverse
                for (int state_k = 0; state_k < KC_STATE_DIM; state_k++){
                    GM_state(e_x[state_k], &wx_inv[state_k][state_k]);
                    wx_inv[state_k][state_k] = (float)1.0 / wx_inv[state_k][state_k];
                }

                // rescale covariance matrix P 
                mat_mult(&Pc_m, &wx_invm, &Pc_w_invm);           // Pc_w_invm = P_c.dot(linalg.inv(w_x))
                mat_mult(&Pc_w_invm, &Pc_tran_m, &P_w_m);        // P_w_m = Pc_w_invm.dot(Pc_tran_m) = P_c.dot(linalg.inv(w_x)).dot(P_c.T)

                // rescale R matrix                 
                float w_y=0.0;      float R_w = 0.0f;
                GM_UWB(e_y, &w_y);                              // compute the weighted measurement error: w_y
                if (fabsf(w_y - 0.0f) < 0.0001f){
                    R_w = (R_chol * R_chol) / 0.0001f;
                }
                else{
                    R_w = (R_chol * R_chol) / w_y;
                }
                // ====== INNOVATION COVARIANCE ====== //
                mat_trans(&H, &HTm);
                mat_mult(&P_w_m, &HTm, &PHTm);        // PHTm = P_w.dot(H.T). The P is the updated P_w 

                float HPHR = R_w;                     // HPH' + R.            The R is the updated R_w 
                for (int i=0; i<KC_STATE_DIM; i++) {  // Add the element of HPH' to the above
                    HPHR += h[i]*PHTd[i];             // this only works if the update is scalar (as in this function)
                }
                // ====== MEASUREMENT UPDATE ======
                // Calculate the Kalman gain and perform the state update
                for (int i=0; i<KC_STATE_DIM; i++) {
                    Kw[i] = PHTd[i]/HPHR;                     // rescaled kalman gain = (PH' (HPH' + R )^-1) with the updated P_w and R_w
                    //[Note]: The error_check here is the innovation term based on x_check, which doesn't change during iterations.
                    x_err[i] = Kw[i] * error_check;           // error state for next iteration
                    X_state[i] = this->S[i] + x_err[i];       // convert to nominal state
                }
                // update P_iter matrix and R matrix for next iteration
                matrixcopy(KC_STATE_DIM, KC_STATE_DIM, P_iter, P_w);
                R_iter = R_w;
            }
        }
        // After n iterations, we obtain the rescaled (1) P = P_iter, (2) R = R_iter, (3) Kw.
        // Call the kalman update function with weighted P, weighted K, h, and error_check
        kalmanCoreUpdateWithPKE(this, &H, &Kwm, &P_w_m, error_check);

    } 
}
