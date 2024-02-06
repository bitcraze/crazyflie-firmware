/** 
 * This robust M-estimation-based Kalman filter was originally implemented in
 * work by the Dynamic Systems Lab (DSL) at the University of Toronto
 * Institute for Aerospace Studies (UTIAS) and the Vector Institute for
 * Artificial Intelligence, Toronto, ON, Canada.
 *
 * It can be cited as:
   @ARTICLE{Zhao2021Learningbased,
    author={Zhao, Wenda and Panerati, Jacopo and Schoellig, Angela P.},
    title={Learning-based Bias Correction for Time Difference of Arrival
           Ultra-wideband Localization of Resource-constrained Mobile Robots},
    journal={IEEE Robotics and Automation Letters},
    year={2021},
    publisher={IEEE}}
 *
 */
#include "mm_distance_robust.h"
#include "test_support.h"

#define MAX_ITER (2) // maximum iteration is set to 2. 
#define UPPER_BOUND (100)
#define LOWER_BOUND (-100)

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

/* Weight function for GM Robust cost function
 * General guidelines for hyperparameter tuning: 
 * For a given measurement error e, decreasing the sigma of the GM weight function will set a
 * smaller weight to this error e. Then, the variance of this measurement will increase, indicating 
 * a large measurement uncertainty. 
 * Intuitively, a small sigma means you trust the measurements more.
*/
static void GM_UWB(float e, float * GM_e){
    float sigma = 1.5;                        
    float GM_dn = sigma + e*e;
    *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
}

static void GM_state(float e, float * GM_e){
    float sigma = 2.0;                       
    float GM_dn = sigma + e*e;
    *GM_e = (sigma * sigma)/(GM_dn * GM_dn);
}

// robsut update function
void kalmanCoreRobustUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d)
{
    float dx = this->S[KC_STATE_X] - d->x;
    float dy = this->S[KC_STATE_Y] - d->y;
    float dz = this->S[KC_STATE_Z] - d->z;
    float measuredDistance = d->distance;

    float predictedDistance = arm_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
    // innovation term based on x_check
    float error_check = measuredDistance - predictedDistance;    // innovation term based on prior state
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
    // ------------------- Initialization -----------------------//
    // x prior (error state), set to be zeros. Not used for error state Kalman filter. Provide here for completeness 
    // float xpr[STATE_DIM] = {0.0};                  

    // x_err comes from the KF update is the state of error state Kalman filter, set to be zero initially
    static float x_err[KC_STATE_DIM] = {0.0};          
    static arm_matrix_instance_f32 x_errm = {KC_STATE_DIM, 1, x_err};
    static float X_state[KC_STATE_DIM] = {0.0};
    float P_iter[KC_STATE_DIM][KC_STATE_DIM];
    memcpy(P_iter, this->P, sizeof(P_iter));

    float R_iter = d->stdDev * d->stdDev;                     // measurement covariance
    memcpy(X_state, this->S, sizeof(X_state));

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
        dx = x_iter - d->x;  dy = y_iter - d->y;   dz = z_iter - d->z;

        float predicted_iter = arm_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
        // innovation term based on x_check
        float error_iter = measuredDistance - predicted_iter; 

        float e_y = error_iter;

        if (predicted_iter != 0.0f) {
            // The measurement is: z = sqrt(dx^2 + dy^2 + dz^2). The derivative dz/dX gives h.
            h[KC_STATE_X] = dx/predicted_iter;
            h[KC_STATE_Y] = dy/predicted_iter;
            h[KC_STATE_Z] = dz/predicted_iter;

        } else {
            // Avoid divide by zero
            h[KC_STATE_X] = 1.0f;
            h[KC_STATE_Y] = 0.0f;
            h[KC_STATE_Z] = 0.0f;
        }
        // check the measurement noise
        if (fabsf(R_chol - 0.0f) < 0.0001f){
            e_y = error_iter / 0.0001f;
        }
        else{ 
            e_y = error_iter / R_chol;
        }
        // Make sure P_chol, lower trangular matrix, is numerically stable              
        for (int col=0; col<KC_STATE_DIM; col++) {
            for (int row=col; row<KC_STATE_DIM; row++) {
                if (isnan(P_chol[row][col]) || P_chol[row][col] > UPPER_BOUND) {
                    P_chol[row][col] = UPPER_BOUND;
                } else if(row!=col && P_chol[row][col] < LOWER_BOUND){
                    P_chol[row][col] = LOWER_BOUND;
                } else if(row==col && P_chol[row][col]<0.0f){
                    P_chol[row][col] = 0.0f;
                } 
            }
        }
        // Matrix inversion is numerically sensitive.
        // Add small values on the diagonal of P_chol to avoid numerical problems.
        float dummy_value = 1e-9f;
        for (int k=0; k<KC_STATE_DIM; k++){
            P_chol[k][k] = P_chol[k][k] + dummy_value;
        }
        // keep P_chol
        memcpy(tmp1, P_chol, sizeof(tmp1));
        mat_inv(&tmp1m, &Pc_inv_m);                            // Pc_inv_m = inv(Pc_m) = inv(P_chol)
        mat_mult(&Pc_inv_m, &x_errm, &e_x_m);                  // e_x_m = Pc_inv_m.dot(x_errm) 

        // compute w_x, w_y --> weighting matrix
        // Since w_x is diagnal matrix, directly compute the inverse
        for (int state_k = 0; state_k < KC_STATE_DIM; state_k++){
            GM_state(e_x[state_k], &wx_inv[state_k][state_k]);
            wx_inv[state_k][state_k] = (float)1.0 / wx_inv[state_k][state_k];
        }

        // rescale covariance matrix P 
        mat_mult(&Pc_m, &wx_invm, &Pc_w_invm);           // Pc_w_invm = P_chol.dot(linalg.inv(w_x))
        mat_mult(&Pc_w_invm, &Pc_tran_m, &P_w_m);        // P_w_m = Pc_w_invm.dot(Pc_tran_m) = P_chol.dot(linalg.inv(w_x)).dot(P_chol.T)

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
        memcpy(P_iter, P_w, sizeof(P_iter));
        R_iter = R_w;
    }


    // After n iterations, we obtain the rescaled (1) P = P_iter, (2) R = R_iter, (3) Kw.
    // Call the kalman update function with weighted P, weighted K, h, and error_check
    kalmanCoreUpdateWithPKE(this, &H, &Kwm, &P_w_m, error_check);

}  