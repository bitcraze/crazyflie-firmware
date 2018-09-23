#ifndef estimatorKalmanStorage_h
#define estimatorKalmanStorage_h

#include "arm_math.h"
#include "FIFO.h"

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

// The quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, STATE_PX, STATE_PY, STATE_PZ, STATE_D0, STATE_D1, STATE_D2, STATE_DIM
} stateIdx_t;

#define ACCELERATION_QUEUE_LENGTH (10)
#define ANGULAR_VELOCITY_QUEUE_LENGTH (10)
#define POSITION_QUEUE_LENGTH (10)
#define DISTANCE_QUEUE_LENGTH (10)
#define VELOCITY_QUEUE_LENGTH (10)

/**
 A x,y,z vector with one associated standard deviation for the three axis
 TODO: acecilia. This may be moved into "stabilizer_types.h"
 */
typedef struct {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float axis[3];
  };
  float stdDev;
} measurement_t;

/*
 * The struct keeping the internal storage for the kalman estimator. Declare it statically: using it in any other way may cause stack overflow or fill the available dynamic memory
 */
typedef struct {
  float S[STATE_DIM];

  // The queues to add data to the filter
  struct fifo_descriptor accelerationDataQueue;
  Axis3f accelerationDataBuffer[ACCELERATION_QUEUE_LENGTH];

  struct fifo_descriptor angularVelocityDataQueue;
  Axis3f angularVelocityDataBuffer[ANGULAR_VELOCITY_QUEUE_LENGTH];

  struct fifo_descriptor positionDataQueue;
  positionMeasurement_t positionDataBuffer[POSITION_QUEUE_LENGTH];

  struct fifo_descriptor distanceDataQueue;
  distanceMeasurement_t distanceDataBuffer[DISTANCE_QUEUE_LENGTH];

  struct fifo_descriptor velocityDataQueue;
  measurement_t velocityDataBuffer[VELOCITY_QUEUE_LENGTH];


  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  float q[4];

  // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  float R[3][3];

  // The covariance matrix
  float P[STATE_DIM][STATE_DIM];
  arm_matrix_instance_f32 Pm;

  // Internal variables
  bool isInit;
  uint32_t lastUpdate;
} estimatorKalmanStorage_t;

#endif /* estimatorKalmanStorage_h */

