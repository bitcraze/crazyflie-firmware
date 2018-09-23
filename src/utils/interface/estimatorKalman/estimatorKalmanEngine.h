#ifndef estimatorKalmanEngine_h
#define estimatorKalmanEngine_h

#include <stdint.h>
#include "stabilizer_types.h"
#include "estimatorKalmanStorage.h"

/**
 A x,y,z vector with an associated standard deviation per axis
 TODO: acecilia. This may be moved into "stabilizer_types.h"
 */
typedef struct {
  Axis3f value;
  Axis3f stdDev;
} vec3Measurement_t;

/**
 NOTE: This values should not be used inside the implementation of the kalman estimator as static variables, but they should be passed as arguments, when calling its functions
 */
typedef struct {
  /**
   Initial variances, uncertain of position, but know we're stationary and roughly flat. If needed, they should be passed as arguments in the init function call
   */
  struct {
    Axis3f position;
    Axis3f velocity;
    Axis3f angularVelocity;
  } stdDevInitialFlat;
} estimatorKalmanConstants_t;

extern const estimatorKalmanConstants_t estimatorKalmanConstants;

typedef struct {
  /**
   Constants defined and used inside the estimator implementation. Externalized here in case they are needed
   */
  const float maximumAbsolutePosition;        // In meters
  const float maximumAbsoluteVelocity;        // In m/s

  void (*initializeEngine)(uint32_t (*getTickCount)(void), uint32_t tickFrequency); // Need to call this function at least once before using the estimator

  void (*init)(estimatorKalmanStorage_t* storage, const vec3Measurement_t* initialPosition, const vec3Measurement_t* initialVelocity, const vec3Measurement_t* initialAttitudeError);
  void (*update)(estimatorKalmanStorage_t* storage, bool performPrediction);

  // Incorporation of additional data
  bool (*enqueueAcceleration)(estimatorKalmanStorage_t* storage, const Axis3f* acceleration);
  bool (*enqueueAngularVelocity)(estimatorKalmanStorage_t* storage, const Axis3f* angularVelocity);
  bool (*enqueuePosition)(estimatorKalmanStorage_t* storage, const positionMeasurement_t* position);
  bool (*enqueueDistance)(estimatorKalmanStorage_t* storage, const distanceMeasurement_t* distance);
  bool (*enqueueVelocity)(estimatorKalmanStorage_t* storage, const measurement_t* velocity);

  bool (*isPositionStable)(const estimatorKalmanStorage_t* storage, const float maxStdDev);
  bool (*isVelocityStable)(const estimatorKalmanStorage_t* storage, const float maxStdDev);

  void (*getPosition)(const estimatorKalmanStorage_t* storage, point_t* position);
  void (*getState)(const estimatorKalmanStorage_t* storage, state_t* state);
} estimatorKalmanEngine_t;

extern const estimatorKalmanEngine_t estimatorKalmanEngine;

#endif /* estimatorKalmanEngine_h */

