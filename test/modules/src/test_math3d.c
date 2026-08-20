// File under test math3d.h
#include "math3d.h"

#include "unity.h"

static void assertQuatEqual(struct quat expected, struct quat actual) {
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected.x, actual.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected.y, actual.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected.z, actual.z);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected.w, actual.w);
}

void testThatQqmulReturnsTheHamiltonProductInArgumentOrder() {
  // Fixture: 90 deg about x, then 90 deg about y. Chosen because these do not
  // commute, so a reversed implementation cannot coincidentally pass.
  struct quat q = qaxisangle(mkvec(1, 0, 0), M_PI_2_F);
  struct quat p = qaxisangle(mkvec(0, 1, 0), M_PI_2_F);

  // Hamilton q*p, computed by hand
  struct quat expected = mkquat(0.5f, 0.5f, 0.5f, 0.5f);

  // Test
  struct quat actual = qqmul(q, p);

  // Assert: the result matches q*p, and the reversed order does not, so a
  // reversed implementation cannot pass
  assertQuatEqual(expected, actual);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, qqmul(p, q).z);
}

void testThatQqmulComposesRotationsRightToLeft() {
  // Fixture: qvrot(qqmul(q, p), v) must equal qvrot(q, qvrot(p, v)),
  // i.e. p is applied first.
  struct quat q = qaxisangle(mkvec(1, 0, 0), M_PI_2_F);
  struct quat p = qaxisangle(mkvec(0, 1, 0), M_PI_2_F);
  struct vec v = mkvec(0, 0, 1);

  // Test
  struct vec composed = qvrot(qqmul(q, p), v);
  struct vec sequential = qvrot(q, qvrot(p, v));

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sequential.x, composed.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sequential.y, composed.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sequential.z, composed.z);
}

void testThatQqmulWithInverseYieldsIdentity() {
  // Fixture
  struct quat q = qaxisangle(vnormalize(mkvec(1, -2, 3)), 0.7f);

  // Test
  struct quat actual = qqmul(q, qinv(q));

  // Assert
  assertQuatEqual(qeye(), actual);
}

void testThatQqmulAppliesBodyFramePerturbationOnTheRight() {
  // Fixture: with q mapping body to world, a body-frame perturbation composes
  // on the right, so it is recovered as q^-1 * qComposed. qComposed is written
  // out by hand rather than composed with qqmul(), so that building the
  // fixture and unwinding it cannot cancel a reversed order.
  struct quat q = qaxisangle(mkvec(0, 0, 1), M_PI_2_F);
  struct quat dq = qaxisangle(mkvec(1, 0, 0), M_PI_2_F);
  struct quat qComposed = mkquat(0.5f, 0.5f, 0.5f, 0.5f); // q * dq

  // Test
  struct quat recovered = qqmul(qinv(q), qComposed);

  // Assert: we recover the body-frame perturbation, not a yaw-rotated version
  assertQuatEqual(dq, recovered);
}
