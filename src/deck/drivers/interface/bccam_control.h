/**
 * Camera Deck Control endpoint: the "deck:bcCam" provider of the Control
 * router, on top of the deck's link Control Service.
 *
 * See docs/functional-areas/crtp/crtp_control.md and
 * docs/functional-areas/camera-deck-control.md.
 */
#pragma once

#include "control.h"

/** Path of the Camera Deck endpoint in the Control router. */
#define BCCAM_CONTROL_ENDPOINT_PATH "deck:bcCam"

/**
 * Register the Camera Deck endpoint in the Control router. Called once from
 * the deck init, after syslink2_control_service_init().
 *
 * @return 0, or the errno of controlEndpointRegister().
 */
int bccam_control_init(void);

/** The registered endpoint node. */
ControlEndpoint *bccam_control_endpoint(void);

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_control_test_reset(void);
#endif
