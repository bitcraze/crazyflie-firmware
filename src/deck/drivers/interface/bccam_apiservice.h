/**
 * Camera Deck API Service endpoint: the "deck:bcCam" provider of the API Service
 * router, on top of the deck's link API Service.
 *
 * See docs/functional-areas/crtp/crtp_apiservice.md and
 * docs/functional-areas/camera-deck-apiservice.md.
 */
#pragma once

#include "apiservice.h"

/** Path of the Camera Deck endpoint in the API Service router. */
#define BCCAM_APISERVICE_ENDPOINT_PATH "deck:bcCam"

/**
 * Register the Camera Deck endpoint in the API Service router. Called once from
 * the deck init, after syslink2_apiservice_init().
 *
 * @return 0, or the errno of apiserviceEndpointRegister().
 */
int bccam_apiservice_init(void);

/** The registered endpoint node. */
ApiserviceEndpoint *bccam_apiservice_endpoint(void);

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_apiservice_test_reset(void);
#endif
