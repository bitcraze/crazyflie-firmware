/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * control.h - Control router: endpoint registry and request routing.
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * The Control router gathers every bitcraze.control endpoint in the
 * Crazyflie, such as the Camera Deck, and routes requests from originators,
 * such as the CRTP Control bridge, to them.
 *
 * A deck driver registers one ControlEndpoint at init, the same way it
 * registers a ConsoleSource. Its provider, the ControlEndpointOps, carries
 * requests over the deck's link. The router gives each request in flight on
 * an endpoint a unique wire ID in byte 1 of the Control message, and puts the
 * originator's ID back in the response. The router allocates nothing: the
 * endpoint node and each ControlRequest are owned by their callers.
 *
 * Unsolicited NOTIFY messages from an endpoint go the other way: its provider
 * passes them to controlEndpointNotify(), which hands them to every
 * registered ControlListener, such as the CRTP Control bridge.
 */

/** Longest endpoint path, in bytes, so that it fits a CRTP catalog item. */
#define CONTROL_ENDPOINT_PATH_MAX_LENGTH 27u
/** Number of assignable endpoint IDs. ID 0xff is reserved for "all". */
#define CONTROL_ENDPOINT_MAX 255u

typedef struct ControlEndpoint ControlEndpoint;
typedef struct ControlRequest ControlRequest;

// ---- Provider side: implemented by a deck driver ----

typedef struct {
  /**
   * Cheap, non-blocking, callable from any task. False while the link or its
   * Control service is down.
   */
  bool (*isReady)(const ControlEndpoint *endpoint);
  /**
   * Non-blocking. Return 0 when accepted: the provider then calls
   * controlRequestComplete() exactly once, in task context, possibly before
   * returning. Return a positive errno when refused: it never does.
   * message[1] is a wire ID that no other request in flight on this endpoint
   * uses. message stays valid and unchanged until completion.
   */
  int (*submit)(const ControlEndpoint *endpoint, ControlRequest *request,
                const uint8_t *message, size_t length);
} ControlEndpointOps;

/**
 * Caller-provided endpoint node. Set the caller fields before registration.
 * The node and its path must stay valid and unchanged for the firmware
 * lifetime, as for ConsoleSource.
 */
struct ControlEndpoint {
  /** Caller: printable ASCII, ':' between non-empty segments, at most 27 B. */
  const char *path;
  /** Caller: the provider. */
  const ControlEndpointOps *ops;
  /** Caller: for the provider's own use. */
  void *context;
  /** Caller: largest Control message the link accepts. At least 2. */
  uint16_t maxRequest;
  /** Caller: largest Control message the link returns. At least 2. */
  uint16_t maxResponse;

  struct ControlEndpoint *next;   ///< Control-owned list link.
  uint8_t id;                     ///< Control-owned catalog ID.
  uint8_t pathLength;             ///< Control-owned path length.
  uint8_t nextWireId;             ///< Control-owned.
  uint32_t wireIdsInFlight[8];    ///< Control-owned: one bit per wire ID.
};

/**
 * Register an endpoint. Call only from the startup task, before
 * controlEndpointFreeze(), as consoleSourceRegister(). Failure does not
 * change the node or the list.
 *
 * @return 0 on success; EINVAL for a NULL node, a bad path, missing ops or a
 *         limit below 2; EEXIST for a node or path already registered;
 *         ENOSPC when 255 endpoints exist; EBUSY after the freeze.
 */
int controlEndpointRegister(ControlEndpoint *endpoint);

/** Close registration for the firmware lifetime. Called by systemStart(). */
void controlEndpointFreeze(void);

/** True once controlEndpointFreeze() has run. */
bool controlEndpointsAreFrozen(void);

/** Number of registered endpoints, 0 before the freeze. */
uint8_t controlEndpointCount(void);

/** The endpoint with this ID; NULL before the freeze or when id >= count. */
ControlEndpoint *controlEndpointById(uint8_t id);

/** Ask the endpoint's provider whether it can take requests now. */
bool controlEndpointIsReady(const ControlEndpoint *endpoint);

// ---- Originator side ----

/**
 * Runs once per accepted request, in the provider's task, possibly inside
 * controlSubmit(). Must not block.
 * status 0: request->response[0..length) holds the response, byte 1 is the
 * originator's ID.
 * status errno: length is 0, and the message buffer is as it was submitted.
 */
typedef void (*ControlCompleteFn)(ControlRequest *request, int status,
                                  size_t length);

/** One request in flight. Caller-owned: one per request in flight. */
struct ControlRequest {
  uint8_t *response;              ///< Caller: may be the message buffer.
  size_t responseCapacity;        ///< Caller.
  ControlCompleteFn complete;     ///< Caller.
  void *user;                     ///< Caller.

  ControlEndpoint *endpoint;      ///< Control-owned while in flight.
  uint8_t *message;               ///< Control-owned.
  uint8_t originatorId;           ///< Control-owned: byte 1 as submitted.
  uint8_t wireId;                 ///< Control-owned: byte 1 as sent.
};

/**
 * Submit a Control request to an endpoint. Task context only, never from
 * inside a completion. request and message stay valid and untouched until
 * complete runs.
 *
 * @return 0 when accepted: complete runs exactly once. Otherwise a positive
 *         errno and complete never runs: EINVAL for a NULL argument, a
 *         message shorter than 2 bytes or a kind with bit 7 set; EMSGSIZE
 *         above the endpoint's maxRequest; ENOTCONN when the endpoint is not
 *         ready; ENOBUFS when all 256 wire IDs are in flight; or the
 *         provider's errno.
 */
int controlSubmit(ControlEndpoint *endpoint, ControlRequest *request,
                  uint8_t *message, size_t length);

// ---- Called by providers ----

/**
 * Complete an accepted request, exactly once. Task context, possibly inside
 * ops->submit(). response is borrowed for the duration of the call; on
 * failure pass NULL and 0. The router checks the response (EPROTO when it is
 * not a response to this request, EMSGSIZE when it does not fit), copies it,
 * restores the originator's ID and calls the originator's complete.
 */
void controlRequestComplete(ControlRequest *request, int status,
                            const uint8_t *response, size_t length);

/**
 * Pass an unsolicited message from the endpoint, such as a NOTIFY, to the
 * listeners. Task context, never an ISR. message is borrowed for the call.
 * Messages before the freeze, empty ones and ones whose kind is not
 * unsolicited (0xc0..0xff) are dropped and counted.
 */
void controlEndpointNotify(ControlEndpoint *endpoint, const uint8_t *message,
                           size_t length);

// ---- Listeners: firmware code that wants the NOTIFY messages ----

typedef struct ControlListener ControlListener;

/**
 * Caller-provided listener node. Set the caller fields before registration.
 * The node must stay valid and unchanged for the firmware lifetime.
 */
struct ControlListener {
  /**
   * Caller: called for each unsolicited message of a matching endpoint, in
   * the provider's task. message is borrowed for the call: copy what you
   * need. Must not block.
   */
  void (*onNotify)(ControlListener *listener, ControlEndpoint *endpoint,
                   const uint8_t *message, size_t length);
  /** Caller: the endpoint to listen to, or NULL for every endpoint. */
  ControlEndpoint *endpoint;
  /** Caller: for the listener's own use. */
  void *context;

  ControlListener *next;          ///< Control-owned list link.
};

/**
 * Register a listener. Call only from the startup task, before
 * controlEndpointFreeze().
 * @return 0 on success; EINVAL for a NULL node or a missing onNotify; EEXIST
 *         for a node already registered; EBUSY after the freeze.
 */
int controlListenerRegister(ControlListener *listener);

/** Unsolicited messages dropped by controlEndpointNotify(), for diagnostics. */
uint32_t controlNotifyDroppedCount(void);

#ifdef UNIT_TEST_MODE
/** Reset router state between host tests. */
void controlResetForTest(void);
#endif

#endif /* CONTROL_H_ */
