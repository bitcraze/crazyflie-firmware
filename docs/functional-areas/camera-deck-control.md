---
title: Camera Deck control
page_id: camera_deck_control
---

Design of the Control Service on the Crazyflie side of the UART link to the
camera deck.

The wire format is defined by the Control Service protocol in
camera-deck-firmware (`docs/protocol/common/control-service.md`). This page
summarizes the layout of the frames in scope, see [Frame layout](#frame-layout). Where they
differ, the protocol document is the authority.

## Scope

In scope, matching `bitcraze.control` version 0.0 on the deck:

- the requests `GET` and `SET`,
- the responses `OK`, `VALUE` and `ERROR`,
- the value types `bool`, `u8`, `i8`, `u16`, `i16`, `u32` and `i32`,
- one request outstanding at a time,
- recognizing unsolicited frames from the deck, such as `NOTIFY`, and
  dropping them, see [Frame classes](#frame-classes).

Out of scope: `CMD`, handling `NOTIFY`, more than one request in flight,
path handles, listing paths and chunking. The probe is also out of scope.
The runtime runs it before the Control Service takes over, see
[Ready](#ready).

## Terms

The terms build on Frame-Slot Credit in the Common Link. Credit is per
service and per direction. Here they are used for the Control service, seen
from the Crazyflie.

- **Frame**: one Control message, the payload of one UART packet on the
  Control service. The runtime removes the UART packet envelope, so a frame
  starts with `kind`. A frame has no length field of its own. Its length is
  the UART packet's `payload_length`, at most the Link MTU. See "Frame
  layout".
- **Transmit credit**: Frame-Slot Credit the deck has given the Crazyflie.
  One credit allows the Crazyflie to send one Control frame.
- **Receive credit**: Frame-Slot Credit the Crazyflie has given the deck.
  One credit allows the deck to send one Control frame.
- **Receive slot**: local space for one received Control frame. Receive
  credit is never more than the number of free receive slots.
- **Grant receive credit**: the Crazyflie makes a free receive slot
  available to the deck. Freeing the slot and telling the deck always happen
  together.
- **Bound**: the Control service is bound when the catalog advertises a
  compatible `bitcraze.control` and the Crazyflie has taken its handle. This
  is Service Binding in the Common Link.
- **Ready**: the Control service is bound and the probe has been answered
  with `VALUE`. User requests are only transmitted while ready.
- **Deck request ID**: the `request_id` on the wire between the Crazyflie
  and the deck. Owned by the Control Service, which assigns it. Users never
  see it.
- **User request ID**: the request ID between a user and the Crazyflie.
  Owned by the user, which assigns it. The Control Service does not read it
  and returns it with the result. For example, a user submits user request
  ID 2, the Control Service sends it to the deck as deck request ID 8, the
  deck answers 8, and the user gets `on_complete` for user request ID 2.
- **Probe**: the `GET 0 meta.schema_modules` request the runtime sends on
  the Control service before ready. The Control Service only observes the
  result.
- **Lifecycle**: the states of something that is created and ends, and goes
  through them once, such as a request from `bccam_control_service_submit`
  to `on_complete`.
- **State machine**: the states of something that lives as long as the
  system and moves between them repeatedly, such as
  `bccam_control_service`.
- **Unit**: one `.c` and `.h` pair with its own interface and its own host
  test. The Control Service layer consists of units.

### Frame layout

Every frame in scope starts with the same two bytes, followed by a body that
depends on `kind`:

| Byte | Field | Content |
|---|---|---|
| 0 | `kind` | `GET` 0x01, `SET` 0x02, `OK` 0x80, `VALUE` 0x81, `ERROR` 0x82. Bit 7 is set in frames from the deck. |
| 1 | `request_id` | The deck request ID. Set by the Crazyflie in the request, copied by the deck into the answer. |
| 2.. | body | Depends on `kind`, see below. |

Bodies:

| `kind` | Body |
|---|---|
| `GET` | `path_len:u8`, `path` |
| `SET` | `path_len:u8`, `path`, `value` |
| `OK` | empty |
| `VALUE` | `value` |
| `ERROR` | `error_code:u8`, `detail_len:u8`, `detail` |

- `value` is `type:u8`, `len:u8`, `data[len]`. Multi-byte integers are
  little-endian.
- `path` is ASCII, `path_len` bytes, not NUL-terminated.
- A response shorter than 2 bytes has no `request_id` and is malformed.

### Frame classes

`bccam_control_codec` sorts every received frame into one of three classes
by its `kind`, using the ranges of the message-kind allocation in the
protocol:

| Class | `kind` |
|---|---|
| Response | `OK`, `VALUE` or `ERROR` (in 0x80..0xbf), with a body that matches [Frame layout](#frame-layout)
| Unsolicited | 0xc0..0xff, for example `NOTIFY`. |
| Malformed | Everything else |

## Layers

A layer is a level of the design with its own responsibility. It talks only
to the layer directly above and the layer directly below, through a defined
interface.

| Layer | Responsibility | Scope |
|---|---|---|
| Users, for example camera settings | Decide what to `GET` and `SET` | Out of scope. Only the API they call is defined. |
| Control Service | Turns user requests into Control frames and back. Handles readiness, timeouts and credit. | In scope |
| Runtime and Common Link | Bytes, frames, session, catalog, Service Binding, credit accounting and the probe | Out of scope. Only the interface the Control Service uses is defined. |

This page describes the Control Service layer and its two interfaces: the
user API above and the runtime interface below.

## Units

The Control Service is broken down into five units by responsibility.
`bccam_control_service` is above the other four and coordinates them.

| Unit | Responsibility |
|---|---|
| [`bccam_control_service`](#bccam_control_service) | The unit that holds all public functions of the control service. The single entry point the service task steps. Holds the ready flag. Tells the other units what to do in a fixed order. Receives frames and passes responses on to `bccam_control_request_handling`. |
| [`bccam_control_request_queue`](#bccam_control_request_queue) | Hands submitted requests from user tasks to the service task. |
| [`bccam_control_request_handling`](#bccam_control_request_handling) | Handles one outstanding request. |
| [`bccam_control_codec`](#bccam_control_codec) | Library of functions for encoding, decoding and validation. |
| [`bccam_control_channel`](#bccam_control_channel) | Handles transmit and receive, and its allowance; the only unit aware of the receive slot and credit. The only unit that calls the runtime. |

`bccam_control_service` is stepped by the bcCam UART service task in
`bccam_uart_service`. `bccam_uart_service` is part of the runtime layer. It
also owns the task and the runtime instance, and wires the Control Service
in by calling `bccam_control_service_init` and `bccam_control_service_step`.
This is the one place where the layer below calls the Control Service.
All units run in that task. The exceptions are
`bccam_control_service_submit` and `bccam_control_service_is_ready`, which
users call from their own task, and `bccam_control_request_queue`, the only
unit shared between tasks.

The sections from [Structure](#structure) to [Error handling](#error-handling) cover more
than one unit. Each unit then has its own section.

## Structure

The units, the user API above them and the runtime interface below them.

The diagrams show functions by their name without the unit prefix: for example, `submit`
in `bccam_control_service` is `bccam_control_service_submit`. Functions are represented by
their names only. The signatures and fields are in [User API](#user-api), [Runtime interface](#runtime-interface) and each
unit's section.

```mermaid
classDiagram
    direction TB

    class bccam_control_user_t {
        <<user API>>
        +on_complete
    }

    class bccam_control_service {
        +init
        +submit
        +is_ready
        +step
    }

    class bccam_control_request_queue {
        +init
        +put
        +take
    }

    class bccam_control_request_handling {
        +step
    }

    class bccam_control_codec {
        +encode_get
        +encode_set
        +decode
    }

    class bccam_control_channel {
        +init
        +runtime_ready
        +open
        +reset
        +can_send
        +send
        +receive
    }

    class bccam_uart_runtime {
        <<runtime interface>>
        +control_ready
        +grant_control_rx_credit
        +control_tx_credit
        +send_control
        +take_control_rx
    }

    class ServiceTask {
        <<bccam_uart_service task>>
    }

    bccam_control_user_t ..> bccam_control_service : submit, is_ready
    ServiceTask --> bccam_control_service : init, step
    bccam_control_service --> bccam_control_request_queue : init, put
    bccam_control_service --> bccam_control_channel : init, runtime_ready, open, reset, receive
    bccam_control_service --> bccam_control_codec : decode
    bccam_control_service --> bccam_control_request_handling : step
    bccam_control_request_handling --> bccam_control_request_queue : take
    bccam_control_request_handling --> bccam_control_codec : encode_get, encode_set
    bccam_control_request_handling --> bccam_control_channel : can_send, send
    bccam_control_request_handling --> bccam_control_user_t : on_complete
    bccam_control_channel --> bccam_uart_runtime : control_ready, credit, send, take
```

The data passed through the user API, and the frame and response passed
between the units:

```mermaid
classDiagram
    direction LR

    class bccam_control_frame_t {
        uint8_t data[BCCAM_CONTROL_FRAME_MAX_LEN]
        uint8_t len
    }
    note for bccam_control_frame_t "BCCAM_CONTROL_FRAME_MAX_LEN = 64"

    class bccam_control_response_t {
        uint8_t kind OK, VALUE or ERROR
        uint8_t request_id
        bccam_control_value_t value
        uint8_t error_code
        char detail[BCCAM_CONTROL_DETAIL_MAX_LEN]
        uint8_t detail_len
    }
    note for bccam_control_response_t "BCCAM_CONTROL_DETAIL_MAX_LEN = 60"

    class bccam_control_request_t {
        uint8_t operation GET or SET
        uint8_t user_request_id
        char path[BCCAM_CONTROL_PATH_MAX_LEN]
        uint8_t path_len
        bccam_control_value_t value
        const bccam_control_user_t *user
    }
    note for bccam_control_request_t "BCCAM_CONTROL_PATH_MAX_LEN = 55"

    class bccam_control_user_t {
        on_complete(const bccam_control_request_t *request, const bccam_control_result_t *result) void
    }

    class bccam_control_result_t {
        bccam_control_outcome_t outcome
        bccam_control_value_t value
        uint8_t error_code
    }

    class bccam_control_outcome_t {
        <<enumeration>>
        BCCAM_CONTROL_OUTCOME_OK
        BCCAM_CONTROL_OUTCOME_VALUE
        BCCAM_CONTROL_OUTCOME_ERROR
        BCCAM_CONTROL_OUTCOME_TIMEOUT
        BCCAM_CONTROL_OUTCOME_NOT_READY
        BCCAM_CONTROL_OUTCOME_ABORTED
        BCCAM_CONTROL_OUTCOME_PROTOCOL_ERROR
    }

    class bccam_control_value_t {
        uint8_t type
        uint8_t len
        uint8_t data[BCCAM_CONTROL_VALUE_MAX_LEN]
    }
    note for bccam_control_value_t "BCCAM_CONTROL_VALUE_MAX_LEN = 4"

    class bccam_control_frame_class_t {
        <<enumeration>>
        BCCAM_CONTROL_FRAME_CLASS_RESPONSE
        BCCAM_CONTROL_FRAME_CLASS_UNSOLICITED
        BCCAM_CONTROL_FRAME_CLASS_MALFORMED
    }

    bccam_control_result_t --> bccam_control_outcome_t
    bccam_control_request_t --> bccam_control_value_t
    bccam_control_request_t --> bccam_control_user_t
    bccam_control_result_t --> bccam_control_value_t
    bccam_control_response_t --> bccam_control_value_t
```

`bccam_control_frame_t` holds one frame, including its bytes. Assumption:
the Link Maximum Transmission Unit (MTU) is 64.

- `len` is the number of valid bytes in `data`, from 0 to
  `BCCAM_CONTROL_FRAME_MAX_LEN`. It replaces the UART packet's
  `payload_length`.
- `BCCAM_CONTROL_FRAME_MAX_LEN` is the Link MTU of the UART profile. `len`
  fits in a `uint8_t` because the MTU is at most 255.

The other types hold their data inline, sized so that every request and
response fits in one frame:

- `BCCAM_CONTROL_VALUE_MAX_LEN` is 4, the size of the largest value type in
  scope, `u32` and `i32`. `len` is the number of valid bytes in `data`.
- `BCCAM_CONTROL_PATH_MAX_LEN` is 55: a `SET` frame has 2 header bytes,
  `path_len`, the path and a value of at most 2 + 4 bytes. A request with a
  longer path cannot be built. `path` is ASCII, `path_len` bytes, not
  NUL-terminated.
- `BCCAM_CONTROL_DETAIL_MAX_LEN` is 60: an `ERROR` frame has 2 header bytes,
  `error_code` and `detail_len` before the detail. `detail` is UTF-8 text,
  `detail_len` bytes, not NUL-terminated.
- `bccam_control_response_t.value` is valid only when `kind` is `VALUE`,
  and `error_code` and `detail` only when `kind` is `ERROR`.
- `bccam_control_frame_class_t` is the class of a received frame, see
  [Frame classes](#frame-classes).
- `user` points to a static `bccam_control_user_t` that outlives every
  request. The path and value are copied, so the user can drop its request
  when `bccam_control_service_submit` returns.
- `bccam_control_user_t` holds one function pointer, `on_complete`. There
  is no context pointer: each user has its own `bccam_control_user_t` and
  its own callback, and `on_complete` gets the request, whose `user` tells
  which user it belongs to.
- `user_request_id` is the user request ID. The user sets it in
  `bccam_control_request_t`, and the Control Service does not read it. It
  reaches `on_complete` in the request copy, so `bccam_control_result_t`
  does not repeat it. It is not the deck request ID, which only
  `bccam_control_request_handling` knows.

Which units include which. An arrow from one unit to another means that
the first unit includes the header of the other. All the types above are in
`bccam_control_types.h`, a header with no `.c` file. Standard headers such
as `stdbool.h` and `stdint.h` are left out.

```mermaid
flowchart TB
    user["user, for example camera settings"]
    uart_service["bccam_uart_service"]

    service["bccam_control_service"]
    handling["bccam_control_request_handling"]
    queue["bccam_control_request_queue"]
    codec["bccam_control_codec"]
    channel["bccam_control_channel"]

    types["bccam_control_types"]
    runtime["bccam_uart_runtime"]

    user --> service
    uart_service --> service

    service --> queue
    service --> handling
    service --> codec
    service --> channel

    handling --> queue
    handling --> codec
    handling --> channel

    service --> types
    handling --> types
    queue --> types
    codec --> types
    channel --> types

    service --> runtime
    channel --> runtime
    runtime --> types
```

- `bccam_control_service` and `bccam_control_channel` include
  `bccam_uart_runtime` for `bccam_uart_runtime_t`, which their init
  functions take.
- `bccam_uart_runtime` includes `bccam_control_types` for
  `bccam_control_frame_t`, which `bccam_uart_runtime_send_control` and
  `bccam_uart_runtime_take_control_rx` take, see
  [Runtime interface](#runtime-interface).
- Users and `bccam_uart_service` include only `bccam_control_service`.
- There are no include cycles. `bccam_control_types` includes no other
  project header.

### User API

Users see only the user API: `bccam_control_service_submit` and
`bccam_control_service_is_ready`, see [bccam_control_service](#bccam_control_service), and their
own `on_complete` in their `bccam_control_user_t`.

| Function | What it does |
|---|---|
| `on_complete(const bccam_control_request_t *request, const bccam_control_result_t *result) void` | Tells the user that a request has completed, with its result. |

- `on_complete` is called in the service task. It must not block.
- Every submitted request gets exactly one `on_complete`.
- `on_complete` gets the copy of the whole request, including its user
  request ID, for every outcome. This includes the outcomes the Crazyflie
  produces itself: `BCCAM_CONTROL_OUTCOME_NOT_READY`, `_ABORTED`,
  `_TIMEOUT` and `_PROTOCOL_ERROR`. A user that needs only the user request
  ID, such as a future cflib proxy, can ignore the rest. A user that needs
  the operation, path or value, such as the param adapter, does not have
  to keep its own table of pending requests.
  `bccam_control_request_handling` holds the copy while the request is
  outstanding anyway.
- User request IDs are separate per user. Two users can use the same user
  request ID. The Control Service does not check them for uniqueness.
- A user should not reuse a user request ID while a request with it is queued
  or outstanding.

### Runtime interface

The functions of the runtime that the Control Service uses. They belong to
`bccam_uart_runtime`. Only `bccam_control_channel` calls them. The runtime
functions take the runtime instance, `bccam_uart_runtime_t *runtime`, as
their first argument. It is left out here. `bccam_uart_service` owns the
runtime instance and passes it to `bccam_control_service_init`, which
passes it on to `bccam_control_channel_init`. `bccam_control_channel`
stores it.

| Function | What it does |
|---|---|
| `bccam_uart_runtime_control_ready() bool` | Returns true when the Control service is bound and the runtime's probe has been answered with `VALUE`. Turns false when the binding is lost. |
| `bccam_uart_runtime_grant_control_rx_credit() void` | Grants one receive credit to the deck. Does not fail temporarily, see [Credit](#credit). |
| `bccam_uart_runtime_get_control_tx_credit() uint8_t` | Returns the transmit credit. |
| `bccam_uart_runtime_send_control(const bccam_control_frame_t *frame) void` | Sends a frame on the Control service, using one transmit credit. |
| `bccam_uart_runtime_take_control_rx(bccam_control_frame_t *frame) bool` | Copies a received frame out of the receive slot into `frame`. Returns true when a frame was taken. Returns false when the receive slot was empty, and `frame` must not be used. |

## Request lifecycle

The lifecycle of one request, from `bccam_control_service_submit` to
`on_complete`. One request is outstanding at a time. The timeout starts
when the request is taken from the queue, so it also covers waiting until a
frame can be sent.

```mermaid
stateDiagram-v2
    [*] --> Queued : bccam_control_service_submit returns true

    Queued --> Completed : taken while not ready / BCCAM_CONTROL_OUTCOME_NOT_READY
    Queued --> Outstanding : taken while ready / assign request_id, start timer

    state Outstanding {
        [*] --> WaitingToSend
        WaitingToSend --> WaitingResponse : bccam_control_channel can send / encode and send
        WaitingResponse --> [*] : matching OK, VALUE or ERROR
        WaitingResponse --> [*] : mismatched answer
    }

    Outstanding --> Completed : answered / BCCAM_CONTROL_OUTCOME_OK, _VALUE or _ERROR
    Outstanding --> Completed : mismatched answer / BCCAM_CONTROL_OUTCOME_PROTOCOL_ERROR
    Outstanding --> Completed : timer expired / BCCAM_CONTROL_OUTCOME_TIMEOUT
    Outstanding --> Completed : stepped while not ready / BCCAM_CONTROL_OUTCOME_ABORTED

    Completed --> [*] : on_complete called
```

### Sequence for request with an answer

A user submits a request and the deck answers it. Frames between
`bccam_control_channel` and the deck go through the runtime and the link.
This diagram describes the full lifecycle including all control service
units.

```mermaid
sequenceDiagram
    autonumber
    participant U as User<br/>(own task)
    participant CS as bccam_control_service
    participant Q as bccam_control_request_queue
    participant H as bccam_control_request_handling
    participant Ch as bccam_control_channel
    participant D as Camera deck<br/>Control Service

    U->>CS: submit(request), no wait
    CS->>Q: put(request)
    Q-->>CS: true
    CS-->>U: true
    CS->>H: step(true, none)
    H->>Q: take()
    Q-->>H: request
    H->>H: ready, assign request_id, start timer
    CS->>H: step(true, none)
    H->>Ch: can_send(), wait until true
    H->>H: encode GET or SET (bccam_control_codec)
    H->>Ch: send(frame)
    Ch->>D: GET or SET request_id path [value]
    alt OK
        D->>Ch: OK request_id
    else VALUE
        D->>Ch: VALUE request_id value
    else ERROR
        D->>Ch: ERROR request_id code detail
    end
    CS->>Ch: receive(frame)
    Ch-->>CS: frame filled in
    Ch->>D: CREDIT_UPDATE control, 1 (grant receive credit)
    CS->>CS: decode (bccam_control_codec), class response
    CS->>H: step(true, response)
    H->>H: match request_id, stop timer
    H->>U: on_complete(request, OK, VALUE or ERROR)
```

## Error handling

The cases in which a request does not get a matching answer, and what the
user gets.

| Case | Detected by | Handling | User gets |
|---|---|---|---|
| No answer before the timeout | `bccam_control_request_handling`, timer | Completes the request. Receive credit stays granted. | `BCCAM_CONTROL_OUTCOME_TIMEOUT` |
| Late answer to a request that timed out | `bccam_control_request_handling`, no request in WaitingResponse | Counts it as stale and drops it. | Nothing more, the request already got `BCCAM_CONTROL_OUTCOME_TIMEOUT` |
| Malformed frame, in any state | `bccam_control_codec`, class malformed | `bccam_control_service` counts it as malformed and drops it. It never completes a request. If it was the answer, the outstanding request times out. | `BCCAM_CONTROL_OUTCOME_TIMEOUT` if it was the answer, otherwise nothing |
| Unsolicited frame, such as `NOTIFY`, in any state | `bccam_control_codec`, class unsolicited | `bccam_control_service` counts it as unsolicited and drops it. It never completes a request. | Nothing |
| `request_id` mismatch in WaitingResponse | `bccam_control_request_handling` | Counts it as a mismatch, a deck defect, and completes the request. The response is the answer, so no other answer will follow. | `BCCAM_CONTROL_OUTCOME_PROTOCOL_ERROR` |
| Ready lost with a request outstanding: deck reset, link fault or handle change | `bccam_control_service`, `bccam_control_channel_runtime_ready()` is false | `bccam_control_service` resets `bccam_control_channel`. `bccam_control_request_handling`, stepped with ready false, completes the request. | `BCCAM_CONTROL_OUTCOME_ABORTED` |
| Request taken from the queue while not ready | `bccam_control_request_handling`, stepped with ready false | Completes the request without sending it. | `BCCAM_CONTROL_OUTCOME_NOT_READY` |
| Queue full | `bccam_control_request_queue`, `bccam_control_request_queue_put` | `bccam_control_service_submit` returns false. | No `on_complete` |

A late answer can never complete the next request. `bccam_control_channel`
lets the next request be sent only when the deck has given transmit credit
again, and the deck does that only after its answer to the previous request
has been accepted for transmission. The link delivers frames in order, so
the late answer arrives before the next request is sent, while it is in
WaitingToSend. See [Credit](#credit) under [bccam_control_channel](#bccam_control_channel).
This holds also when the answer is malformed and dropped.

### Sequence for ready lost

The deck resets, the link faults or the handle changes, with or without a
request outstanding. `bccam_control_service`, in its step, finds that
`bccam_control_channel_runtime_ready()` is false. It resets
`bccam_control_channel`, goes to not ready and steps
`bccam_control_request_handling` with ready false, in the order in
[bccam_control_service](#bccam_control_service). In WaitingToSend or
WaitingResponse, `bccam_control_request_handling` completes the outstanding
request with `BCCAM_CONTROL_OUTCOME_ABORTED` and goes to Idle. In Idle, a
request taken from the queue completes with
`BCCAM_CONTROL_OUTCOME_NOT_READY`.

```mermaid
sequenceDiagram
    autonumber
    participant U as User<br/>(own task)
    participant CS as bccam_control_service
    participant Q as bccam_control_request_queue
    participant H as bccam_control_request_handling
    participant Ch as bccam_control_channel
    participant R as bccam_uart_runtime
    participant D as Camera deck<br/>Control Service

    CS->>Ch: reset()
    Ch->>Ch: clear receive buffer
    CS->>CS: not ready, do not receive
    CS->>H: step(false, none)
    alt WaitingToSend or WaitingResponse
        H->>H: ready false
        H->>U: on_complete(request, ABORTED)
        H->>H: state Idle
    else Idle
        H->>Q: take()
        alt request queued
            Q-->>H: request
            H->>U: on_complete(request, NOT_READY)
        else queue empty
            Q-->>H: none
        end
    end
```

Assumptions:

- `bccam_uart_runtime_control_ready()` goes false only when the Link
  session ends. Deck reset, link fault and handle change all end it. The
  session reset clears the credit in both directions and drops pending
  units, so an answer the deck still owes from the old session never
  reaches a later request, and the next ready starts from
  [Credit](#credit) again.

## bccam_control_service

The unit above the other four. It is the only unit users see, through
`bccam_control_service_submit` and `bccam_control_service_is_ready`, and the
single entry point the service task steps. `bccam_control_service_submit`
passes the request on to `bccam_control_request_queue` and does not block.
The Control Service is ready when the runtime reports
`bccam_uart_runtime_control_ready()`: the Control service is bound and the
runtime's probe has been answered. `bccam_control_service` reads it through
`bccam_control_channel_runtime_ready()`. User requests are only sent while
it is ready. `bccam_control_service` observes ready and not ready and tells
the other units what to do, in a fixed order on each. The other units do
not act on ready and not ready themselves. `bccam_control_channel` only
reports it. `bccam_control_request_handling`
gets the ready flag as an argument to its step.

- Order on ready: `bccam_control_service` opens `bccam_control_channel`
  with `bccam_control_channel_open`, which grants receive credit, then
  sets the ready flag.
- Order on not ready: `bccam_control_service` resets
  `bccam_control_channel` with `bccam_control_channel_reset`, goes to not
  ready, then steps `bccam_control_request_handling` with ready false,
  which completes the outstanding request with
  `BCCAM_CONTROL_OUTCOME_ABORTED`. Requests taken from the queue after that
  complete with `BCCAM_CONTROL_OUTCOME_NOT_READY`.
- The ready flag is written only in the service task. Users read it with
  `bccam_control_service_is_ready` from their own task.

`bccam_control_service` also receives the frames from the deck, so that
`bccam_control_request_handling` sees only responses:

- It receives only while ready. While not ready, the receive slot belongs
  to the runtime and its probe, and `bccam_control_channel` has been reset.
- It receives at most one frame per step, and before it steps
  `bccam_control_request_handling`.
- It decodes the frame with `bccam_control_codec` and acts on its class,
  see [Frame classes](#frame-classes). A response is passed on to
  `bccam_control_request_handling`. An unsolicited or malformed frame is
  counted and dropped.

### Functions

| Function | What it does |
|---|---|
| `bccam_control_service_submit(const bccam_control_request_t *request) bool` | Passes the request on to `bccam_control_request_queue` with `bccam_control_request_queue_put`, without blocking. Returns false when the queue is full, and the request is not queued. |
| `bccam_control_service_is_ready() bool` | Returns the ready flag. |
| `bccam_control_service_init(bccam_uart_runtime_t *runtime) void` | The public init functions of all control_service units. Initializes the control service unit and subunits. Passes `runtime` on to `bccam_control_channel_init`. |
| `bccam_control_service_step() void` | Does one step, see [One step of bccam_control_service](#one-step-of-bccam_control_service). |

### Data

| Field | Type | What it holds |
|---|---|---|
| `ready` | `bool` | The ready flag. Written only in the service task. |

### State machine

```mermaid
stateDiagram-v2
    [*] --> NotReady

    NotReady --> Ready : ready / open bccam_control_channel
    Ready --> NotReady : not ready / reset bccam_control_channel, step bccam_control_request_handling with ready false (completes the outstanding request with BCCAM_CONTROL_OUTCOME_ABORTED)

    note right of NotReady
        Requests taken from the queue in
        NotReady complete with
        BCCAM_CONTROL_OUTCOME_NOT_READY
        without being sent.
    end note
```

### One step of bccam_control_service

What `bccam_control_service` does in one call to
`bccam_control_service_step()`, from start to return.

```mermaid
flowchart TD
    s0(["bccam_control_service_step()"])
    r0{"bccam_control_channel<br/>runtime_ready()?"}

    r1{"ready flag set?"}
    r2["bccam_control_channel open,<br/>set ready flag"]
    f0{"bccam_control_channel<br/>receive gives a frame?"}
    f1["decode (bccam_control_codec)"]
    f2{"class?"}
    fu["count unsolicited, drop"]
    fm["count malformed, drop"]
    h1["bccam_control_request_handling<br/>step(true, response)"]
    h2["bccam_control_request_handling<br/>step(true, none)"]

    n1{"ready flag set?"}
    n2["bccam_control_channel reset,<br/>clear ready flag"]
    h3["bccam_control_request_handling<br/>step(false, none)"]

    e0(["return"])

    s0 --> r0
    r0 -- "true" --> r1
    r1 -- "no" --> r2 --> f0
    r1 -- "yes" --> f0
    f0 -- "yes" --> f1 --> f2
    f0 -- "no" --> h2
    f2 -- "response" --> h1
    f2 -- "unsolicited" --> fu --> h2
    f2 -- "malformed" --> fm --> h2

    r0 -- "false" --> n1
    n1 -- "yes" --> n2 --> h3
    n1 -- "no" --> h3

    h1 --> e0
    h2 --> e0
    h3 --> e0
```

### Ready

The runtime establishes the link, enumerates the catalog and runs the
probe. The flow starts when `bccam_control_service`, in its step, finds
that `bccam_control_channel_runtime_ready()` is true. It
opens `bccam_control_channel` and sets the ready flag, which users read with
`bccam_control_service_is_ready`. Users can then submit requests as in [Request with an answer](#sequence-for-request-with-an-answer). Frames between
`bccam_control_channel` and the deck go through the runtime and the link.

```mermaid
sequenceDiagram
    autonumber
    participant CS as bccam_control_service
    participant Ch as bccam_control_channel
    participant R as bccam_uart_runtime
    participant D as Camera deck<br/>Control Service

    CS->>Ch: runtime_ready()
    Ch->>R: control_ready()
    R-->>Ch: true
    Ch-->>CS: true
    CS->>Ch: open()
    Ch->>D: CREDIT_UPDATE control, 1 (grant receive credit)
    CS->>CS: ready
```

## bccam_control_request_queue

Hands submitted requests from user tasks to the service task.
`bccam_control_request_queue_put` is called from user tasks, through
`bccam_control_service`. `bccam_control_request_queue_take` is called from
the service task, by `bccam_control_request_handling`.

### Storage

- The requests are stored in a FreeRTOS queue in static memory, the same
  pattern as the request queue in `bccam_uart_service`.
- The queue holds 4 requests.
- Each entry is a copy of the whole `bccam_control_request_t`, including
  its `value`. The user can reuse or drop its request as soon as
  `bccam_control_service_submit` returns.
- `bccam_control_request_queue_take` copies the request out to
  `bccam_control_request_handling`, which keeps it while the request is
  outstanding. The `request` passed to `on_complete` is that copy, not the
  user's original.
- The FreeRTOS queue makes `bccam_control_request_queue_put` and
  `bccam_control_request_queue_take` safe between tasks.
  `bccam_control_request_queue` has no lock of its own.
- The service task is not notified when a request is put. It finds the
  request the next time it polls the queue, within its idle wait.
- The queue is not cleared when ready is lost.

### Functions

| Function | What it does |
|---|---|
| `bccam_control_request_queue_init() void` | Creates the queue. |
| `bccam_control_request_queue_put(const bccam_control_request_t *request) bool` | Copies the request into the queue without blocking. Returns false when the queue is full, and the request is not queued. |
| `bccam_control_request_queue_take(bccam_control_request_t *request) bool` | Copies the oldest request out of the queue without blocking. Returns false when the queue is empty. |

Requests are taken in the order they were put.

## bccam_control_request_handling

The unit `bccam_control_request_handling` is the main responsible unit for
the request lifecycle: taking requests off the queue, sending, matching
the answer and calling the `on_complete` function.

The unit is stepped by `bccam_control_service`, once per service step.

Handles one outstanding request at a time. It sends requests only while
ready. It calls `bccam_control_codec` to encode requests. It does not
receive frames itself: `bccam_control_service` passes it the decoded
responses, and only responses, see [Frame classes](#frame-classes).

- The deck request ID, `request_id`, is assigned here and is never shown
  to users. It is incremented for each request and wraps at 255. In
  WaitingResponse it is a check: a mismatch means a deck defect, not a
  late answer. Starts at 1 (since one request is needed to make the 
  control service ready, part of runtime).

### Functions

| Function | What it does |
|---|---|
| `bccam_control_request_handling_step(bool ready, const bccam_control_response_t *response) void` | Does one step, see [One step](#one-step). `response` is the response received in this step, or NULL when there is none. |

### Data

| Field | Type | What it holds |
|---|---|---|
| `outstanding` | `bccam_control_request_t` | The copy of the outstanding request, taken from `bccam_control_request_queue`. Passed to `on_complete`. |
| `next_request_id` | `uint8_t` | The `request_id` for the next request. Starts at 1 and wraps at 255. |
| `deadline` | Not decided | When the outstanding request times out. |

### State machine

```mermaid
stateDiagram-v2
    [*] --> Idle

    Idle --> Idle : taken while not ready / on_complete BCCAM_CONTROL_OUTCOME_NOT_READY
    Idle --> WaitingToSend : taken while ready / assign request_id, start timer

    WaitingToSend --> WaitingResponse : bccam_control_channel can send / encode and send
    WaitingToSend --> Idle : timer expired / on_complete BCCAM_CONTROL_OUTCOME_TIMEOUT

    WaitingResponse --> Idle : matching answer / stop timer, on_complete BCCAM_CONTROL_OUTCOME_OK, _VALUE or _ERROR
    WaitingResponse --> Idle : mismatched answer / stop timer, on_complete BCCAM_CONTROL_OUTCOME_PROTOCOL_ERROR
    WaitingResponse --> Idle : timer expired / on_complete BCCAM_CONTROL_OUTCOME_TIMEOUT

    WaitingToSend --> Idle : stepped while not ready / on_complete BCCAM_CONTROL_OUTCOME_ABORTED
    WaitingResponse --> Idle : stepped while not ready / on_complete BCCAM_CONTROL_OUTCOME_ABORTED
```

### One step

What `bccam_control_request_handling` does in one call to
`bccam_control_request_handling_step(ready, response)`, from start to
return. Each state has its own swim lane.  In Idle and WaitingToSend there is no request waiting
for an answer, so a response is counted as stale and dropped before ready
is checked. A lane ends either without changing the state
or by setting a new state, which takes effect from the next step.

```mermaid
flowchart TD
    s0(["bccam_control_request_handling_step(ready, response)"])
    st{"state?"}

    subgraph idle ["Idle"]
        i0{"response?"}
        is["count stale, drop"]
        i4{"bccam_control_request_queue<br/>take gives a request?"}
        i5{"ready?"}
        i6["on_complete<br/>BCCAM_CONTROL_OUTCOME_NOT_READY"]
        i7["assign request_id,<br/>start timer"]
        i8(["state WaitingToSend"])
        i9(["nothing"])
        i0 -- "yes" --> is --> i4
        i0 -- "no" --> i4
        i4 -- "no" --> i9
        i4 -- "yes" --> i5
        i5 -- "no" --> i6 --> i9
        i5 -- "yes" --> i7 --> i8
    end

    subgraph send ["WaitingToSend"]
        w0f{"response?"}
        ws["count stale, drop"]
        wr{"ready?"}
        wa["on_complete<br/>BCCAM_CONTROL_OUTCOME_ABORTED"]
        w4{"timer expired?"}
        w5["on_complete<br/>BCCAM_CONTROL_OUTCOME_TIMEOUT"]
        w6{"bccam_control_channel<br/>can send?"}
        w7["encode and send"]
        w8(["state Idle"])
        w9(["state WaitingResponse"])
        w0(["nothing"])
        w0f -- "yes" --> ws --> wr
        w0f -- "no" --> wr
        wr -- "no" --> wa --> w8
        wr -- "yes" --> w4
        w4 -- "yes" --> w5 --> w8
        w4 -- "no" --> w6
        w6 -- "no" --> w0
        w6 -- "yes" --> w7 --> w9
    end

    subgraph resp ["WaitingResponse"]
        pr{"ready?"}
        pn{"response?"}
        ps["count stale, drop"]
        pa["on_complete<br/>BCCAM_CONTROL_OUTCOME_ABORTED"]
        p0{"response?"}
        p4{"request_id<br/>matches?"}
        px["count mismatch,<br/>deck defect"]
        pe["stop timer,<br/>on_complete<br/>BCCAM_CONTROL_OUTCOME_PROTOCOL_ERROR"]
        pc["stop timer, build result,<br/>on_complete<br/>BCCAM_CONTROL_OUTCOME_OK, _VALUE or _ERROR"]
        p6{"timer expired?"}
        p7["on_complete<br/>BCCAM_CONTROL_OUTCOME_TIMEOUT"]
        p8(["state Idle"])
        p9(["nothing"])
        pr -- "no" --> pn
        pn -- "yes" --> ps --> pa
        pn -- "no" --> pa
        pa --> p8
        pr -- "yes" --> p0
        p0 -- "yes" --> p4
        p0 -- "no" --> p6
        p4 -- "no" --> px --> pe --> p8
        p4 -- "yes" --> pc --> p8
        p6 -- "yes" --> p7 --> p8
        p6 -- "no" --> p9
    end

    e0(["return"])

    s0 --> st
    st -- "Idle" --> i0
    st -- "WaitingToSend" --> w0f
    st -- "WaitingResponse" --> pr
    i8 --> e0
    i9 --> e0
    w8 --> e0
    w9 --> e0
    w0 --> e0
    p8 --> e0
    p9 --> e0
```

Assumptions:

- `bccam_control_channel` grants one receive credit at a time.
- `bccam_control_channel` lets a request be sent only after any answer to
  the previous request has been delivered, see [Request lifecycle](#request-lifecycle).
- The deck sends exactly one answer per request.
- Every response from the deck is an answer. Unsolicited frames, such as
  `NOTIFY`, and malformed frames never reach
  `bccam_control_request_handling`, see [Frame classes](#frame-classes).

Consequences and design choices:

- At most one response is handled per step, since `bccam_control_service`
  receives at most one frame per step.
- A request outstanding when ready is lost cannot
  complete with another outcome than `BCCAM_CONTROL_OUTCOME_ABORTED`.
- The response is handled before the timeout check, so an answer that
  arrives in the same step as the deadline completes the request.
- In WaitingResponse, any response is the answer to the outstanding
  request. A `request_id` mismatch therefore completes the request at once
  with `BCCAM_CONTROL_OUTCOME_PROTOCOL_ERROR` instead of waiting for the
  timeout, since no other answer will follow.
- A malformed answer is dropped by `bccam_control_service` and does not
  complete the request. The request completes with
  `BCCAM_CONTROL_OUTCOME_TIMEOUT`.
- At most one request is taken from the queue per step, also when it
  completes with `BCCAM_CONTROL_OUTCOME_NOT_READY`.
- A request moves at most one state per step. A request taken in one step
  is sent at the earliest in the next step, and the next request is taken
  at the earliest in the step after the previous one completed.

## bccam_control_codec

Encodes `GET` and `SET`, sorts received frames into classes, and decodes
`OK`, `VALUE` and `ERROR` and validates their bodies. Pure functions
without state.

### Functions

| Function | What it does |
|---|---|
| `bccam_control_codec_encode_get(uint8_t request_id, const char *path, uint8_t path_len) bccam_control_frame_t` | Builds a `GET` frame as in [Frame layout](#frame-layout). |
| `bccam_control_codec_encode_set(uint8_t request_id, const char *path, uint8_t path_len, const bccam_control_value_t *value) bccam_control_frame_t` | Builds a `SET` frame as in [Frame layout](#frame-layout). |
| `bccam_control_codec_decode(const bccam_control_frame_t *frame, bccam_control_response_t *response) bccam_control_frame_class_t` | Returns the class of the frame, see [Frame classes](#frame-classes). Fills in `response` only when the class is response. Otherwise `response` must not be used. |

A response holds `kind`, `request_id` and the body.

A frame is unsolicited when it is at least 1 byte long and `kind` is in
0xc0..0xff. The rest of the frame is not read, so the class does not depend
on the body.

A frame is malformed when:

- it is shorter than 2 bytes and not unsolicited,
- `kind` is neither `OK`, `VALUE` or `ERROR`, nor in 0xc0..0xff,
- `kind` is `OK`, `VALUE` or `ERROR` and the body does not match
  [Frame layout](#frame-layout), or bytes remain after it.

## bccam_control_channel

The only unit aware of the receive slot and credit, and the only unit that
calls the runtime. It works with raw frames only and does not know the
Control message format.

- On `bccam_control_channel_init`, it stores the runtime instance. All its
  runtime calls use it.
- `bccam_control_channel_runtime_ready` passes on
  `bccam_uart_runtime_control_ready()` without acting on it.

- On `bccam_control_channel_open`, it grants the first receive credit. The
  runtime's probe leaves receive credit at 0 when it is done.
- On `bccam_control_channel_reset`, it clears the receive buffer. The
  credit is cleared by the Link session reset, see
  [Sequence for ready lost](#sequence-for-ready-lost).
- It tells whether a frame can be sent, which hides the transmit credit.
- On receive, it copies the frame out of the receive slot into the caller's
  `bccam_control_frame_t` and grants receive credit at once.
- It guarantees that a frame can be sent only after any answer to the
  previous request has been delivered. `bccam_control_request_handling`
  relies on this to recognize stale answers.

### Functions

| Function | What it does |
|---|---|
| `bccam_control_channel_init(bccam_uart_runtime_t *runtime) void` | Stores `runtime` and clears the receive buffer. |
| `bccam_control_channel_runtime_ready() bool` | Returns `bccam_uart_runtime_control_ready()`. |
| `bccam_control_channel_open() void` | Grants the first receive credit. |
| `bccam_control_channel_reset() void` | Clears the receive buffer. |
| `bccam_control_channel_can_send() bool` | Returns true when a frame can be sent, that is when there is transmit credit. |
| `bccam_control_channel_send(const bccam_control_frame_t *frame) void` | Sends the raw frame. |
| `bccam_control_channel_receive(bccam_control_frame_t *frame) bool` | Copies a received frame out of the receive slot into `frame` and grants receive credit at once. Returns true when a frame was received. Returns false when there was none, and `frame` must not be used. |

### Data

| Field | Type | What it holds |
|---|---|---|
| `runtime` | `bccam_uart_runtime_t *` | The runtime instance, owned by `bccam_uart_service`. Set on `bccam_control_channel_init`. |
| `rx_buffer` | `bccam_control_frame_t` | The receive buffer. Cleared on `bccam_control_channel_reset`. |

### Credit

How the Control service's transmit credit and receive credit change from
link establishment, through the runtime's probe, until
`bccam_control_channel` is opened and both sides have credit again. This is
why `bccam_control_channel` can guarantee that a frame can be sent only
after any answer to the previous request has been delivered, also for the
first request after the probe.

There is no negotiation. Each receiver grants credit for its free receive
slots with `CREDIT_UPDATE`, and each sender uses one credit per frame. The
deck has one Control RX slot and grants credit again only after its response
has been accepted for transmission. The link delivers frames in order, so
transmit credit 1 means that the deck is ready for the next request and that
its answer to the previous one has already arrived.

```mermaid
sequenceDiagram
    autonumber
    participant C as Crazyflie<br/>bccam_uart_runtime and bccam_control_channel
    participant D as Camera deck<br/>Control Service

    Note over C,D: Link established, all credit 0
    Note left of C: transmit 0, receive 0
    Note right of D: transmit 0, receive 0

    D->>C: CREDIT_UPDATE control, 1
    Note left of C: transmit 1
    Note right of D: receive 1, its one RX slot is free

    C->>D: CREDIT_UPDATE control, 1 (runtime grants receive credit for the probe)
    Note left of C: receive 1, receive slot free
    Note right of D: transmit 1

    C->>D: GET 0 meta.schema_modules (runtime probe)
    Note left of C: transmit 0, cannot send
    Note right of D: receive 0, RX slot holds the request

    D->>C: VALUE 0 table
    Note right of D: transmit 0
    Note left of C: receive 0, runtime takes the answer, probe done, ready

    D->>C: CREDIT_UPDATE control, 1
    Note right of D: receive 1, RX slot free after the response was accepted for TX
    Note left of C: transmit 1, can send

    C->>D: CREDIT_UPDATE control, 1 (bccam_control_channel grants receive credit on open)
    Note left of C: receive 1, receive slot free
    Note right of D: transmit 1
```

- Steps 1 and 2 can come in either order, and so can steps 5 and 6.
- The runtime does not grant receive credit again after the probe answer.
  `bccam_control_channel` grants it on `bccam_control_channel_open`, after
  `bccam_control_service` has seen ready.
- After a timeout, the receive credit stays granted. Either the late answer
  uses it, and receive credit is granted again when it is received, or the
  next request finds it already granted.
- `bccam_uart_runtime_grant_control_rx_credit` does not fail temporarily.
  When the grant cannot be sent at once, the runtime keeps it pending and
  sends it on a later step. A request sent while a grant is pending waits
  for its answer until the grant is sent, within its timeout.
- Every received frame uses one receive credit. `bccam_control_channel` grants receive
  credit at once after a frame has been taken. So an unsolicited frame, such as `NOTIFY`, sent by
  the deck before an answer delays the answer by one round trip but never
  blocks it. Many unsolicited frames in a row can delay an answer past its
  timeout.
- The order in which the deck sends answers and unsolicited frames is the
  deck's policy, see the protocol. This design does not depend on it.
- Unsolicited frames do not affect the guarantee that a frame can be sent
  only after any answer to the previous request has been delivered. That
  guarantee rests on the deck's Control RX slot and the transmit credit,
  and unsolicited frames use neither.

## Open questions

Does this design support the full chain control service, from deck to cflib? Because it is created with the temporary param path in mind.
  One step of control service - what does ready flag add?

Does the design guarantuee that we cannot get stuck in off-by-one loops? Where the camera deck always sends a response to a request that came one request ago?
