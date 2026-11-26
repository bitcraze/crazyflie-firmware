---
title: API Service
page_id: crtp_apiservice
---

CRTP port 10 lets a client list the Crazyflie's API Service endpoints and send
`bitcraze.api` requests to them. An endpoint is a target that serves the
API Service protocol, such as the Camera Deck (`deck:bcCam`). Its requests and
responses (`GET`, `SET`, `CMD`, `OK`, `VALUE`, `ERROR`) are defined by the
`bitcraze.api` service contract and pass through the Crazyflie unchanged.
An endpoint can also send unsolicited `NOTIFY` messages when one of its values
changes. A client that turns forwarding on with `SET_NOTIFY` receives them on
the tunnel.

Port 10 requires CRTP protocol version 14.

## Channels

| Ch | Name              | Framing       | Content                                            |
|----|-------------------|---------------|----------------------------------------------------|
| 0  | Catalog           | single packet | `GET_ITEM`, `GET_INFO`                             |
| 1  | Endpoint commands | single packet | `GET_STATUS`, `SET_NOTIFY`, `RESET`                |
| 2  | Tunnel            | fragmented    | Requests up; responses, bridge failures and forwarded `NOTIFY` down |
| 3  | Events            | single packet | `NOTIFY_LOST`                                      |

Channels 0 and 1 follow the [console](crtp_console.md) conventions. A response
starts with the request's command byte, then a status byte: 0 on success or a
[firmware error number](crtp_error_numbers.md). An error response is those two
bytes only. An empty request gets no response. An unknown command returns
`ENOSYS`, and a request too short for its command returns `EINVAL`. Clients
must ignore bytes after the fields they know, so later versions can append
fields.

## Endpoints and catalog

Firmware code, such as a deck driver, registers a `ApiserviceEndpoint` with the
API Service router (`apiservice.h`) at init. `systemStart()` freezes the list, after
deck init. Endpoint IDs start at 0 in registration order and stay the same
until reboot. IDs go from `0x00` to `0xfe`; `0xff` is reserved for "all
endpoints". An endpoint whose link is not up yet is still listed, and reports
not ready until it is up.

Before the freeze, every request on channels 0 and 2, and `GET_STATUS`,
answers `EAGAIN`. Clients should retry shortly.

## Channel 0: catalog

```text
GET_ITEM request:  [0x00, endpoint_id]
GET_ITEM success:  [0x00, 0x00, endpoint_id, path ...]

GET_INFO request:  [0x01]
GET_INFO success:  [0x01, 0x00, count, catalog_crc (u32 LE),
                    max_in_flight (u8), max_message (u16 LE)]
```

- `path` fills the rest of the packet: printable ASCII with `:` between
  segments, at most 27 bytes, not NUL terminated.
- `catalog_crc` is the same CRC-32 as the [console catalog](crtp_console.md),
  over each entry in ID order: one ID byte, then the path bytes.
- `max_in_flight` is the number of bridge slots: the most requests a client may
  have outstanding, across all endpoints.
- `max_message` is the slot size: the largest API Service message the bridge
  carries in either direction, not counting the tunnel's own bytes.

An unknown ID returns `ENOENT`.

## Channel 1: endpoint commands

### 0x00 GET_STATUS

```text
Request: [0x00, endpoint_id]
Success: [0x00, 0x00, endpoint_id, state (u8),
          max_request (u16 LE), max_response (u16 LE)]
```

`state` is 0 when the endpoint is not ready and 1 when it is ready. Value 2
(configuring) is reserved. `max_request` and `max_response` are the endpoint's
own limits on an API Service message. They can be lower than `max_message`: the
Camera Deck carries 85 bytes. More fields may be appended later.

An unknown ID returns `ENOENT`.

### 0x01 SET_NOTIFY

```text
Request: [0x01, endpoint_id | 0xff, enabled]
Success: [0x01, 0x00, endpoint_id | 0xff, enabled]
```

Turns forwarding of the endpoint's `NOTIFY` messages on (`enabled = 1`) or off
(`enabled = 0`). `0xff` applies to every endpoint. Forwarding starts off, is
idempotent, and `RESET` turns it off on every endpoint. Errors: `EAGAIN` before
the catalog is frozen, `ENOENT` for an unknown ID, `EINVAL` for a short request
or `enabled` above 1, and `ENOSYS` when the firmware is built without a
notification buffer (`CRTP_APISERVICE_NOTIFY_BUFFER = 0`). Firmware code that
listens to an endpoint gets its notifications whatever this setting is.

### 0x02 RESET

```text
Request: [0x02]
Success: [0x02, 0x00]
```

A client sends `RESET` once, before its first tunnel request. CRTP has one
client at a time, so it tells the bridge that a new client starts from a clean
state. The bridge:

1. drops the uplink message being reassembled, without answering it, turns
   notification forwarding off on every endpoint, and drops the notifications
   and loss counts not sent yet;
2. marks every request that is submitted, or completed but not yet sent, as
   orphaned: its completion frees the slot and nothing is sent;
3. sends the `RESET` response.

The response is a boundary. Every channel 2 message before it belongs to an
earlier client, and no message for that client follows it. Orphaned slots stay
busy until their endpoint completes them, because requests cannot be
cancelled. A new client can therefore see `ENOBUFS` right after `RESET`.
`RESET` works before the catalog is frozen.

## Channel 2: tunnel

### Fragments

Every channel 2 packet starts with one header byte:

```text
+-----------------------+-----------------+
| L | index (bits 6..0) | message bytes   |
+-----------------------+-----------------+
  u8                      1..29 bytes
```

- A message is sent as fragments 0, 1, 2, … in order. The last one has bit 7
  (L) set, so a one-packet message starts with `0x80`.
- Messages never interleave in one direction. The client finishes one request
  before sending the next. The Crazyflie sends downlink from a single task.
- Index 0 always starts a new message. A receiver that sees any other
  unexpected index drops the partial message.
- The format allows 128 × 29 = 3712 bytes. Messages are limited by
  `max_message`.

### Uplink message (client to Crazyflie)

```text
+-------------+------+------------+--------------------------------+
| endpoint_id | kind | request_id | rest of the API Service request    |
+-------------+------+------------+--------------------------------+
  u8            u8     u8           path, value ...
```

Everything after `endpoint_id` is a `bitcraze.api` request (bit 7 of
`kind` is 0), passed to the endpoint as is. `request_id` is chosen by the
client; the firmware gives the request another ID on its way to the endpoint
and puts the client's ID back in the response. The first fragment must hold at
least `endpoint_id`, `kind` and `request_id`. If it does not, it is dropped
without an answer.

### Downlink message (Crazyflie to client)

```text
+-------------+--------+------------------------------+
| endpoint_id | status | body                         |
+-------------+--------+------------------------------+
  u8            u8       ...
```

- `status = 0`: `body` is the endpoint's response (`OK`, `VALUE` or `ERROR`;
  kinds `0x80..0xbf`), with the client's request ID in its byte 1.
- `status ≠ 0`: the request failed before or outside the endpoint. `body` is
  exactly `[kind, request_id]` from the request.
- `status = 0` with a body kind in `0xc0..0xff`: an unsolicited message from
  the endpoint, forwarded because the client turned forwarding on. A `NOTIFY`
  (`0xc0`) body is `[0xc0, path_len, path, value TLV]`, as in
  `bitcraze.api`. It has no request ID. Clients ignore the other kinds.

An API Service-level failure, such as an unknown path, arrives as status 0 with an
`ERROR` body.

Notifications are best effort. The bridge keeps them in a buffer
(`CRTP_APISERVICE_NOTIFY_BUFFER`) and sends them after the responses that are
ready, in the order the endpoint sent them. When the buffer is full, a
notification is dropped and counted, and the count is reported on channel 3.
After a loss, a client reads again the paths it mirrors.

### Uplink checks

Checked in this order at fragment 0. When a check fails, the Crazyflie answers
with status ≠ 0 and body `[kind, request_id]`, then ignores packets until the
next index 0.

| When    | Situation                                         | Status                         |
|---------|---------------------------------------------------|--------------------------------|
| index 0 | A message is still being reassembled              | `EBADMSG` for that one, then the new one continues |
| index 0 | Fewer than 3 bytes after the header               | none, dropped                  |
| index 0 | Catalog not frozen                                | `EAGAIN`                       |
| index 0 | `endpoint_id` ≥ count                             | `ENOENT`                       |
| index 0 | `kind` bit 7 set                                  | `EINVAL`                       |
| index 0 | Same endpoint and request ID already outstanding  | `EBUSY`                        |
| index 0 | No free slot                                      | `ENOBUFS`                      |
| any     | Index is not the expected one (and not 0)         | `EBADMSG`                      |
| any     | Total exceeds `max_message`                       | `EMSGSIZE`                     |
| last    | Endpoint refuses: not ready, over its `max_request`, no room on its link, … | `ENOTCONN`, `EMSGSIZE`, `ENOBUFS`, … |
| last    | Accepted                                          | later, with the response       |

### Examples

Catalog for these examples: 0 = `cf:stm32`, 1 = `deck:bcCam`.

`GET meta.schema_modules` on `deck:bcCam`, request ID 7:

```text
up   ch2  80 01 01 07 13 "meta.schema_modules"           (24 bytes)
down ch2  80 01 00 81 07 21 01 00
```

`SET wifi.ssid` to a 32-byte string, request ID 8. The API Service message is
`02 08 09 "wifi.ssid" 20 20 <32 bytes>`, 46 bytes:

```text
up   ch2  00 01 02 08 09 "wifi.ssid" 20 20 <first 14 string bytes>  (30 bytes)
up   ch2  81 <last 18 string bytes>                                 (19 bytes)
down ch2  80 01 00 80 08
```

Endpoint not ready:

```text
down ch2  80 01 80 01 07     status 0x80 = ENOTCONN (128), body = GET, id 7
```

The first uplink packet carries 28 API Service bytes, enough for a `GET` with a
path of up to 25 characters. The first downlink packet carries 27, enough for
a `VALUE` with up to 23 data bytes. Every later packet carries 29.

## Channel 3: events

Single packets with no fragment header, from the Crazyflie only. Clients ignore
unknown events and trailing bytes. Event `0x00` is reserved.

### 0x01 NOTIFY_LOST

```text
[0x01, endpoint_id, dropped]
```

At least `dropped` notifications of the endpoint were lost since the last
report (saturating at 255). It is sent when the CRTP queue has room, so it
never blocks responses, and a later loss is reported in a later event.

## Errors

| errno        | No. | Channels | Meaning |
|--------------|-----|----------|---------|
| `EAGAIN`     | 11  | 0, 1, 2  | Catalog not frozen yet. Retry shortly. |
| `ENOENT`     | 2   | 0, 1, 2  | Unknown endpoint ID. |
| `EINVAL`     | 22  | 0, 1, 2  | Malformed command, or a request the endpoint cannot carry. |
| `ENOSYS`     | 88  | 0, 1     | Unknown or reserved command. |
| `EBUSY`      | 16  | 2        | This request ID is already outstanding at this endpoint. |
| `ENOBUFS`    | 105 | 2        | All slots busy, or the endpoint's link has no room. Retry after a response arrives. |
| `EMSGSIZE`   | 122 | 2        | Request larger than `max_message` or the endpoint's `max_request`, or a response that does not fit in the slot. |
| `EBADMSG`    | 77  | 2        | Fragment sequence broken on the way up. |
| `EPROTO`     | 71  | 2        | The endpoint answered with something that is not a response to this request. |
| `ENOTCONN`   | 128 | 2        | Endpoint not ready. |
| `ECONNRESET` | 104 | 2        | The session to the endpoint ended after the request was sent. |
| `ETIMEDOUT`  | 116 | 2        | The endpoint did not answer in time. Camera Deck: 500 ms after the request left its queue. |
| `EIO`        | 5   | 2        | The link refused to send the request. |
| any other    | —   | 2        | Forwarded from the endpoint's provider. |

## Guarantees

- One terminal message on channel 2 for every request whose first fragment
  arrived with a kind and request ID: a response or a status error. Exceptions:
  requests dropped by `RESET`, and packets lost on the radio.
- No deadline in the Crazyflie bridge. A request stays in its slot until its
  endpoint completes it. Each endpoint's link is responsible for its own
  timeouts; the Camera Deck fails a request after 500 ms without an answer, and
  at once when its session is lost.
- No retries in the bridge.
- Order: requests are submitted in arrival order. Responses can come back in
  any order. Notifications from one endpoint arrive in the order it sent them,
  and never ahead of a response that is ready; a `SET` whose setter changes
  other paths is answered before the notifications it raised.
- Notifications are best effort: a dropped one is always followed by a
  `NOTIFY_LOST`, unless a `RESET` comes first.
- Forward compatibility: trailing bytes in channel 0 and 1 responses are
  ignored by clients, unknown commands answer `ENOSYS`, unknown unsolicited
  kinds and unknown channel 3 events are ignored by clients.

## Firmware configuration

| Kconfig                    | Default | Meaning |
|----------------------------|---------|---------|
| `CRTP_APISERVICE`             | y       | Build the bridge. With no endpoint registered it serves an empty catalog. |
| `CRTP_APISERVICE_SLOTS`       | 4       | 1 to 16. Reported as `max_in_flight`. |
| `CRTP_APISERVICE_MAX_MESSAGE` | 256     | 64 to 1024. Reported as `max_message`. |
| `CRTP_APISERVICE_NOTIFY_BUFFER` | 512   | 0 to 4096 bytes for notifications waiting to be sent, 3 bytes per message plus its length. 0 turns forwarding off. |
