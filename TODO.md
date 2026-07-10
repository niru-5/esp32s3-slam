# TODO

Ideas and open threads not yet scheduled into active work. See `CLAUDE.md`
for project goals and `docs/architecture.md` for the current `data_capture`
design.

## firmware/data_capture

- **Replace HTTP POST with raw TCP sockets for camera/IMU streaming.**
  Currently `net_client.c` POSTs each JPEG frame / IMU batch via
  `esp_http_client` to `host_server`. Measured per-request latency was
  110-260ms for camera frames, 15-160ms for IMU batches — HTTP's per-request
  overhead (header generation/parsing, no true streaming) is a likely
  contributor. ESP-IDF has a full lwIP stack, so raw BSD sockets
  (`lwip/sockets.h`) are available; a simple length-prefixed frame (4-byte
  length + payload, similar to the existing IMU wire format) over a
  persistent TCP connection could cut that overhead substantially. Would
  require `host_server` to grow a raw TCP listener alongside/instead of its
  HTTP routes.
