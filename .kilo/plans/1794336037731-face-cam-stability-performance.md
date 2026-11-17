# Plan: Face Recognition Camera (cam_face) — Stability, Low-Light Auto-Flash, Multi-Core Performance

## Goal
Fix Camera 1 (`src/cam_face/main.cpp`, HW-297 ESP32-CAM, ESP-DL 2-stage detection + MobileFaceNet recognition) so it:
1. Stays online on the hub dashboard (ESP-NOW heartbeats never stall long enough to trip the 15 s offline timeout).
2. Does not lag during user recognition (dedicated core + priority, no task starvation).
3. Auto-turns its flash LED on in low light, off when bright (hysteresis, NVS-persisted, UI toggle).
4. Uses correct dual-core placement for maximum ESP32 performance.

## Scope
- **In**: `src/cam_face/main.cpp`, `src/cam_common/cam_hw.h`, `src/cam_common/cam_hw.cpp`
- **Out**: `cam_plate`, main hub (`src/main.cpp`), hub-side `CAM_ONLINE_TIMEOUT_MS` (keep 15 s; fix heartbeat delivery instead of masking)

## Root causes
1. **Priority inversion on Core 1**: `faceWorkerTask` pinned to Core 1 at priority 1, while `loop()` (loopTask, same core) runs at priority 2 via `vTaskPrioritySet(nullptr, 2)`. loopTask preempts the recognition worker constantly → inference drags (recognition lag); when `loop()` blocks on a slow HTTP client (`handlePreview` → blocking `server.sendContent`), `camNetLoop()` heartbeats stall → dashboard shows OFFLINE after 15 s.
2. **No auto-light**: `flashDuty` is a static NVS value (default 0); the GPIO4 LED never turns on automatically in darkness.
3. **No camera-hang recovery**: `esp_camera_fb_get()` returning NULL is never recovered; the worker spins and the camera stays dead.

## Design decisions
- **D1 Core split (formalize existing)**: Core 0 = Wi-Fi/lwIP + MJPEG stream server (`camStreamTask`, pinned Core 0, prio 1). Core 1 = face recognition worker (`faceWorkerTask`, pinned Core 1, prio 2) + Arduino loop (HTTP, ESP-NOW rx, heartbeats, prio 1). No other tasks on Core 1.
- **D2 Worker priority 2 > loopTask priority 1**: recognition runs at max speed; worker yields to loopTask during blocking frame waits (`esp_camera_fb_get`) and via a short per-iteration `vTaskDelay(1)`, so HTTP/heartbeats keep running while inference hogs CPU between frames.
- **D3 Heartbeat stays in `camNetLoop()` every 5 s**: with D2, loopTask gets CPU each frame wait, so heartbeats stay well inside the 15 s offline timeout.
- **D4 Auto flash**: estimate mean scene luma from the RGB565 frame (downsampled, e.g. every 8th pixel) in `cam_hw`; worker applies flash duty before the next capture. Hysteresis: ON when luma < 0.18, OFF when luma > 0.40, keep state otherwise; auto duty default 150/255. Manual `flashDuty` respected when `flashAuto == false`. Persist `flash_auto` (bool, default true) in NVS ("facecfg"); add UI toggle.
- **D5 Camera-hang recovery**: count consecutive `esp_camera_fb_get()` NULL returns; after ~25 (~1 s at 40 ms poll) call `esp_camera_deinit()` + `esp_camera_init()` to reinitialize OV2640; set `cameraReady` accordingly and expose in `/api/status`.
- **D6 Throughput**: adaptive per-iteration delay — `vTaskDelay(0)`/yield when a face is detected or recognizing (max recognition throughput), 5 ms when idle.

## File changes

### `src/cam_common/cam_hw.h`
Add declaration:
```c
float camHwEstimateMeanLuma(const camera_fb_t *fb);
```

### `src/cam_common/cam_hw.cpp`
Implement `camHwEstimateMeanLuma`: downsampled iteration over the RGB565 buffer; convert RGB565 → luminance `0.299R + 0.587G + 0.114B`; return mean/255 in [0,1]; guard null/empty fb → 0.

### `src/cam_face/main.cpp`
- `FaceConfig`: add `bool flashAuto = true;`. NVS: load `flash_auto` (default true), save it in `saveConfigToNvs`.
- `FaceStatusSnapshot`: add `float sceneLuma` and `uint8_t flashDutyApplied` (telemetry, exposed in `/api/status`).
- `faceWorkerTask`:
  - take mutex, read cfg incl. `flashAuto`;
  - `fb = esp_camera_fb_get()`; NULL → failure counter; at threshold → `cameraReady = false`, `esp_camera_deinit()` + `esp_camera_init()`, reapply hmirror/vflip/flash settings, reset counter; success → `cameraReady = true`, reset counter;
  - compute `sceneLuma = camHwEstimateMeanLuma(fb)`; flash policy with hysteresis → `duty`; if `duty != currentDuty`: `camHwSetFlashLed(duty)`, track `flashDutyApplied`;
  - run detection/recognition/enroll on `fb` (unchanged logic);
  - `camHwUpdatePreviewJpeg(...)`, `esp_camera_fb_return(fb)`;
  - status: sceneLuma, flashDutyApplied, inferenceMs, fps, frameCount;
  - delay: `vTaskDelay(detected or recognizing ? 0 : 5)`.
- `setup()`: `xTaskCreatePinnedToCore(faceWorkerTask, ..., 2, ..., 1)` (prio 2, Core 1); `vTaskPrioritySet(nullptr, 1)` for loopTask; comment documenting Core 0 vs Core 1 split; log stack high-water mark once.
- Web UI: add "Auto Flash (low light)" toggle in settings row, bound to `cfg.flashAuto`, included in `saveSettings()` POST params; `handleConfig()` handles `flash_auto` arg; `/api/status` includes `flashAuto`, `flashDutyApplied`, `sceneLuma`.

## Build & validation
- `pio run -e cam_face` must compile cleanly (Arduino-ESP32 6.9.0, esp32cam board, PSRAM flags from `cam_base`).
- No hardware available in this session → runtime not flashable here. Validation checklist for the user: dashboard "online" badge stays green (heartbeats < 15 s apart); flash LED turns on covering the lens in darkness and off in bright light; `/api/status` `inferenceMs`/`fps` show recognition not stalled by HTTP; no WDT/reboot loops; NVS round-trip of `flash_auto`.

## Open questions (recommended defaults)
1. Auto-flash thresholds (0.18 on / 0.40 off) and duty 150 — confirm against actual IR LED brightness on the user's HW-297; tune if the face is overexposed or underexposed with flash on.
2. Whether to apply the same priority inversion fix to `cam_plate` (same pattern) — out of scope for this request.

## Risks
- Worker prio 2 > loopTask prio 1 can starve Wi-Fi/HTTP if the worker never blocks — mitigated by per-iteration yield and blocking `esp_camera_fb_get` waits.
- `esp_camera_deinit/init` recovery takes ~1 s (brief offline, self-heals; acceptable).
