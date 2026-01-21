# Zivid ROS 2: capture_2d_trigger Fail-Fast Behavior

## Summary

The `capture_2d_trigger` node can trigger the `/capture_2d` service continuously at a fixed rate.
When the Zivid camera is misconfigured (for example, `settings_2d_file_path` is not set on
`/zivid_camera`), this can result in:

- Continuous terminal spam
- Repeated failed service calls
- No clear single-point failure signal

This document proposes and documents a fail-fast, one-shot behavior for `capture_2d_trigger` that
improves usability and debuggability.

## Problem

When launched with:

```bash
ros2 launch zivid_samples zivid_camera.launch auto_capture_2d:=true
```

and the `zivid_camera` node does not have either `settings_2d_file_path` or `settings_2d_yaml` set,
the following happens:

- `capture_2d_trigger` calls `/capture_2d` repeatedly (default 30 Hz)
- The camera node throws the same error each time
- The terminal and logs are spammed with repeated warnings
- The root cause is obscured by noise

## Desired Behavior (Fail Fast)

The recommended default behavior is:

- Wait for `/capture_2d` service to become available (with timeout).
- Call `/capture_2d` once.
- If the call fails:
  - Print one clear error message describing the likely misconfiguration.
  - Exit the node (or shut down cleanly).

No retries, no continuous spam.

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `oneshot` | bool | `true` | If true, trigger capture once and exit |
| `stop_on_failure` | bool | `true` | Exit after first failure |
| `wait_for_service_timeout_s` | double | `5.0` | Max time to wait for `/capture_2d` |
| `rate_hz` | double | `30.0` | Only used if `oneshot == false` |

Default behavior is fail-fast (`oneshot=true`).

## Expected Error Message (Example)

On failure, print once:

```
ERROR: capture_2d failed.
Reason: Zivid camera is not configured for 2D capture.
Fix: Set 'settings_2d_file_path' or 'settings_2d_yaml' on the /zivid_camera node.
```

Then exit.

## Launch File Context

The XML launch file `zivid_samples/launch/zivid_camera.launch` forwards
`settings_2d_file_path` and `settings_2d_yaml` into the `zivid_camera` node. Without this,
`auto_capture_2d:=true` will always fail unless a sample sets the parameters elsewhere.

Example (simplified):

```xml
<arg name="settings_2d_file_path" default="" />
<arg name="settings_2d_yaml" default="" />
<node pkg="zivid_camera" exec="zivid_camera" name="zivid_camera">
  <param name="settings_2d_file_path" value="$(var settings_2d_file_path)"/>
  <param name="settings_2d_yaml" value="$(var settings_2d_yaml)"/>
</node>
```

## Rationale

Fail-fast behavior is preferred because it:

- Eliminates terminal spam
- Makes misconfiguration obvious
- Avoids unnecessary service calls
- Matches how most ROS “trigger” utilities are expected to behave

Retry logic is explicitly opt-in rather than the default.
