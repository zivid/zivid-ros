# Zivid ROS 2: capture_2d_trigger Wait-Forever Behavior

## Summary

The `capture_2d_trigger` node waits for the `/capture_2d` service to become available, then
continuously triggers it at a configurable rate. By default, it waits forever so a robot can bring
up dependencies in any order.

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `rate_hz` | double | `30.0` | Capture trigger frequency |
| `wait_for_service_timeout_s` | double | `0.0` | Startup timeout. `<= 0` means wait forever |

## Runtime Behavior

### Startup

1. Create a `capture_2d` service client (`std_srvs/srv/Trigger`).
2. Wait for the service to exist using short waits so shutdown remains responsive:
   - If `wait_for_service_timeout_s <= 0`, wait forever.
   - If the timeout is set and exceeded, print one error and exit.
3. Once the service exists, start a periodic timer at `rate_hz`.

### Continuous Operation

- If a request is already in flight, skip the tick.
- If the service is temporarily unavailable, log a throttled warning and skip the tick.
- On failed capture responses, log a throttled warning and continue running.

## Launch File Context

The XML launch file `zivid_samples/launch/zivid_camera.launch` forwards
`settings_2d_file_path` and `settings_2d_yaml` into the `zivid_camera` node. Without this,
`auto_capture_2d:=true` will fail unless a sample sets the parameters elsewhere.

Example (simplified):

```xml
<arg name="settings_2d_file_path" default="" />
<arg name="settings_2d_yaml" default="" />
<node pkg="zivid_camera" exec="zivid_camera" name="zivid_camera">
  <param name="settings_2d_file_path" value="$(var settings_2d_file_path)"/>
  <param name="settings_2d_yaml" value="$(var settings_2d_yaml)"/>
</node>
```
