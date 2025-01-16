# Zivid Description

## Usage

The URDF or the underlying macro can be included in your own project.

To view the Zivid 2, Zivid 2+, or Zivid 2+R cameras:

```
ros2 launch zivid_description view_zivid_camera.launch model:=ZIVID_2_M70 field_of_view:=true
```

## Available Models

The available Zivid camera models are:

- `ZIVID_2_M70` (default)
- `ZIVID_2_L100`
- `ZIVID_2_PLUS_L110`
- `ZIVID_2_PLUS_M60`
- `ZIVID_2_PLUS_M130`
- `ZIVID_2_PLUS_LR110`
- `ZIVID_2_PLUS_MR130`
- `ZIVID_2_PLUS_MR60`

## Launch Files

### Load Zivid camera

`load_zivid_camera.launch`
: Loads the Zivid camera description.

Arguments:
- `model` (string, default `ZIVID_2_M70`). Choose from the list of [available camera models](#available-models).
- `field_of_view` (boolean, default `true`). Enable to include a mesh of the camera field of view.

### View Zivid camera

`view_zivid_camera.launch`
: Loads the Zivid camera description and visualizes the camera in RViz.

Arguments:
- `model` (string, default `ZIVID_2_M70`). Choose from the list of [available camera models](#available-models).
- `field_of_view` (boolean, default `true`). Enable to include a mesh of the camera field of view.
