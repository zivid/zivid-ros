# Zivid Description

## Usage
The URDF or the underlying macro can be included in your own project.

To view the Zivid One Plus:
```bash
roslaunch zivid_description view_zivid_one_plus_camera.launch type:='ZIVID_ONE_M'
```

The available types are: `ZIVID_ONE_S`, `ZIVID_ONE_M` (default), `ZIVID_ONE_L`

If an incorrect type is specified, the xacro will fail with the following error:
```bash
name 'optical_frame_angle' is not defined 
when evaluating expression '0.5*pi + optical_frame_angle/180*pi' 
```

To view the Zivid Two (Plus):
```
roslaunch zivid_description view_zivid_two_camera.launch type:='ZIVID_TWO_M70'
```

The available types are: `ZIVID_TWO_M70` (default), `ZIVID_TWO_M100`, `ZIVID_TWO_PLUS_L110`, `ZIVID_TWO_PLUS_M60`, `ZIVID_TWO_PLUS_M130`

Note: Currently the types do not actually influence the URDF in any way for the Zivid Two or Zivid Two Plus types.

## Launch files
- `load_zivid_one_plus_camera.launch`: Loads the Zivid One Plus camera    
  Arguments:  
  - `type` (string, options: `ZIVID_ONE_S`, `ZIVID_ONE_M`, `ZIVID_ONE_L`, default: `ZIVID_ONE_M`)
- `view_zivid_one_plus_camera.launch`: Visualizes the Zivid One Plus camera in RViz  
  Arguments:  
  - `type` (string, options: `ZIVID_ONE_S`, `ZIVID_ONE_M`, `ZIVID_ONE_L`, default: `ZIVID_ONE_M`)
- `load_zivid_two_camera.launch`: Loads the Zivid Two or Zivid Two Plus camera   
  Arguments:  
  - `type` (string, options: `ZIVID_TWO_M70`, `ZIVID_TWO_M100`, `ZIVID_TWO_PLUS_L110`, `ZIVID_TWO_PLUS_M60`, `ZIVID_TWO_PLUS_M130`, default: `ZIVID_TWO_M70`)
- `load_zivid_two_camera.launch`: Visualizes the Zivid Two or Zivid Two Plus camera in RViz  
  Arguments:  
  - `type` (string, options: `ZIVID_TWO_M70`, `ZIVID_TWO_M100`, `ZIVID_TWO_PLUS_L110`, `ZIVID_TWO_PLUS_M60`, `ZIVID_TWO_PLUS_M130`, default: `ZIVID_TWO_M70`)
