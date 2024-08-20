# not-a-cub
not-a-cub or lizarde is a small open-source quadruped robot (that was supposed to look like a dog but looks more like a stegosaurus now, also walks like a lizard), that you can 3D print and assemble at home while maintaining low cost (it is also highly customizable).

## Installation
The project consists of three parts:
- Mechanical
- Electronic
- Software

### Mechanical part(s)
All mechanical parts are 3D printable. While making this project, i have utilized Prusa i3 MK3 3D printer with filament type set to PLA and PETG (although you can use just one type of filament) and 0.15mm SPEED printing settings. Furthermore, here is onShape link to export all necessary parts for the print: https://cad.onshape.com/documents/0241e0a68a52b06a72d01161/w/a2f6d68f5b03f40f3b05c63f/e/2f8e02bca12e0e12c7c54226 (note a need to paste this link to a browser, it is not possible to open directly on github as of writing this readme). The parts you will need are:
- 4x `servo_hm_extended`
- 4x `servo_middle_mount`
- `servo_mount_semi` 2x original + 2x mirrored in slicer alongside y axis (please check against your model)
- 4x `servo_hm_tightened`
- 4x `servo_mount_body`
- 2x `body_servo_mount`
- 1x `body`
- 1x (+ depends on customizations) `lidar_servo_mount`
- 1x (+ depends on customizations) `lidar_senzor_mount`

You need to connect multiple parts with each other through servos, please consult next part on electronics to learn more about the servos. All servos will have the standard one-sided mount connected on them. servo_hm_extended will be connected through a servo to servo_middle_mount. servo_middle_mount will be connected through servo_mount_semi and servo_hm_tightened, which will in turn be connected through a servo to servo_mount_body. servo_mount_body will be connected to body_servo_mount which is finally connected to body. To connect lidar, first locate the front of the robot. the front is there, where body_servo_mount L shape has nothing in front of it. The layout of the robot should look like this: rear -- `L___L` -- front. On the front plate, lidar_servo_mount will be connected, servo will be installed and lidar_senzor_mount will be connected on the mount on the servo.

### Electronic part(s)
Following list contains the electronics components needed:
- 12x `SG90` servo
- 1x `ESP32 38-pin` board (prefferably)
- lots of wires (PCB coming soon, though some wires will still be needed)
- possibly a board to solder contacts onto (PCB coming soon)

Following is a list of pins and servos corresponding to each. It is adapted from arduino C code which will be included in the repo. To be able to read it, here is a list of keys:

1st character in the abbreviation
- `F` = front
- `R` = rear

2nd character in the abbreviation
- `L` = left
- `R` = right

3rd character in the abbreviation
- `D` = downer servo
- `M` = middle servo
- `T` = top servo

Here is the list of pins and servos:

```// Front
// Front-Left
#define PIN_FLD 25
#define PIN_FLM 14
#define PIN_FLT 26

// Front-Right
#define PIN_FRD 33
#define PIN_FRM 32
#define PIN_FRT 18

// Rear
// Rear-Left
#define PIN_RLD 16
#define PIN_RLM 04
#define PIN_RLT 27

// Rear-Right
#define PIN_RRD 17
#define PIN_RRM 05
#define PIN_RRT 19
```

### Software part(s)
software is included for download here. Note that you will have to change constants named as `"SERVO ABBREVIATION"_MIS`, e.g. `FLD_MIS`. These constants express how off the servo is from its correct position. This way you can fine-tune the servos to be in the correct position. As an example let's take servo `FLD`. Let's say we put `FLD` servo as a part of calibration into `restLayingPositionAuto`. The leg part of the servo should now be parallel to the ground and perpendicular to the leg part the servo body is attached in. Aka. the `FLD` servo should be the lower part of capital L.
