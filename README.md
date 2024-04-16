# not-a-cub
not-a-cub is a small open-source quadruped robot (that was supposed to look like a dog but looks more like a stegosaurus now), that you can 3D print and assemble at home while maintaining low cost (it is also highly customizable).

## installation
the project consists of three parts:
- mechanical
- electronics
- software

### mechanical part
all mechanical parts are 3D printable. while making this project, i have utilized Prusa i3 MK3 3D printer with filament type set to PLA and PETG (although you can use just one type of filament) and 0.15mm SPEED printing settings. furthermore, here is onShape link to export all necessary parts for the print: https://cad.onshape.com/documents/0241e0a68a52b06a72d01161/w/a2f6d68f5b03f40f3b05c63f/e/2f8e02bca12e0e12c7c54226. The parts you will need are:
- 4x servo_hm_extended
- 4x servo_middle_mount
- servo_mount_semi 2x original + 2x mirrored in slicer alongside y axis (please check against your model)
- 4x servo_hm_tightened
- 4x servo_mount_body
- 2x body_servo_mount
- 1x body
- 1x (+ depends on customizations) lidar_servo_mount
- 1x (+ depends on customizations) lidar_senzor_mount

you need to connect multiple parts with each other through servos, please consult next part on electronics to learn more about the servos. all servos will have the standard one-sided mount connected on them. servo_hm_extended will be connected through a servo to servo_middle_mount. servo_middle_mount will be connected through servo_mount_semi and servo_hm_tightened, which will in turn be connected through a servo to servo_mount_body. servo_mount_body will be connected to body_servo_mount which is finally connected to body. to connect lidar, first locate the front of the robot. the front is there, where body_servo_mount L shape has nothinf ahead of it. the layout of the robot should look like this: rear -- L___L -- front. on the front plate, lidar_servo_mount will be connected, servo will be installed and lidar_senzor_mount will be connected on the mount on the servo.

### electronics part
following list contains the electronics components needed:
- 12x SG90 servo
- 1x ESP32 38-pin board (prefferably)
- lots of wires
- possibly a board to solder contacts onto

following is a list of pins and servos corresponding to each. it is adapted from arduino C code which will be included in the repo. to be able to read it, here is a list of keys:

1st place in the text
- F = front
- R = rear

2nd place in the text
- L = left
- R = right

3rd place in the text
- D = downer servo
- M = middle servo
- T = top servo

here is the promised list of pins and servos ;):

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

### software part
software will be included for download here.
