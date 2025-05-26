# Smart Pen Stylus with Integrated Force and Motion Sensing

A smart stylus system that combines force sensing, inertial motion tracking, and computer vision for high-precision pen input. This project integrates a custom force-sensing PCB with a Seeed Studio Xiao nRF52840 Sense microcontroller, assembled into a 3D-printed pen body with computer vision tracking via ArUco markers.

---
## Clone Repository
Before continuing, clone the repository:
bash git clone https://github.com/woshishuaige2/awesome-dodeca-
pen.git cd awesome-dodeca-pen

## 🔧 Tools and Equipment

| Component               | Details                                     |
| ----------------------- | ------------------------------------------- |
| Development board       | Seeed Studio XIAO nRF52840 Sense            |
| Force sensor            | Alps Alpine HSFPAR003A                      |
| Custom PCB              | Use provided Gerber files (1.6mm thickness) |
| Printer filament        | PLA (other hard plastics also acceptable)   |
| Spring                  | 6.5 x 10 mm compression spring              |
| Battery                 | 10440 lithium-ion battery                   |
| Wire                    | 22–24 AWG                                   |
| Stylus nib              | Wacom replacement nib or PLA substitute     |
| 3D printer              | Any (we used Bambu X1)                      |
| Hot air rework station  | For soldering force sensor                  |
| Soldering iron          | For general electronics                     |
| Inkjet or laser printer | For printing ArUco/ChArUco markers          |
| Smalll circular  magnet | For securing the dodeca lid                 | #TODO

---

## Microcontroller Code  #TODO
To upload the code to the development board:

Follow https://wiki.seeedstudio.com/XIAO_BLE/#software-setup to download the necessary software.
Open dodeca-pen-arduino in the Arduino IDE. 
Connect the board to your computer using a USB-C cable.
Click Tools > Board > Seeed nRF52 Boards > Seeed XIAO nRF52840 Sense.
Upload the code.

### Build Step
1. Print all three STL files from print/export/ on your 3D printer.
2. Solder the force sensor to the custom PCB. Make sure to align the top-left of the force sensor (marked with a very small circle) with the circle on the PCB.
3. Solder the force sensor PCB, battery, and development board together using wires, following the schematic below. Make sure to cut wires the right length to fit the stylus body.
4. Place the force sensor PCB, battery, and microcontroller into the bottom half of the 3D printed stylus. The stylus body also has a hole for a switch to disconnect the battery during development, but you can ignore this.
5. Find a small, flat piece of metal or hard plastic, and glue it to the back of the 3D printed nib base where it contacts the force sensor.
6. Insert the nib into the 3D printed nib base, then fit it into the front of the stylus using the spring. You may need to hold it in place until the stylus is fully assembled.
7. Push the half-assembled pen through the 3D printed bigger ring (for the dodeca ball), until the ring is at the rear side of the stylus. 
8. Add the top half of the stylus, and secure it in place using the 3D printed smaller ring.
9. Glue small magnet onto the magnet holder hole on both the dodeca ball lid and one of the dodeca ball half.
10. Carefully insert the IMU into the first half of the dodeca ball
10. Align the second haLf of the dodeca ball, and secure them onto the stylus body using the bigger ring.
11. Place the lid onto the dodeca ball until two magnets fit together.
12. Print out the ArUco markers from xxxxx.pdf at 100% scale. This has two copies of each marker (and some spares), but you only need one. #TODO: update the pdf file name.
13. Cut carefully along each of the lines around the markers. The white borders are important, so don't cut them off.
14. Glue the markers to the stylus using a glue stick (or your glue of choice). Place them according to the diagram.
15. Finally, you'll also need a ChArUco board for camera calibration, which you can print using the pattern from markers/. You can either attach this to a flat board, or leave it on your desk.
---

### Software Step
Placeholder

### Run Application
Placeholder

### Camera Calibration
Placeholder