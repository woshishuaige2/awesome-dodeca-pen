# Dodeca Pen with Integrated Force and Motion Sensing

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

## 🛠️Build Step
1. Print all STL files in the ./STL folder using your 3D printer.

2. Solder the force sensor to the custom PCB. Align the top-left corner of the sensor (marked with a tiny circle) to the matching circle on the PCB.

3. Solder the PCB, battery, and development board together using wires, following the schematic below.
✂️ Make sure to cut each wire to the appropriate length so the components fit snugly inside the stylus body.

4. Insert the assembled electronics (force sensor PCB, battery, and microcontroller) into the bottom half of the 3D-printed stylus body.
🕳️ Note: The body includes a hole for a switch to disconnect the battery during development. You can ignore this if not needed.

5. Find a small flat piece of metal or hard plastic, and glue it to the back of the 3D-printed nib base—specifically the part that makes contact with the force sensor.

6. Insert the nib into the 3D-printed nib base. Then fit the nib and compression spring into the front end of the stylus.
🧷 You may need to hold this assembly in place until the stylus is fully closed.

7. Slide the assembled stylus through the larger ring (07_d_ring.stl) until the ring rests at the rear of the stylus.

8. Attach the top half of the stylus, and secure it using the smaller ring (05_ring.stl).

9. Glue a small magnet into the magnet holder holes on both the dodeca ball lid and one half of the dodeca ball.

10. Insert the IMU into the first half of the dodeca ball.

11. Align the second half of the dodeca ball and secure both halves onto the stylus body using the larger ring.

12. Place the lid onto the dodeca ball so the two magnets snap together securely.

13. Print the ArUco markers from ./aruco_images/print.jpg at the correct scale. Carefully glue them onto each face of the dodeca ball, aligning them with the square cutouts.

14. Glue the markers to the stylus using a glue stick (or another adhesive).
🧭 Place the markers in the same order as labeled in the dodeca_stylus_half.stl file.

15. Finally, you'll also need a ChArUco board for camera calibration, which you can print using the pattern from markers/. You can either attach this to a flat board, or leave it on your desk. #TODO
---

## Software Step
Placeholder

## Run Application
Placeholder

## Camera Calibration
Placeholder 
