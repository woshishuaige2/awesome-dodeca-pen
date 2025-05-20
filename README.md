# Smart Pen Stylus with Integrated Force and Motion Sensing

This project aims to build a smart pen stylus by integrating a force sensor and IMU with a custom PCB, powered by a battery and enclosed in a 3D-printed pen body. It draws inspiration from the **DPOINT** GitHub repository and the **DodecaPen** research paper.

---

## 🔧 Hardware Overview

- ✅ **Force sensor** integrated into a custom PCB
- ✅ **PCB board** connected to Xiao nRF52840 Sense (IMU-enabled microcontroller)
- ✅ **Battery** connected to power the system
- ✅ **3D printed stylus body** assembled with all components

---

## 💻 Software Components

### 1. **IMU Sensor Code**
- Based on the [DPOINT GitHub repo](https://github.com/DPOINT-repo)  
- Reads 6-DoF/9-DoF motion data from the Xiao nRF’s built-in IMU

### 2. **Computer Vision Code**
- Based on techniques from the [DodecaPen paper](https://example.com)  
- Enables spatial pose estimation using external cameras (planned)

### 3. **Force Sensor Code**
- Custom code to read analog/digital input from the force sensor (in progress)
- Will be integrated with motion data for stylus pressure sensing

---

## 🗺️ Project Progress

### ✅ Completed
- [x] PCB design and force sensor integration
- [x] Xiao nRF connected to PCB
- [x] Battery connected and power verified
- [x] All components enclosed in 3D printed stylus shell
- [ ] Basic IMU code integration

### 🔄 In Progress
- [ ] Develop and test force sensor code
- [ ] Sync force + IMU readings for fused data
- [ ] Explore external CV-based tracking (DodecaPen-style)

---

## 📁 Folder Structure (tentative)

