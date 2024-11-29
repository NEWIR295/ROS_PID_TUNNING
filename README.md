
# ROS_PID_TUNNING

This repository contains the **ROS_PID_TUNNING** project, designed to facilitate the tuning and implementation of a PID controller for motor speed control. The project is organized into two main directories:  

1. **ROS_PID_TUNNING_WS**  
2. **PID_HW_INTERFACE**  

---

## Overview  

This project aims to provide a comprehensive setup for controlling motor speed using PID control through ROS and hardware interfaces. The system allows:  

- **Real-time visualization** of desired and actual speed using `matplotlib`.  
- **Dynamic reconfiguration** of PID parameters (`P`, `I`, `D`) and motor speed during runtime.  
- Compatibility with motor drivers like **L298** and **Cytron**.  

---

## Directories  

### 1. ROS_PID_TUNNING_WS  

This directory contains the ROS workspace for the PID tuning package:  

- **Package**: `pid_tunning`  
- **Functionality**:  
  - Launch a ROS node for PID control and dynamic reconfiguration.  
  - Visualize motor speed in real-time using `matplotlib`.  
  - Adjust PID parameters and motor speed dynamically.  

### 2. PID_HW_INTERFACE  

This directory includes the hardware interface code for motor control:  

- **Micro-controller Node**: Controls motor speed using a motor driver.  
- **Supported Motor Drivers**:  
  - **L298**  
  - **Cytron**  
- **Configuration File**: `ctrl.h`  
  - Ensure to set proper parameters, such as **Pulses Per Revolution (PPR)**, based on the selected encoder.  

---

## Usage  

### Running the ROS Node  

To launch the PID tuning package:  

```bash  
roslaunch pid_tunning pid.launch  
```  

This will:  
- Start the ROS node for PID control.  
- Enable dynamic parameter adjustments.  
- Plot desired and actual speed in real time using `matplotlib`.  

### Adjusting Parameters  

- **PID Parameters (`P`, `I`, `D`)**:  
  Use `dynamic_reconfigure` to adjust these parameters while the system is running.  

- **Motor Speed**:  
  Dynamically adjust the motor speed through `dynamic_reconfigure`.  

- **Hardware Configuration (`ctrl.h`)**:  
  - Set the **PPR** (Pulses Per Revolution) value based on the used encoder.
  - Ensure proper motor wiring and connections.  

---

## Requirements  

- **Software**:  
  - [ROS Noetic (recommended)](http://wiki.ros.org/noetic)  
  - `dynamic_reconfigure` package for ROS.  
  - Python packages:  
    - `matplotlib` for real-time plotting.  

- **Hardware**:  
  - **Microcontroller**: Arduino (Uno or similar).  
  - **Motor Driver**: Cytron or L298.  
  - **DC Motor with Encoder**: Ensure compatibility with your driver and controller.  

---

## Additional Notes  

- **Safety Precautions**:  
  - Verify motor connections before powering up.  
  - Use appropriate power supplies for the motor driver and motor.  

---

## Author  

**Mohamed Newir**  
- [LinkedIn](https://www.linkedin.com/in/mohamed-newir-a8a572182)  

Feel free to reach out for any inquiries or collaborations! 🎉