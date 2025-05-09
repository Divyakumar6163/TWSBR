# Robofest 4.0 - Two Wheeled Self Balancing Robot

## Overview

This repository contains all the documentation, design files, and implementation details of a **Two-Wheeled Self-Balancing Robot** developed by students of **IIT Bhubaneswar** as part of **Robofest 4.0**.

The robot operates on the principle of an **inverted pendulum**, achieving dynamic stability using sensors, real-time data processing, and a PID control system. It combines **self-balancing** capabilities with **line-following** functionality, and features a **payload-carrying structure**.

## Table of Contents

* [Overview](#overview)
* [Robot Description](#robot-description)
* [Salient Features](#salient-features)
* [Mechanical Design](#mechanical-design)
* [Electronics and Control](#electronics-and-control)
* [Softwares Used](#softwares-used)
* [Block Diagrams](#block-diagrams)
* [Applications](#applications)
* [Team](#team)

## Robot Description

The robot uses two NEMA 23 stepper motors driven by a PID controller implemented on an **Arduino Nano**. An **MPU-6050** sensor detects the tilt and angular velocity. The robot balances itself by calculating real-time error values and adjusting motor outputs accordingly.

## Salient Features

* **Self-Balancing** using MPU-6050 and PID control
* **Payload capacity** of \~1kg
* **Line-following** using IR sensors
* **Adjustable height** using threaded rods
* **Symmetric aluminum frame** for better stability

## Mechanical Design

* 3-layer structure with aluminum plates (Top, Middle, Bottom)
* Fully threaded rods for height adjustment
* Custom motor mount flanges
* Center of gravity optimized for balance

## Electronics and Control

* **Microcontroller**: Arduino Nano
* **Sensors**: MPU-6050, IR Sensors
* **Actuators**: NEMA 23/17 Stepper Motors with TB6600/A4988 drivers
* **Protection**: Diodes, resistors, and capacitors for circuit stability

## Softwares Used

* **Arduino IDE**: PID logic, sensor reading, and motor control
* **SolidWorks**: CAD design of chassis
* **MATLAB**: PID tuning and simulation
* **KiCad & Fritzing**: PCB and circuit design
* **VS Code (Python + OpenCV)**: Code Editor
* **ANSYS**: Stress and vibration analysis

## Block Diagrams

#### Electronics Block Diagram
![Electronics Block Diagram](https://github.com/Divyakumar6163/TWSBR/blob/main/CODE/Circuit_Diagram.jpg?raw=true)

#### Flowchart
![Flowchart](https://github.com/Divyakumar6163/TWSBR/blob/main/CODE/CODE_FLOWCHART.png?raw=true)

## Applications

* Autonomous delivery systems
* Line-following warehouse bots
* Educational robotics
* Assistive mobility devices
* Smart surveillance platforms

## Team

**Team Members:**

* Ayush Gupta
* Divya Kumar
* Jiya Chakraborty
* Rachit Jain
* Vivek Singh

**Mentor:** Dr. Satyanarayan Panigrahi, Associate Professor, School of Mechanical Sciences

**Institute:** Indian Institute of Technology, Bhubaneswar


For full details, refer to the [REPORT.pdf](Reports/3.%20Grand_Finale/REPORT_finale.pdf) in the repository.








