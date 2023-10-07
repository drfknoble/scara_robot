# Selective Compliance Articulated Robot Arm (SCARA) Project

![](https://img.shields.io/badge/CAD-SOLIDWORKS-red)

![](https://img.shields.io/badge/CAD-KiCad-yellow)

![](https://img.shields.io/badge/IDE-VSCode-blue)
![](https://img.shields.io/badge/IDE-Arduino-green)

## Introduction

Welcome to my repository for my Selective Compliance Articulared Robot Arm (SCARA) project.

<!-- <div>
    <img src="./cad/renders/back.png" style="height:240px;">
    <img src="./doc/photos/back.jpg" style="height:240px;">
</div> -->

## Motivation

I started this project because commercial SCARA robots are very expensive and there any low-cost options for me to use in my teaching or research.

## Aims

The aims of this project included:
- Developing a low-cost, SCARA robot that can be used to teach students about mechatronics and robotics.

## Objectives

The objectives of this project included:
- Creating accessible Computer Aided Design (CAD) files.
- Creating accessible Electronics Design Automation (EDA) files.
- Creating accessible source code and software libraries.
- Creating an easy-to-assemble mobile robot that uses a microcontroller, stepper motors, and rotational sensors.

## Repository Structure

The repository is organised as follows:

```console
scara_project/         
    cad/                          
        fabrication/
        parts/
        sub_assemblies/   
    doc/                                   
    eda/
        manipulator_pcb/        
    src/
        esp32_program/
        my_bringup/
        my_scara_1_0_movit_config/
        my_scara_support/
```

All CAD files can be found in the `cad/` sub-directory. I used SolidWorks to create the 3D models of the robot's parts, sub-assemblies, and assembly. Files neeeded to fabricate the robot can be found in the `fabrication/` sub-directory.

All EDA files can be found in the `eda/` sub-directory. I used KiCAD to create the PCB for the robot. Files need to fabricate the PCB can be found in the `manipulator_pcb/` sub-directory.

All source code and software libaries are included in the `src/` sub-directory. I used an ESP32 as the robot's microcontroller and programmed it using PlatformIO's Arduino framework. The ESP32's source code can be found in the `esp32_program/` sub-directory. I also developed a ROS 2 package that makes use of ROS 2 Control and Moveit 2. The package's launch file can be found in the `my_bringup/` sub-directory. 

Documentation is included in the `doc/` sub-directory. I've included a Bill of Materaisl (BOM) file, which includes the compenents used.

## Tools Used

I used the following tools in this project:
- SOLIDWORKS.
- KiCAD.
- Microsoft Visual Studio Code and PlatforIO.
- ROS 2.

Get FreeCAD here: https://www.freecadweb.org/.  
Get KiCAD here: https://www.kicad.org/.  
Get Visual Studio Code here: https://code.visualstudio.com/.  
Get PlatforIO from here: https://platformio.org/. 

## Credit

Dr Frazer K. Noble  
http://drfknoble.com