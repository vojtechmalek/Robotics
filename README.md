# Robotics Course 34753 – Robot Arm Control Repository

Welcome to the official repository for **Robotics Course 34753**.  
This repository contains resources, code examples, and CAD documents for controlling a **4-DOF robot arm** using **Python**, including the necessary **Dynamixel** library build.

---

## 📁 Repository Structure

├── DynamixelSDK/\
│\
├── Robot CAD Files/\
│\
└── README.md\

---
## ⚙️ Dynamixel Control Library

The **DynamixelSDK/** folder contains the pre-built Dynamixel SDK library required for communicating with the robot arm’s actuators. 
Additionally, it containts example control codes for basic robot control. 

For installment of library please watch **Installation and Library Setup (Python)** below.

---
## 📐 CAD Files (Robot Arm Dimensions)

The **Robot CAD Files/** folder contains 3D models for the 4DOF robot arm. These files can be usefull for calculations and end-effector designs used in the assignment. 

---

## 🎥 Video Tutorials for Robot Setup (Python)

### Step 1 - Install Git and uv
[Link for Git installation](https://git-scm.com/install/) 

## UV installation: 
Windows: 

```
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Mac/linux: 
```
curl -LsSf https://astral.sh/uv/install.sh | sh
```

If you already have Git and uv installed procede to Step 2. 

<br>

### Step 2 - Download Dynamixel Wizard
#### Please install Dynamixel Wizard through the following link:
[Download Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)


<br>


### Step 3 - Download and install Dynamixel library
#### ▶️ Installation and Library Setup (Python)
<a href="https://www.youtube.com/watch?v=O5IRRObA7h0" target="_blank">
 <img src="https://img.youtube.com/vi/O5IRRObA7h0/0.jpg" alt="Watch the video" width="400" border="10" />
</a>



#### The cmd line for downloading and installing library onto computer
```
cd <Insert folder path you wish to clone folder into>
git clone https://github.com/PHTHBI/MyRobot.git
cd MyRobot\DynamixelSDK\python
pip install . 
uv sync
```

<br>

### Step 4 - Control the Robot
#### ▶️ How to Use the Control Code (Dynamixel)
<a href="https://www.youtube.com/watch?v=B7lK3koXsyQ" target="_blank">
 <img src="https://img.youtube.com/vi/B7lK3koXsyQ/0.jpg" alt="Watch the video" width="400" border="10" />
</a>


#### ▶️ How to Use the Control Code (Python)
<a href="https://www.youtube.com/watch?v=wZjfYP7Yaws" target="_blank">
 <img src="https://img.youtube.com/vi/wZjfYP7Yaws/0.jpg" alt="Watch the video" width="400" border="10" />
</a>

---

## Constributions 

Haotian Liu & Philip Thun Bisgaard

---


## ⚖️ Rights & Licensing for Dynamixel Servo Code and Setup

This project uses code, configuration practices, and communication protocols related to Dynamixel servos, which are products of ROBOTIS Co., Ltd.

