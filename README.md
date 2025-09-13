# Drone Stabilization Using AI

## Table of Contents
- [Introduction](#introduction)  
- [Methodology](#methodology)  
  - [Libraries Used](#libraries-used)  
  - [Project Files](#project-files)  
  - [Environment](#environment)  
  - [PID Controller](#pid-controller)  
  - [Dataset & Regression](#dataset--regression)  
- [Results](#results)  
- [Conclusion](#conclusion)  

---

## Introduction
This project focuses on stabilizing a drone in two dimensions (X and Y) against various disturbances such as wind conditions, gravity, and inertia using **linear regression**.

---

## Methodology

### Libraries Used
- **Pygame** – for creating the window and environment  
- **Pymunk** – for implementing physics elements such as forces, objects, and walls  
- **Random** – for generating random wind conditions  
- **Math** – for trigonometric calculations (mainly wind direction)  
- **Numpy** – for handling arrays and reading data from files  
- **TensorFlow** – for ML algorithms and error/accuracy calculations  

### Project Files
- **PID_Environment.py** – runs the environment with a PID-stabilized drone, allowing user movement  
- **AI_Environment.py** – similar to the above, but the drone is stabilized using the regression model + accuracy calculation algorithms  
- **Data.csv** – dataset with ~3000 rows and 11 columns  
- **model_train.py** *(optional)* – trains an alternative model using **Stochastic Gradient Descent (SGD)**  

### Environment
The drone is modeled as a rectangular object with a mass of 8kg and perpendicular forces at each end representing propellers.  

- The **target position** can be controlled using **W, A, S, D** keys.  
- Wind can be toggled using the **V key**.  

A function simulates wind conditions (direction and intensity), updated every 5 seconds. A top-left panel displays these using arrows.  

### PID Controller
To generate a correct dataset for training, a **PID controller (Proportional, Integral, Derivative)** was used first.  

- Three separate PID controllers were implemented:
  - Error correction on the X-axis  
  - Error correction on the Y-axis  
  - Error correction for the drone’s angle  

Each controller provides feedback for both motors.  

Error vectors were defined (`errors_x`, `errors_y`, `errors_angle`) containing:  
- Current error  
- Previous cycle error  
- Error two cycles ago  

This results in **9 error values as input**, which generate the necessary forces to stabilize the drone at the target position.  

### Dataset & Regression
- Dataset: ~6000 rows, 11 columns  
  - First 9 = input errors  
  - Last 2 = output forces applied to the propellers  
- Data is normalized to [-1, 1].  

The **linear regression model** approximates functions close to the dataset and stabilizes the drone under new random conditions.  

---

## Results
- Using **LinearRegression()**, the model achieved **acceptable performance** with relatively low **Mean Squared Error**.  
- **R² scores** were slightly above 0.5, indicating moderate but not perfect performance.  
- An alternative **SGD optimization model** performed slightly better but caused a significant FPS drop, negatively impacting simulation performance.  

---

## Conclusion
- Training data was not perfect since even the original PID controller fails in some cases.  
- PID parameters (**Kp, Ki, Kd**) required long trial-and-error adjustments.  
- Despite limitations, the results were **satisfactory**, and the regression model provided a reasonable approximation for drone stabilization.  

---
