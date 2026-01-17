# Pure Pursuit Controller for F1TENTH

This repository contains an implementation of the **Pure Pursuit path tracking algorithm** for the **F1TENTH autonomous racing platform**.  
The project supports both **simulation environments** and **real F1TENTH vehicle deployment**.

Pure Pursuit is a geometric controller commonly used in mobile robotics and autonomous vehicles to follow a predefined trajectory by continuously steering toward a lookahead point on the path.

---

## 🚗 Project Scope

This repository is designed for:
- Testing and validating the Pure Pursuit algorithm in **simulation**
- Deploying and running the controller on a **real F1TENTH car**
- Educational and research purposes related to autonomous driving and mobile robotics

The implementation focuses on:
- Path tracking accuracy
- Integration with ROS / ROS2
- Compatibility with the F1TENTH software stack

---

## 📁 Repository Structure

pure_pursuit/
├── simulation/ # Code and configuration for simulation
├── real/ # Code for running on real F1TENTH vehicle
├── config/ # Parameters and tuning files
├── scripts/ # Controller and helper scripts
├── launch/ # Launch files
└── README.md


> The structure may evolve as the project develops.

---

## 🧠 Algorithm Overview

The **Pure Pursuit** controller works by:
1. Selecting a lookahead point on the reference path
2. Computing the curvature required to reach that point
3. Converting curvature into steering commands
4. Continuously updating commands based on vehicle state feedback

This approach is simple, efficient, and widely used in autonomous vehicle research.

---

## 🚀 Running the Project

> ⚠️ **Instructions for running the simulation and the real vehicle will be added later.**

This section will include:
- Simulation setup
- Launch commands
- Parameter tuning
- Running on real F1TENTH hardware

---

## 🛠️ Requirements

- ROS / ROS2 (depending on setup)
- F1TENTH software stack
- Python / C++ (depending on implementation)
- Simulation environment (e.g., Gazebo)

---

## 📌 Notes

- This project is under active development.
- Parameters may require tuning depending on the environment and vehicle.
- Use with caution on real hardware.

---

## 📄 License

This project is intended for educational and research use.  
License information will be added if needed.

---

## ✨ Author

Developed by **Âu Dương Tài**  with the support of **Trần Duy Nhất** , **Huỳnh Công Danh**, **Phạm Như Ý**, **Bùi Hoàng Dũng**,**Võ Tuấn Đạt** 
For learning, experimentation, and autonomous racing research.
