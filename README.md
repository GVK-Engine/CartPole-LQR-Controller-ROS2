
ROS2 implementation of a Cart-Pole system stabilized with LQR control | Space Robotics and AI
# 🤖 Cart-Pole LQR Controller (ROS 2)

**Author:** Vamshikrishna Gadde  


![ROS2](https://img.shields.io/badge/Framework-ROS2-blue)
![Python](https://img.shields.io/badge/Language-Python3-yellow)
![Controller](https://img.shields.io/badge/Type-LQR_Control-green)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

---

## 🛰️ Overview
This project implements a **Linear Quadratic Regulator (LQR)** to stabilize an inverted pendulum on a moving cart using **ROS 2**.  
The controller minimizes both **cart position** and **pole angle deviation** while respecting actuator limits.  
State-space modeling, optimal control, and PD-like feedback are used for real-time stability in Gazebo simulation.

---

## 🧩 Repository Structure
```
CartPole-LQR-Controller-ROS2/
├── README.md
├── report/ASSIGNMENT_2_SPACE_ROBOTICS_AND_AI.pdf
├── src/cart_pole_lqr_controller.py
└── results/
    ├── cart_position_velocity.png
    ├── pole_angle_velocity.png
    └── control_force_plot.png
```

---

## ⚙️ Dependencies
| Component | Description |
|------------|-------------|
| **ROS 2 Humble or Foxy** | Middleware for node communication |
| **rclpy** | Python ROS 2 API |
| **NumPy, SciPy** | Matrix math and LQR solution |
| **Gazebo / RViz 2** | Simulation visualization |

Install:
```bash
pip install numpy scipy
```

---

## 🚀 Run the Controller
```bash
# Build and source
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch the sim
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

---

## 🧮 State-Space Model
State vector:

$$
x = \begin{bmatrix} x_c \\ \dot{x_c} \\ \theta \\ \dot{\theta} \end{bmatrix}, 
\qquad
\dot{x} = A x + B u
$$

**System Matrices**

$$
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & \frac{m g}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & \frac{(M + m) g}{M L} & 0
\end{bmatrix},
\quad
B = \begin{bmatrix}
0 \\ 1/M \\ 0 \\ -1/(M L)
\end{bmatrix}
$$

---

## ⚙️ LQR Formulation
The LQR controller minimizes the cost function:

$$
J = \int (x^T Q x + u^T R u)\,dt
$$

Resulting feedback law:

$$
u = - Kx
$$

### **Optimal Gain Computation**
$$
A^T P + P A - P B R^{-1} B^T P + Q = 0 
\Rightarrow K = R^{-1} B^T P
$$

---

## 🧩 Tuned Matrices
| Matrix | Values | Purpose |
|---------|---------|----------|
| Q | diag ( 100, 50, 100, 10 ) | Higher weights on cart position and pole angle |
| R | [ 0.01 ] | Lower cost → allows aggressive control |

---

## 📊 Performance
- **Settling time:** ≈ 2.5 s  
- **Overshoot:** None  
- **Average force:** ≤ 10 N (clipped to ±15 N)  
- **Stable angle response** with minimal oscillation  

![Cart Position & Velocity]<img width="783" height="223" alt="cart" src="https://github.com/user-attachments/assets/05aba592-4430-41fe-b2b6-a6138926eb0f" />

![Pole Angle & Velocity]<img width="787" height="220" alt="Pole" src="https://github.com/user-attachments/assets/a4ad789a-4523-48fa-a00c-c03b8acc87d9" />

![Control Force] <img width="787" height="218" alt="Control" src="https://github.com/user-attachments/assets/99c93412-1cf8-45f1-832a-0d0628789238" />


📺 **Simulation Video:** [Google Drive Link](https://drive.google.com/file/d/15NFFquUA6RoZW3iBdSnGAeDoxM7-QIuO/view?usp=sharing)

---

## 🧠 Key Insights
- Larger Q₁₁ and Q₃₃ weights → faster position and angle correction.  
- Lower R value → stronger control actions within limits.  
- Real-time filtering (EWMA) reduces sensor noise.  
- Confirms stability via MATLAB and ROS2 plots.

---

## 🔮 Future Optimizations – Bayesian Tuning
Use Bayesian Optimization to auto-tune Q and R:  
1️⃣ Define objective (minimize settling time, control effort).  
2️⃣ Iteratively update parameters.  
3️⃣ Integrate with ROS2 for adaptive learning.

---

## 🧮 Equation Summary

**Control Law**

$$
u(t) = - K \big(x(t) - x_{ref}(t)\big)
$$

**Linear and Angular Dynamics for Comparison**

$$
v = K_p^{(lin)} d + K_d^{(lin)} \frac{\Delta d}{\Delta t}
\qquad
\omega = K_p^{(ang)} \theta_{error} + K_d^{(ang)} \frac{\Delta \theta}{\Delta t}
$$

---

## 📚 References
- SciPy LQR Solver – `scipy.linalg.solve_continuous_are()`  
- Gazebo ROS 2 Plugins  
- Space Robotics and AI Course Material  

---

## Demonstrates:  
- **Optimal control system design** for dynamic stability.  
- **State-space modelling and matrix analysis.**  
- **Integration of simulation + hardware loops via ROS 2.**  
- **Tuning automation concepts** like Bayesian optimization and reinforcement learning.  

---

## 👤 Author
**Vamshikrishna Gadde**  
📧 vgadde5@asu.edu  
🔗 [LinkedIn](https://www.linkedin.com/in/vamshikrishna-gadde9999/)  
💻 [GitHub](https://github.com/GVK-Engine/VAMSHIKRISHNA-GADDE)

---

## 🪪 License
MIT License – Free for educational use.
