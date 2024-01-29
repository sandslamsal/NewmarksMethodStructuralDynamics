# Newmark's Method Simulation in MATLAB

This repository contains a MATLAB implementation of Newmark's method, a numerical integration scheme used for simulating the dynamic response of a system subjected to a harmonic force. The code is designed to solve equations of motion in structural dynamics and related fields.

## Features

- **Newmark's Method:** Numerical integration technique for dynamic systems.
- **Dynamic System Simulation:** Simulate displacement, velocity, and acceleration over time.
- **Results Visualization:** Table summarizing key results and plots for better insights.

## System Parameters

The simulation considers the following system parameters:

- Eigen-frequency (`wn`)
- Damping ratio (`eta`)
- Mass (`M`)
- Stiffness (`K`)
- Damping (`C`)
- Time step (`dt`)
- Harmonic force amplitude (`F0`)
- Pulsation of the harmonic force (`w`)

## Usage

1. Clone the repository:

   ```bash
   git clone https://github.com/sandslamsal/newmarks-method-simulation-Dynamics.git
