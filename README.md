
```markdown
# Self-Organizing Motion Planning for Lane-Free Roundabouts (LFR-MPF)

This repository contains the core Python implementation of the **Lane-Free Roundabout Self-Organized Motion Planning Framework (LFR-MPF)**. The work is presented in the paper:

> Chen, X., Zhang, W., Bai, H., et al. (2025). The Revolution of Roundabouts in the Autonomous Driving Era: A Lane-Free Self-Organized Motion Planning Framework. *Advanced Engineering Informatics*.

This code is provided to support academic research, enhance reproducibility, and facilitate future studies in the field of autonomous vehicle motion planning in complex, lane-free environments.

## 📝 Abstract

As autonomous driving advances, lane-free roundabouts (LFRs) offer potential for improved traffic efficiency. However, ensuring safe and efficient passage (entry, circulation, exit) through LFRs is a challenge for non-connected autonomous vehicles (AVs) relying solely on their sensors. To address this, we propose the LFR-MPF based on an intelligent agent model (IAM), inspired by the foundational work of [Dirk Helbing](https://www.trafficfluid.tuc.gr/members/dirk-helbing/) on social force models. It enables traffic flow to self-organize through local vehicle interactions, allowing AVs to operate safely and efficiently in complex LFR environments.

## 🎥 Demonstration

We conducted simulation validation in the complex Paris Charles de Gaulle roundabout. This scenario evaluates LFR-MPF's performance and stability under dynamic traffic loads simulating a complete cycle from off-peak to peak hours and back.

An 8-minute traffic simulation is accelerated in the demonstration video below. The simulation is divided into four consecutive stages to show how traffic flow evolves under different load levels, particularly the formation, development, and dissipation of congestion.

**➡️ Watch the simulation video on Bilibili: [https://www.bilibili.com/video/BV1exAGesEqr/](https://www.bilibili.com/video/BV1exAGesEqr/)**

<p align="center">
  <img src="https://github.com/user-attachments/assets/b90f529a-c159-4a37-8b03-09e64f7b4a97" alt="Microscopic Trajectory Analysis" />
  <br>
  <em>Fig: Microscopic trajectory (start time of each trajectory adjusted to 0) and safety analysis from the simulation.</em>
</p>


## 🛠️ Framework Overview

The LFR-MPF is a decentralized motion planning framework designed for non-connected Autonomous Vehicles (AVs) in lane-free roundabouts. It enables safe and efficient navigation using only onboard sensing and local interactions, without reliance on V2X communication or centralized control. The framework is built upon three core components:

- **Enhanced Intelligent Agent Model (IAM):** Simulates microscopic vehicle interactions in polar coordinates.
- **Target Guidance Model:** Provides flexible, long-range path planning and exit guidance.
- **Self-Organizing Yielding Strategy:** Enables autonomous conflict resolution at entries and within the roundabout.

<p align="center">
  <img width="886" height="718" alt="image" src="https://github.com/user-attachments/assets/b5ff01ca-04aa-474c-9b6b-4d8a77cf7a47" />

  <br>
  <em>Fig: Lane-Free Roundabout Self-Organizing Motion Planning Framework.</em>
</p>


## 💻 About this Implementation

This repository provides the fundamental Python scripts for the simulation logic.

- **Simulation Core:** The code is written in Python and heavily utilizes `numpy` for efficient numerical computations.
- **Inspiration:** The microscopic interaction models are inspired by the foundational work on traffic simulation by Treiber and Kesting, particularly the Intelligent Driver Model (IDM) and social force concepts.
- **Context of Use:** In our research, this Python backend was integrated with a custom Java-based GUI. The GUI was responsible for visualizing the simulation in real-time and allowing researchers to define the roundabout geometry and simulation parameters. This code represents the "brains" of that system.

> **Disclaimer:** This implementation is a simplified version of the code used for the paper, intended for demonstration and to illustrate the core algorithms. It is provided "as-is" and may require modifications or the development of a visualization front-end to be fully runnable and replicate the paper's results.

## 📂 Project Structure

The code is organized into several modules for clarity:
```

LFR-MPF-Simulation/
│
├── main.py           \# Main script to run the simulation loop
├── config.py         \# Contains all global parameters and simulation settings
├── vehicle.py        \# The Vehicle class, defining state and behaviors
├── models.py         \# Core traffic models (IDM, IAM, etc.)
├── utils.py          \# Helper functions (finding vehicles, geometry calculations)
├── visualization.py  \# Placeholder for plotting and visualization logic
│
├── README.md         \# This file
└── requirements.txt  \# Required Python libraries

````

## 🚀 Getting Started

### 1. Install Dependencies
```bash
pip install -r requirements.txt
````

### 2\. Configure Simulation

Adjust parameters in `config.py` to define the roundabout geometry, vehicle properties, and model constants.

### 3\. Run Simulation

Execute the main script from your terminal:

```bash
python main.py
```

### 4\. Visualize Results

The `visualization.py` module contains basic functions to plot vehicle trajectories after the simulation completes. For real-time animation, a more advanced graphics library (like Pygame or Matplotlib's animation API) or an external GUI would be needed.

## 🙏 Acknowledgements

The research into lane-free strategies by the **TrafficFluid DSSL team at the Technical University of Crete** has been a significant source of inspiration. Their work is crucial in understanding and developing effective lane-free traffic systems, and their numerous insightful videos have greatly informed our approach. Many of their studies and resources can be found on their website: [trafficfluid.tuc.gr](https://trafficfluid.tuc.gr).

## 📖 Citation

If you use this code or the concepts from our paper in your research, please cite:

```bibtex
@article{chen2025lfr,
  title={The Revolution of Roundabouts in the Autonomous Driving Era: A Lane-Free Self-Organized Motion Planning Framework},
  author={Chen, Xingyu and Zhang, Weihua and Bai, Haijian and Zhu, Feng and Ding, Heng and Wang, Liangwen},
  journal={Advanced Engineering Informatics},
  year={2025},
  publisher={Elsevier}
}
```

```
```
