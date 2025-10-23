
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

## 📊 Human-like Calibration & Validation

To ensure the LFR-MPF model produces realistic and human-plausible driving behavior, its key parameters were calibrated and validated using the real-world RounD dataset 8. This dataset provides detailed vehicle trajectories from the Neuweiler "lane-less" roundabout in Germany.

The calibration process anchored the model's dynamics (like desired speed $v_0$, time gap $T$, and acceleration limits $a_{max}, b_{max}$) in empirically observed human driving patterns. The calibrated model was then validated against a separate set of trajectories, confirming its ability to accurately replicate real-world vehicle motion in terms of position, speed, and acceleration.

 <p align="center">
<img width="357" height="266" alt="image" src="https://github.com/user-attachments/assets/9c7080b1-623c-43e6-8e3e-4a17d15ba7bd" /><img width="533" height="266" alt="image" src="https://github.com/user-attachments/assets/3c43076e-74b1-4e8c-a4b9-30123e937f34" />

  <br>
  <em>Fig: High-resolution satellite image of the Neuweiler lane-less roundabout.</em>
</p>


<p align="center">
  <img width="975" height="274" alt="image" src="https://github.com/user-attachments/assets/b513afb8-2e4b-4aa1-82e0-75374bead11e" />
</p>
<p align="center">
  <img width="975" height="765" alt="image" src="https://github.com/user-attachments/assets/939eaa69-7044-4544-82e2-2eb91d32f8e4" />

<br>
  <em>Fig: LFR-MPF trajectory fitting performance in calibration and validation.</em>
</p>

## 💻 About this Implementation

This repository provides the fundamental Python scripts for the simulation logic.

- **Simulation Core:** The code is written in Python and heavily utilizes `numpy` for efficient numerical computations.
- **Inspiration:** The microscopic interaction models are inspired by the foundational work on traffic simulation by Treiber and Kesting, particularly the Intelligent Driver Model (IDM) and social force concepts.
- **Context of Use:** In our research, this Python backend was integrated with a custom Java-based GUI. The GUI was responsible for visualizing the simulation in real-time and allowing researchers to define the roundabout geometry and simulation parameters. This code represents the "brains" of that system.

> **Disclaimer:** This implementation is a simplified version of the code used for the paper, intended for demonstration and to illustrate the core algorithms. It is provided "as-is" and may require modifications or the development of a visualization front-end to be fully runnable and replicate the paper's results.
> 
Integration with Web-Based Visualization
To achieve a full, real-time visualization as shown in our research, this Python backend must be connected to a compatible visualization front-end. The front-end used in our work is based on the movsim/traffic-simulation-de project.

Frontend Repository: movsim/traffic-simulation-de
Frontend Repository: [movsim/traffic-simulation-de](https://github.com/movsim/traffic-simulation-de)

This JavaScript application is responsible for rendering the vehicle movements and roundabout geometry in a web browser. The Python code in this repository acts as the simulation server, performing all the physics calculations and passing the state of each vehicle (e.g., position, speed) to the JavaScript client at each time step.

## 🌍 Scenarios and Reproducibility

To ensure full transparency and allow for independent validation of our findings across different geometries, this repository includes the necessary setup instructions and configuration files for the key scenarios presented in the paper. This directly addresses reviewer feedback regarding external validity and reproducibility.

The following simulation environments are provided within the `scenarios/` directory:

-   [cite_start]**RounD Dataset Validation (`scenarios/round/`)**: Contains scripts and configurations used for the human-like calibration and cross-site validation presented in **Section 4** [cite: 1051-1322, 1340-1404], using data from the Neuweiler and Thiergarten roundabouts.
-   [cite_start]**Paris Charles de Gaulle Simulation (`scenarios/paris_cdg/`)**: Includes the environment setup for the large-scale, high-density simulations detailed in **Section 5** [cite: 1417-1900].
-   **Xi'an Bell Tower Configuration (`scenarios/xian_bell_tower/`)**: Provides the geometric configuration and basic setup for the Xi'an Bell Tower roundabout, including the raw OpenStreetMap data (`map.osm`) and the generated SUMO network file (`map.net.xml`). While a full analysis of this scenario is presented as future work in the paper, its inclusion here **demonstrates the framework's applicability to other complex, large-scale geometries** and allows researchers to readily extend our work.

Researchers can utilize these files to reproduce our core validation results or adapt them for further studies. Please refer to the `README.md` file within each scenario directory for specific instructions.

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
