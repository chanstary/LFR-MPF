Self-Organizing Motion Planning for Lane-Free Roundabouts (LFR-MPF)
This repository contains the core Python implementation of the Lane-Free Roundabout Self-Organized Motion Planning Framework (LFR-MPF), as presented in the paper:

Chen, X., Zhang, W., Bai, H., et al. (2025). The Revolution of Roundabouts in the Autonomous Driving Era: A Lane-Free Self-Organized Motion Planning Framework. Advanced Engineering Informatics.

This code is provided to support academic research, enhance reproducibility, and facilitate future studies in the field of autonomous vehicle motion planning in complex, lane-free environments.

Framework Overview
The LFR-MPF is a decentralized motion planning framework designed for non-connected Autonomous Vehicles (AVs) in lane-free roundabouts. It enables safe and efficient navigation using only onboard sensing and local interactions, without reliance on V2X communication or centralized control. The framework is built upon three core components:

Enhanced Intelligent Agent Model (IAM): Simulates microscopic vehicle interactions in polar coordinates.

Target Guidance Model: Provides flexible, long-range path planning and exit guidance.

Self-Organizing Yielding Strategy: Enables autonomous conflict resolution at entries and within the roundabout.

About this Implementation
This repository provides the fundamental Python scripts for the simulation logic.

Simulation Core: The code is written in Python and heavily utilizes numpy for efficient numerical computations.

Inspiration: The microscopic interaction models are inspired by the foundational work on traffic simulation by Treiber and Kesting, particularly the Intelligent Driver Model (IDM) and social force concepts.

Context of Use: In our research, this Python backend was integrated with a custom Java-based GUI. The GUI was responsible for visualizing the simulation in real-time and allowing researchers to define the roundabout geometry (e.g., road boundaries, entry/exit points) and simulation parameters. This code represents the "brains" of that system.

Disclaimer: This implementation is a simplified version of the code used for the paper, intended for demonstration and to illustrate the core algorithms. It is provided "as-is" and may require modifications or the development of a visualization front-end to be fully runnable and replicate the paper's results.

Project Structure
The code is organized into several modules for clarity:

LFR-MPF-Simulation/
│
├── main.py               # Main simulation script to run the simulation loop
├── config.py             # Contains all global parameters and simulation settings
├── vehicle.py            # The Vehicle class, defining state and behaviors
├── models.py             # Core traffic models (IDM, IAM, etc.)
├── utils.py              # Helper functions (finding vehicles, geometry calculations)
├── visualization.py      # Placeholder for plotting and visualization logic
│
├── README.md             # This file
└── requirements.txt      # Required Python libraries

How to Use (Conceptual)
Install Dependencies:

pip install -r requirements.txt

Configure Simulation: Adjust parameters in config.py to define the roundabout geometry, vehicle properties, and model constants.

Run Simulation: Execute the main script.

python main.py

Visualize Results: The visualization.py module contains basic functions to plot vehicle trajectories after the simulation completes. For real-time animation, a more advanced graphics library (like Pygame or Matplotlib's animation API) or an external GUI would be needed.

Citation
If you use this code or the concepts from our paper in your research, please cite:

@article{chen2025lfr,
  title={The Revolution of Roundabouts in the Autonomous Driving Era: A Lane-Free Self-Organized Motion Planning Framework},
  author={Chen, Xingyu and Zhang, Weihua and Bai, Haijian and Zhu, Feng and Ding, Heng and Wang, Liangwen},
  journal={Advanced Engineering Informatics},
  year={2025},
  publisher={Elsevier}
}


As autonomous driving advances, lane-free roundabouts (LFRs) offer potential for improved traffic efficiency. However, ensuring safe and efficient passage (entry, circulation, exit) through LFRs is a challenge for non-connected autonomous vehicles (AVs) relying solely on their sensors. To address this, we propose the LFR-MPF based on an intelligent agent model (IAM) @der_Physiker. It enables traffic flow to self-organize through local vehicle interactions, allowing AVs to operate safely and efficiently in complex LFR environments.

@TrafficFluid DSSL TUC has been at the forefront of lane-free strategy research, providing numerous insightful videos that showcase their findings. Their work is crucial in understanding and developing effective lane-free traffic systems. Many of their studies and resources can be found on their website, trafficfluid.tuc.gr.

We conducted simulation validation in the complex Paris Charles de Gaulle roundabout, focusing on evaluating LFR-MPF's performance and stability under dynamic traffic loads simulating peak and off-peak cycles. An 8-minute traffic simulation, accelerated to 1.5 minutes in this video (see: https://www.bilibili.com/video/BV1exAGesEqr/), was divided into four consecutive stages (Stage 1: 0-2 min, Stage 2: 2-4 min, Stage 3: 4-6 min, Stage 4: 6-8 min). This simulates a complete cycle of traffic load increasing from low to high and then decreasing. This staged design allows for detailed observation of how traffic flow evolves under different load levels, particularly the formation, development, and dissipation of congestion.

 ![image](https://github.com/user-attachments/assets/b90f529a-c159-4a37-8b03-09e64f7b4a97)

Fig: Microscopic trajectory (Start time of each trajectory adjusted to 0) and safety analysis.
