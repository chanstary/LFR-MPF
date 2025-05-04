# LFR-MPF

LFR-MPF: Target-Guided Motion Planning for AVs in Self-Organizing LFR

As autonomous driving advances, lane-free roundabouts (LFRs) offer potential for improved traffic efficiency. However, ensuring safe and efficient passage (entry, circulation, exit) through LFRs is a challenge for non-connected autonomous vehicles (AVs) relying solely on their sensors. To address this, we propose the LFR-MPF based on an intelligent agent model (IAM) @der_Physiker. It enables traffic flow to self-organize through local vehicle interactions, allowing AVs to operate safely and efficiently in complex LFR environments.

@TrafficFluid DSSL TUC has been at the forefront of lane-free strategy research, providing numerous insightful videos that showcase their findings. Their work is crucial in understanding and developing effective lane-free traffic systems. Many of their studies and resources can be found on their website, trafficfluid.tuc.gr.

We conducted simulation validation in the complex Paris Charles de Gaulle roundabout, focusing on evaluating LFR-MPF's performance and stability under dynamic traffic loads simulating peak and off-peak cycles. An 8-minute traffic simulation, accelerated to 1.5 minutes in this video (see: https://www.bilibili.com/video/BV1exAGesEqr/), was divided into four consecutive stages (Stage 1: 0-2 min, Stage 2: 2-4 min, Stage 3: 4-6 min, Stage 4: 6-8 min). This simulates a complete cycle of traffic load increasing from low to high and then decreasing. This staged design allows for detailed observation of how traffic flow evolves under different load levels, particularly the formation, development, and dissipation of congestion.

 ![image](https://github.com/user-attachments/assets/b90f529a-c159-4a37-8b03-09e64f7b4a97)

Fig: Microscopic trajectory (Start time of each trajectory adjusted to 0) and safety analysis.
