# Multi-Objective Reinforcement Learning with Physics-Aware Vehicle Dynamics for ACC

**Official MATLAB Implementation** of the research paper:

> **"MULTI-OBJECTIVE REINFORCEMENT LEARNING WITH PHYSICS-AWARE VEHICLE DYNAMICS FOR SAFE AND EFFICIENT ADAPTIVE CRUISE CONTROL"**  
> Haneesh K. M¹* and Jisha P²  
> *¹CHRIST (Deemed to be University), Bangalore*  
> *²BMS College of Engineering, Bangalore*

**Journal**: International Journal of Intelligent Transportation Systems Research (2026)  
**DOI**: [Add DOI when available] | **PDF**: [paper/International_Journal_of_Intelligent_Transportation_Systems_Research.pdf](paper/International_Journal_of_Intelligent_Transportation_Systems_Research.pdf)

---

### ✨ Features
- Full **physics-aware longitudinal dynamics** (aerodynamic drag, rolling resistance, road gradient, friction coefficient μ)
- **Multi-objective reward** balancing safety, comfort, energy efficiency, and speed tracking
- Supports **DDPG** (recommended) and custom **DQN** agents
- Rich scenario library: Highway, Emergency Brake, Lane Change, Traffic Signal, Varying Road Conditions (dry/wet/icy)
- Realistic actuator dynamics (1st-order lag) + road-dependent forces
- MATLAB Reinforcement Learning Toolbox compatible

---

### 📋 Prerequisites
- MATLAB R2022b or newer
- **Reinforcement Learning Toolbox**
- **Deep Learning Toolbox**
- (Optional) Parallel Computing Toolbox for faster training

---

### 🚀 Quick Start

run('src/Train_ACC_DDPG.m')
→ Saves efficient_acc_agent.mat (best agent)
run('src/Highway.m')
Other Scenarios

EmergencyBrakeScenario.m
LaneChangeScenario.m
TrafficSignalScenario.m
VaryingRoadScenario.m
Results from Paper
The physics-aware RL controller achieves:

Stable following with improved safety margins
Smoother acceleration (lower jerk)
Better energy efficiency compared to classical PID/ACC
Robust performance under varying road friction and gradients

(Plots generated automatically in Highway.m and analysis function)


File,Purpose
ACCEnv.m,Main physics-aware RL environment
Train_ACC_DDPG.m,DDPG training script (paper version)
Highway.m,Highway driving demo
*.Scenario.m,Custom traffic scenarios
DQNAgent.m + replayMemory.m,Alternative DQN implementation

@article{haneesh2025multi,
  title={Multi-Objective Reinforcement Learning with Physics-Aware Vehicle Dynamics for Safe and Efficient Adaptive Cruise Control},
  author={Haneesh, K.M. and Jisha, P.},
  journal={International Journal of Intelligent Transportation Systems Research},
  year={2026},
  publisher={Springer}
}


