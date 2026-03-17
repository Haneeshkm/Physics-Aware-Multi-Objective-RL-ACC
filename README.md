# Physics-Aware-Multi-Objective-RL-ACC

**Official MATLAB Implementation** of the research paper:

> **MULTI-OBJECTIVE REINFORCEMENT LEARNING WITH PHYSICS-AWARE VEHICLE DYNAMICS FOR SAFE AND EFFICIENT ADAPTIVE CRUISE CONTROL**  
> Haneesh K. M¹* and Jisha P²  
> ¹Department of Electrical and Electronics Engineering, CHRIST (Deemed to be University), Bangalore, India  
> ²Department of Electronics and Communication Engineering, BMS College of Engineering, Bangalore, India  

**Journal**: International Journal of Intelligent Transportation Systems Research (2026)  
**DOI**: [Add DOI when available]  

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

### ✨ Features
- Full **physics-aware longitudinal dynamics** (aerodynamic drag, rolling resistance, road gradient, friction μ)
- **Multi-objective reward** balancing safety, comfort, energy efficiency & speed tracking
- Supports **DDPG** (recommended) + custom **DQN** agent
- Rich scenario library: Highway, Emergency Brake, Lane Change, Traffic Signal, Varying Road (dry/wet/icy)
- Realistic actuator lag + road-dependent forces
- Fully compatible with MATLAB Reinforcement Learning Toolbox

---

### 📋 Prerequisites
- MATLAB R2022b or newer
- Reinforcement Learning Toolbox
- Deep Learning Toolbox
- (Optional) Parallel Computing Toolbox

---

### 🚀 Quick Start

```matlab
% 1. Train the agent (DDPG - recommended)
run('Train_ACC_DDPG.m')

% 2. Test on highway scenario
run('Highway.m')
Other scenarios (just run the file):

EmergencyBrakeScenario.m
LaneChangeScenario.m
TrafficSignalScenario.m
VaryingRoadScenario.m

Results from the Paper
The physics-aware RL controller achieves:

Stable following with improved safety margins
Smoother control actions (lower jerk)
Better energy efficiency vs. classical ACC/PID
Robust performance under varying road friction and gradients

File Structure
File,Purpose
ACCEnv.m,Main physics-aware RL environment
Train_ACC_DDPG.m,DDPG training script (used in paper)
Highway.m,Highway driving demo + plots
*.Scenario.m,Custom traffic scenarios
DQNAgent.m + replayMemory.m,Alternative DQN implementation
ACCSimulator.m,Legacy simulator
vehicleDynamics.m,Physics model
paper/International_Journal_of_Intelligent_Transportation_Systems_Research.pdf,Full published paper

Pre-trained agent: bestAgent_ACC_TD3_Improved.mat (ready to use)

How to Cite
@article{haneesh2026multi,
  title={Multi-Objective Reinforcement Learning with Physics-Aware Vehicle Dynamics for Safe and Efficient Adaptive Cruise Control},
  author={Haneesh, K.M. and Jisha, P.},
  journal={International Journal of Intelligent Transportation Systems Research},
  year={2026},
  publisher={Springer}
}

📜 License
This code is released under the MIT License (see LICENSE).
The research paper follows standard journal copyright.

Star ⭐ the repo if it helps your work!
For questions or collaboration: haneesh.km@christuniversity.in

