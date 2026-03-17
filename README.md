# Multi-Objective Reinforcement Learning with Physics-Aware Vehicle Dynamics for ACC

**Official MATLAB Implementation** of the research paper:

> **"MULTI-OBJECTIVE REINFORCEMENT LEARNING WITH PHYSICS-AWARE VEHICLE DYNAMICS FOR SAFE AND EFFICIENT ADAPTIVE CRUISE CONTROL"**  
> Haneesh K. M¹* and Jisha P²  
> *¹CHRIST (Deemed to be University), Bangalore*  
> *²BMS College of Engineering, Bangalore*

**Journal**: International Journal of Intelligent Transportation Systems Research (2025)  
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

```bash
git clone https://github.com/YOUR_USERNAME/Physics-Aware-Multi-Objective-RL-ACC.git
cd Physics-Aware-Multi-Objective-RL-ACC
