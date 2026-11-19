<!-- markdownlint-disable -->
<h1 align="center">
    best-of-robot-simulators
    <br>
</h1>

<p align="center">
    <strong>🏆&nbsp; A ranked list of awesome projects. Updated weekly.</strong>
</p>

<p align="center">
    <a href="https://best-of.org" title="Best-of Badge"><img src="http://bit.ly/3o3EHNN"></a>
    <a href="#Contents" title="Project Count"><img src="https://img.shields.io/badge/projects-180-blue.svg?color=5ac4bf"></a>
    <a href="#Contribution" title="Contributions are welcome"><img src="https://img.shields.io/badge/contributions-welcome-green.svg"></a>
    <a href="https://github.com/knmcguire/best-of-robot-simulators/releases" title="Best-of Updates"><img src="https://img.shields.io/github/release-date/knmcguire/best-of-robot-simulators?color=green&label=updated"></a>
</p>


This curated list contains 180 awesome simulator projects with a total of 510K stars grouped into 11 categories. All projects are ranked by a project-quality score, which is calculated based on various metrics automatically collected from GitHub and different package managers. If you like to add or update projects, feel free to open an [issue](https://github.com/knmcguire/best-of-robot-simulators/issues/new/choose), submit a [pull request](https://github.com/knmcguire/best-of-robot-simulators/pulls), or directly edit the [projects.yaml](https://github.com/knmcguire/best-of-robot-simulators/edit/main/projects.yaml). Contributions are very welcome!

> 🧙‍♂️  Discover other [best-of lists](https://best-of.org) or [create your own](https://github.com/best-of-lists/best-of/blob/main/create-best-of-list.md).

> [!WARNING]
> Currently the git meta updates to the list have been stopped and we are using [library.io](https://libraries.io/) to retrieve the metadata of the projects, which catches most but unfortunately does not grab all the data. We are keeping an eye for an upstream fix through this issue: https://github.com/knmcguire/best-of-robot-simulators/issues/275

### Definition Robotics Simulator
Here is a definition of a robotics simulator derived in [this blogpost](https://www.mcguirerobotics.com/blog/2025/04/17/navigating-through-the-robotic-simulation-landscape/)

> A robotic simulator is a software framework that provides a virtual environment, often leveraging different physics/rendering engines and sensor models, to model the robot's behavior, its interaction and perception with the simulated world for design, evaluative or data-generative purposes.

With:

* **virtual environment** - To provide the scenario for the simulated robot to act in, depending on the application, like an indoor building, forest, or lunar landscape.
* **behavior, its interaction and perception** - The simulated entity should be able to interact with and act upon that virtual environment or world through its simulated sensors and actuators.
* **physics/rendering engines and sensor models** - To be able to simulate those interactions and perceptions caused by the robot's behavior, to model how an object will slip while being grasped or the noise of the lidar ranges.
* **design, evaluative or data-generative** - To use this as a development tool, as part of continuous integration to assure quality, or to collect data that can be used for AI training purposes.

## Contents

- [Generic Robotics Simulators](#generic-robotics-simulators) _28 projects_
- [Robotic simulators in 2D](#robotic-simulators-in-2d) _7 projects_
- [Aerial Robotics Simulators](#aerial-robotics-simulators) _33 projects_
- [Maritime Robotics Simulators](#maritime-robotics-simulators) _20 projects_
- [Automotive Simulators](#automotive-simulators) _5 projects_
- [Space Robotics Simulators](#space-robotics-simulators) _6 projects_
- [AI training Simulators](#ai-training-simulators) _37 projects_
- [Other Domain Specific Simulators](#other-domain-specific-simulators) _10 projects_
- [Game engines](#game-engines) _6 projects_
- [Physics Engines](#physics-engines) _22 projects_
- [Rendering engines](#rendering-engines) _7 projects_
- [Others](#others) _1 projects_

## Explanation
- 🥇🥈🥉&nbsp; Combined project-quality score
- ⭐️&nbsp; Star count from GitHub
- 🐣&nbsp; New project _(less than 6 months old)_
- 💤&nbsp; Inactive project _(12 months no activity)_
- 💀&nbsp; Dead project _(999999 months no activity)_
- 📈📉&nbsp; Project is trending up or down
- ➕&nbsp; Project was recently added
- 👨‍💻&nbsp; Contributors count from GitHub
- 🔀&nbsp; Fork count from GitHub
- 📋&nbsp; Issue count from GitHub
- ⏱️&nbsp; Last update timestamp on package manager
- 📥&nbsp; Download count from package manager
- 📦&nbsp; Number of dependent projects

<br>

## Generic Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Generic simulators, tools or SDKs made for robotics_

<details><summary><b><a href="https://mujoco.org/">mujoco</a></b> (🥇40 ·  ⭐ 18K) - Multi-Joint dynamics with Contact. A general purpose physics simulator. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google-deepmind/mujoco) (👨‍💻 190 · 🔀 2.2K · 📥 1.5M · 📦 6.3K · 📋 2.2K - 17% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/google-deepmind/mujoco
	```
</details>
<details><summary><b><a href="https://o3de.org/">O3DE for Robotics</a></b> (🥇30 ·  ⭐ 9.9K) - Open 3D Engine (O3DE) is an Apache 2.0-licensed multi-platform 3D engine that enables developers and content creators.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Apache-2">MIT and Apache-2</a></code></summary>

- [GitHub](https://github.com/o3de/o3de) (👨‍💻 360 · 🔀 2.6K · 📥 13K · 📋 8.2K - 42% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://www.cyberbotics.com/">Webots</a></b> (🥇30 ·  ⭐ 4.8K) - Webots Robot Simulator. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/cyberbotics/webots) (👨‍💻 160 · 🔀 2K · 📥 2M · 📋 2K - 12% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/cyberbotics/webots
	```
</details>
<details><summary><b><a href="https://newton-physics.github.io/newton/">Newton (Physics)</a></b> (🥈27 ·  ⭐ 6K) - An open-source, GPU-accelerated physics simulation engine built upon NVIDIA Warp, specifically targeting roboticists.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/newton-physics/newton) (👨‍💻 140 · 🔀 760 · 📥 250 · 📋 1.9K - 16% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/newton-physics/newton
	```
</details>
<details><summary><b><a href="https://gazebosim.org/home">Gazebo</a></b> (🥈22 ·  ⭐ 1.6K) - Open source robotics simulator. The latest version of Gazebo. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/gazebosim/gz-sim) (👨‍💻 210 · 🔀 520 · 📋 1.2K - 42% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/gazebosim/gz-sim
	```
</details>
<details><summary><b><a href="https://github.com/Unity-Technologies/Unity-Robotics-Hub">Unity Robotics Hub</a></b> (🥈19 ·  ⭐ 2.6K) - Central repository for tools, tutorials, resources, and documentation for robotics simulation in Unity. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) (👨‍💻 15 · 🔀 470 · 📋 200 - 27% open · ⏱️ 22.09.2026):

	```
	git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub
	```
</details>
<details><summary><b><a href="https://openrave.org/">OpenRAVE</a></b> (🥈19 ·  ⭐ 820) - Open Robotics Automation Virtual Environment: An environment for testing, developing, and deploying robotics motion.. <code><a href="https://tldrlegal.com/search?q=Apache-2%20and%20LGPL-3">Apache-2 and LGPL-3</a></code></summary>

- [GitHub](https://github.com/rdiankov/openrave) (👨‍💻 130 · 🔀 340 · 📋 550 - 59% open · ⏱️ 02.10.2026):

	```
	git clone https://github.com/rdiankov/openrave
	```
</details>
<details><summary><b><a href="https://developer.nvidia.com/isaac/sim">NVIDIA Isaac Sim</a></b> (🥈18 ·  ⭐ 4.5K) - NVIDIA Isaac Sim is an open-source application on NVIDIA Omniverse for developing, simulating, and testing AI-driven.. <code><a href="https://tldrlegal.com/search?q=Apache%202.0%20and%20NVIDIA%20Omniverse%20License%20Agreement">Apache 2.0 and NVIDIA Omniverse License Agreement</a></code></summary>

- [GitHub](https://github.com/isaac-sim/IsaacSim) (👨‍💻 6 · 🔀 600 · 📋 550 - 10% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/isaac-sim/IsaacSim
	```
</details>
<details><summary><b><a href="https://github.com/RobotecAI/ros2-for-unity">Ros2 For Unity</a></b> (🥉15 ·  ⭐ 640) - High-performance ROS2 solution for Unity3D. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/RobotecAI/ros2-for-unity) (👨‍💻 13 · 🔀 79 · 📥 8.9K · 📋 99 - 44% open · ⏱️ 06.11.2026):

	```
	git clone https://github.com/RobotecAI/ros2-for-unity
	```
</details>
<details><summary><b><a href="https://www.argos-sim.info/">ARGoS</a></b> (🥉15 ·  ⭐ 320) - A parallel, multi-engine simulator for heterogeneous swarm robotics. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/ilpincy/argos3) (👨‍💻 25 · 🔀 130 · 📋 140 - 23% open · ⏱️ 05.10.2026):

	```
	git clone https://github.com/ilpincy/argos3
	```
</details>
<details><summary><b><a href="http://coppeliarobotics.com/">CoppeliaSim core library</a></b> (🥉15 ·  ⭐ 150) - CoppeliaSim core library. <code><a href="https://tldrlegal.com/search?q=gnu-gpl">gnu-gpl</a></code></summary>

- [GitHub](https://github.com/CoppeliaRobotics/coppeliaSimLib) (👨‍💻 3 · 🔀 42 · 📋 22 - 4% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/CoppeliaRobotics/coppeliaSimLib
	```
</details>
<details><summary><b><a href="https://github.com/code-iai/ROSIntegration">ROS1 Intergration for Unreal 4</a></b> (🥉14 ·  ⭐ 470) - Unreal Engine Plugin to enable ROS Support. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/code-iai/ROSIntegration) (👨‍💻 33 · 🔀 140 · 📋 130 - 41% open · ⏱️ 03.11.2025):

	```
	git clone https://github.com/code-iai/ROSIntegration
	```
</details>
<details><summary><b><a href="https://urlab-sim.github.io/UnrealRoboticsLab/">Unreal Robotics Lab</a></b> (🥉11 ·  ⭐ 580) - A high-fidelity, open-source robotics simulator integrating Unreal Engines photorealistic rendering with MuJoCos.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/URLab-Sim/UnrealRoboticsLab) (👨‍💻 3 · 🔀 67 · 📋 23 - 26% open · ⏱️ 08.08.2026):

	```
	git clone https://github.com/URLab-Sim/UnrealRoboticsLab
	```
</details>
<details><summary><b><a href="https://gears.aposteriori.com.sg/">Gears</a></b> (🥉11 ·  ⭐ 91) - Generic Educational Robotics Simulator. <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/QuirkyCort/gears) (👨‍💻 14 · 🔀 58 · 📋 98 - 22% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/QuirkyCort/gears
	```
</details>
<details><summary><b><a href="http://robwork.dk/">Robwork</a></b> (🥉9 ·  ⭐ 33 · 💤) - RobWork is a collection of C++ libraries for simulation and control of robot systems, see http://robwork.dk To get.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitLab](https://gitlab.com/sdurobotics/RobWork) (🔀 160 · 📋 110 - 15% open · ⏱️ 07.04.2016):

	```
	git clone https://gitlab.com/sdurobotics/RobWork
	```
</details>
<details><summary><b><a href="https://robosimstudio.github.io/">RoboSimStudio</a></b> (🥉8 ·  ⭐ 620 · 🐣) - An open agentic toolkit for physics-based Real2Sim of Robot Manipulation Tasks. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/robosimstudio/robosimstudio) (👨‍💻 2 · 🔀 47 · ⏱️ 13.11.2026):

	```
	git clone https://github.com/robosimstudio/robosimstudio
	```
</details>
<details><summary><b><a href="https://github.com/rlxai/rbot">Rbot</a></b> (🥉7 ·  ⭐ 120 · 🐣) - An open-source end-to-end Autonomous Mobile Robot simulation stack for ROS 2 Jazzy and Gazebo Harmonic. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/rlxai/rbot) (👨‍💻 2 · 🔀 27 · ⏱️ 30.10.2026):

	```
	git clone https://github.com/rlxai/rbot
	```
</details>
<details><summary><b><a href="https://gitlab.com/robocup-sim/SimSpark">SimSpark</a></b> (🥉7 ·  ⭐ 22 · 💤) - A generic physical simulator. <code><a href="https://tldrlegal.com/search?q=Missing">Missing</a></code></summary>

- [GitLab](https://gitlab.com/robocup-sim/SimSpark) (🔀 8 · 📋 57 - 49% open · ⏱️ 13.10.2017):

	```
	git clone https://gitlab.com/robocup-sim/SimSpark
	```
</details>
<details><summary><b><a href="http://coppeliarobotics.com/">CoppeliaSim</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.abb.com/global/en/areas/robotics/products/software/robotstudio-suite">RobotStudio</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://robodk.com/">RoboDK</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.mathworks.com/products/robotics.html">MATLAB Robotics Systems Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.robotec.ai/products">RoSi</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://prototwin.com/">ProtoTwin</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary>Show 4 hidden projects...</summary>

- <b><a href="https://classic.gazebosim.org/">Gazebo Classic</a></b> (🥈20 ·  ⭐ 1.3K · 💤) - Gazebo classic. For the latest version, see https://github.com/gazebosim/gz-sim. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="http://morse-simulator.github.io/">Morse</a></b> (🥈17 ·  ⭐ 370 · 💤) - The Modular OpenRobots Simulation Engine. <code><a href="https://tldrlegal.com/search?q=OFL-1.1">OFL-1.1</a></code>
- <b><a href="https://blog.openai.com/roboschool/">Roboschool</a></b> (🥈16 ·  ⭐ 2.2K · 💤) - DEPRECATED: Open-source software for robot simulation, integrated with OpenAI Gym. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
- <b><a href="https://simbad.sourceforge.net/">Simbad</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=GNU-gpl2">GNU-gpl2</a></code>
</details>
<br>

## Robotic simulators in 2D

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Robotic simulators that only work in a 2D environment, for instance navigation_

<details><summary><b><a href="https://ir-sim.readthedocs.io/en">IR-SIM</a></b> (🥇25 ·  ⭐ 1.2K) - A Python-based lightweight robot simulator designed for navigation, control, and learning. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/hanruihua/ir-sim) (👨‍💻 16 · 🔀 150 · 📦 16 · 📋 53 - 1% open · ⏱️ 12.11.2026):

	```
	git clone https://github.com/hanruihua/ir-sim
	```
- [PyPi](https://pypi.org/project/ir-sim) (📥 1.9K / month · 📦 4 · ⏱️ 11.11.2026):
	```
	pip install ir-sim
	```
</details>
<details><summary><b><a href="https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide">AutonomousVehicleControlBeginnersGuide</a></b> (🥈20 ·  ⭐ 2K) - Learn autonomous driving by building it: Python simulations and an open textbook where the system grows one stage at a.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide) (👨‍💻 32 · 🔀 320 · 📋 100 - 50% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide
	```
</details>
<details><summary><b><a href="https://github.com/MRPT/mvsim">mvsim</a></b> (🥈20 ·  ⭐ 420) - Vehicle and mobile robotics simulator. C++ & Python API. Use it as a standalone application or via ROS 1 or ROS 2. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/MRPT/mvsim) (👨‍💻 9 · 🔀 62 · 📋 41 - 14% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/MRPT/mvsim
	```
</details>
<details><summary><b><a href="https://pyrobosim.readthedocs.io/">pyrobosim</a></b> (🥈20 ·  ⭐ 400) - 2D mobile robot simulator for behavior prototyping with optional ROS 2 interface. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/sea-bass/pyrobosim) (👨‍💻 22 · 🔀 72 · 📦 9 · 📋 130 - 4% open · ⏱️ 02.11.2026):

	```
	git clone https://github.com/sea-bass/pyrobosim
	```
- [PyPi](https://pypi.org/project/pyrobosim) (📥 680 / month · ⏱️ 20.09.2026):
	```
	pip install pyrobosim
	```
</details>
<details><summary><b><a href="https://flatland-simulator.readthedocs.io/">Flatland</a></b> (🥉13 ·  ⭐ 120) - A 2D robot simulator for ROS. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/avidbots/flatland) (👨‍💻 15 · 🔀 65 · 📋 37 - 43% open · ⏱️ 25.09.2026):

	```
	git clone https://github.com/avidbots/flatland
	```
</details>
<details><summary><b><a href="https://github.com/abaeyens/vehicle_dynamics_sim">Vehicle Dynamics Sim</a></b> (🥉5 ·  ⭐ 7) - Simulates a variety of easily configurable bicycle and Ackermann vehicles with dynamic effects such as acceleration.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/abaeyens/vehicle_dynamics_sim) (🔀 1 · ⏱️ 30.11.2025):

	```
	git clone https://github.com/abaeyens/vehicle_dynamics_sim
	```
</details>
<details><summary><b><a href="https://github.com/EricChen0104/DWA_Algorithm_PYTHON">DWA_Algorithm_PYTHON</a></b> (🥉4 ·  ⭐ 18 · 💤) - A fully visualized implementation of the Dynamic Window Approach (DWA) in Python using Pygame. Simulate and visualize.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/EricChen0104/DWA_Algorithm_PYTHON) (👨‍💻 2 · 🔀 2 · ⏱️ 18.07.2025):

	```
	git clone https://github.com/EricChen0104/DWA_Algorithm_PYTHON
	```
</details>
<br>

## Aerial Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for aerial robotics_

<details><summary><b><a href="https://iamaisim.github.io/ProjectAirSim/">Project AirSim</a></b> (🥇22 ·  ⭐ 960 · 📈) - Project AirSim is Microsofts evolution of AirSim, an advanced simulation platform for building, training, and testing.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/iamaisim/ProjectAirSim) (👨‍💻 20 · 🔀 180 · 📥 19K · 📋 70 - 57% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/iamaisim/ProjectAirSim
	```
</details>
<details><summary><b><a href="https://cosys-lab.github.io/Cosys-AirSim/">Cosys-AirSim</a></b> (🥇22 ·  ⭐ 450) - Cosys-AirSim is a simulator for drones, cars, robots and their sensors built on Unreal Engine with an extensive API. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Cosys-Lab/Cosys-AirSim) (👨‍💻 270 · 🔀 150 · 📥 20K · 📋 97 - 50% open · ⏱️ 05.10.2026):

	```
	git clone https://github.com/Cosys-Lab/Cosys-AirSim
	```
</details>
<details><summary><b><a href="https://github.com/JacopoPan/aerial-autonomy-stack">Aerial Autonomy Stack</a></b> (🥇21 ·  ⭐ 670 · 📈) - An open harness to simulate and deploy PX4/ArduPilot/Betaflight perception-based drone swarms with ROS2, YOLO, NVIDIA.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/JacopoPan/aerial-autonomy-stack) (👨‍💻 8 · 🔀 110 · 📥 2K · 📋 26 - 19% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/JacopoPan/aerial-autonomy-stack
	```
</details>
<details><summary><b><a href="https://codexlabsllc.github.io/Colosseum/">Colosseum</a></b> (🥇21 ·  ⭐ 670) - Open source simulator for autonomous robotics built on Unreal Engine with support for Unity. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/CodexLabsLLC/Colosseum) (👨‍💻 260 · 🔀 190 · 📥 20K · 📋 88 - 70% open · ⏱️ 08.12.2025):

	```
	git clone https://github.com/CodexLabsLLC/Colosseum
	```
</details>
<details><summary><b><a href="https://github.com/jjshoots/PyFlyt">PyFlyt</a></b> (🥈20 ·  ⭐ 260) - UAV Flight Simulator for Reinforcement Learning Research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/jjshoots/PyFlyt) (👨‍💻 12 · 🔀 47 · 📦 41 · 📋 62 - 22% open · ⏱️ 06.04.2026):

	```
	git clone https://github.com/jjshoots/PyFlyt
	```
- [PyPi](https://pypi.org/project/pyflyt) (📥 1.6K / month · 📦 2 · ⏱️ 01.03.2025):
	```
	pip install pyflyt
	```
</details>
<details><summary><b><a href="https://learnsyslab.github.io/gym-pybullet-drones/">Gym Pybullet Drones</a></b> (🥈19 ·  ⭐ 2.2K) - PyBullet Gymnasium environments for single and multi-agent reinforcement learning of quadcopter control. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/learnsyslab/gym-pybullet-drones) (👨‍💻 29 · 🔀 550 · 📋 220 - 49% open · ⏱️ 13.11.2026):

	```
	git clone https://github.com/learnsyslab/gym-pybullet-drones
	```
</details>
<details><summary><b><a href="https://github.com/ethz-asl/rotors_simulator">RotorS</a></b> (🥈18 ·  ⭐ 1.5K · 💤) - RotorS is a UAV gazebo simulator. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/ethz-asl/rotors_simulator) (👨‍💻 50 · 🔀 790 · 📋 390 - 44% open · ⏱️ 06.07.2021):

	```
	git clone https://github.com/ethz-asl/rotors_simulator
	```
</details>
<details><summary><b><a href="https://learnsyslab.github.io/crazyflow/">Crazyflow</a></b> (🥈18 ·  ⭐ 200) - Scalable drone simulation using JAX. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/learnsyslab/crazyflow) (👨‍💻 15 · 🔀 48 · 📦 2 · 📋 72 - 20% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/learnsyslab/crazyflow
	```
</details>
<details><summary><b><a href="https://uzh-rpg.github.io/flightmare/">Flightmare</a></b> (🥈17 ·  ⭐ 1.5K · 💤) - An Open Flexible Quadrotor Simulator. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/uzh-rpg/flightmare) (👨‍💻 7 · 🔀 400 · 📥 7.4K · 📋 190 - 67% open · ⏱️ 15.05.2023):

	```
	git clone https://github.com/uzh-rpg/flightmare
	```
</details>
<details><summary><b><a href="https://github.com/gsilano/CrazyS">CrazyS</a></b> (🥈17 ·  ⭐ 180) - CrazyS is an extension of the ROS package RotorS, aimed to modeling, developing and integrating the Crazyflie 2.0. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/gsilano/CrazyS) (👨‍💻 3 · 🔀 85 · 📦 1 · 📋 98 - 10% open · ⏱️ 13.10.2026):

	```
	git clone https://github.com/gsilano/CrazyS
	```
</details>
<details><summary><b><a href="https://pegasussimulator.github.io/PegasusSimulator/">Pegasus Simulator</a></b> (🥈15 ·  ⭐ 920) - A framework built on top of NVIDIA Isaac Sim for simulating drones with PX4 support and much more. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/PegasusSimulator/PegasusSimulator) (👨‍💻 6 · 🔀 210 · 📋 93 - 56% open · ⏱️ 24.07.2026):

	```
	git clone https://github.com/PegasusSimulator/PegasusSimulator
	```
</details>
<details><summary><b><a href="https://flightgoggles.mit.edu/">FlightGoggles</a></b> (🥈15 ·  ⭐ 440 · 💤) - A framework for photorealistic hardware-in-the-loop agile flight simulation using Unity3D and ROS. Developed by MIT.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/mit-aera/FlightGoggles) (👨‍💻 9 · 🔀 99 · 📥 1.7K · 📋 170 - 12% open · ⏱️ 01.04.2024):

	```
	git clone https://github.com/mit-aera/FlightGoggles
	```
</details>
<details><summary><b><a href="https://ntnu-arl.github.io/aerial_gym_simulator/">Aerial Gym Simulator</a></b> (🥉14 ·  ⭐ 780) - Aerial Gym Simulator - Isaac Gym Simulator for Aerial Robots. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/ntnu-arl/aerial_gym_simulator) (👨‍💻 4 · 🔀 120 · 📋 52 - 32% open · ⏱️ 28.06.2026):

	```
	git clone https://github.com/ntnu-arl/aerial_gym_simulator
	```
</details>
<details><summary><b><a href="https://github.com/PX4/jMAVSim">jMAVSim</a></b> (🥉14 ·  ⭐ 110) - Simple multirotor simulator with MAVLink protocol support. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/PX4/jMAVSim) (👨‍💻 29 · 🔀 220 · 📋 52 - 30% open · ⏱️ 29.01.2026):

	```
	git clone https://github.com/PX4/jMAVSim
	```
</details>
<details><summary><b><a href="https://github.com/spencerfolk/rotorpy">rotorpy</a></b> (🥉13 ·  ⭐ 330) - A multirotor simulator with aerodynamics for education and research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/spencerfolk/rotorpy) (👨‍💻 10 · 🔀 63 · 📥 25 · 📋 27 - 25% open · ⏱️ 07.09.2026):

	```
	git clone https://github.com/spencerfolk/rotorpy
	```
</details>
<details><summary><b><a href="http://wfk.io/neuroflight/">Gymfc</a></b> (🥉12 ·  ⭐ 440 · 💤) - A universal flight control tuning framework. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/wil3/gymfc) (👨‍💻 1 · 🔀 100 · 📋 81 - 11% open · ⏱️ 07.10.2021):

	```
	git clone https://github.com/wil3/gymfc/
	```
</details>
<details><summary><b><a href="https://pterolabs.ai/">PteroSim</a></b> (🥉12 ·  ⭐ 140) - UAV simulator on Unreal Engine 5 with JSBSim flight dynamics, PX4 and ArduPilot SITL, and a gRPC API with a Python SDK.. <code><a href="https://tldrlegal.com/search?q=proprietary">proprietary</a></code></summary>

- [GitHub](https://github.com/PteroLabsAI/PteroSim-UAV-Simulator) (🔀 26 · 📥 260 · ⏱️ 08.11.2026):

	```
	git clone https://github.com/PteroLabsAI/PteroSim-UAV-Simulator
	```
</details>
<details><summary><b><a href="https://shupx.github.io/FastSwarmSim/">FastSwarmSim</a></b> (🥉12 ·  ⭐ 49 · 🐣) - Lightweight, faster-than-real-time multi-UAV PX4 swarm simulator with ROS 2 lock-step simulation. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/shupx/FastSwarmSim) (🔀 8 · ⏱️ 10.10.2026):

	```
	git clone https://github.com/shupx/FastSwarmSim
	```
</details>
<details><summary><b><a href="https://github.com/GongXudong/fly-craft">Fly Craft</a></b> (🥉11 ·  ⭐ 120) - An efficient goal-conditioned reinforcement learning environment for fixed-wing UAV velocity vector control based on.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/GongXudong/fly-craft) (👨‍💻 1 · 🔀 6 · 📦 4 · ⏱️ 25.11.2025):

	```
	git clone https://github.com/GongXudong/fly-craft
	```
- [PyPi](https://pypi.org/project/flycraft) (📥 69 / month · ⏱️ 30.12.2025):
	```
	pip install flycraft
	```
</details>
<details><summary><b><a href="https://github.com/kousheekc/isaac_drone_racer">Isaac Drone Racer</a></b> (🥉10 ·  ⭐ 200) - Isaac Drone Racer is a reinforcement learning framework for high speed autonomous drone racing, built on top of.. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/kousheekc/isaac_drone_racer) (🔀 44 · ⏱️ 04.12.2025):

	```
	git clone https://github.com/kousheekc/isaac_drone_racer
	```
</details>
<details><summary><b><a href="https://github.com/ctu-mrs/mrs_uav_flightforge_simulator">FlightForge</a></b> (🥉10 ·  ⭐ 55) - ROS counterpart for the FlightForge UAV simulator. <code><a href="https://tldrlegal.com/search?q=GLP-2.0">GLP-2.0</a></code></summary>

- [GitHub](https://github.com/ctu-mrs/mrs_uav_flightforge_simulator) (👨‍💻 11 · 🔀 10 · 📋 4 - 25% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/ctu-mrs/mrs_uav_flightforge_simulator
	```
</details>
<details><summary><b><a href="https://www.flightgear.org/">Flightgear</a></b> (🥉9 ·  ⭐ 81 · 💤) - FlightGear open-source flight simulator [flightgear.org](https://www.flightgear.org). <code><a href="https://tldrlegal.com/search?q=gnu-gpl2">gnu-gpl2</a></code></summary>

- [GitLab](https://gitlab.com/flightgear/flightgear) (🔀 180 · 📋 690 - 36% open · ⏱️ 04.03.2015):

	```
	git clone https://gitlab.com/flightgear/flightgear
	```
</details>
<details><summary><b><a href="https://github.com/shupx/swarm_sync_sim">swarm_sync_sim</a></b> (🥉8 ·  ⭐ 26 · 💤) - swarm_sync_sim (also known as sss) is an ultra-lightweight, ROS-based simulator for robotic swarms, including.. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/shupx/swarm_sync_sim) (👨‍💻 3 · 🔀 4 · ⏱️ 18.03.2025):

	```
	git clone https://github.com/shupx/swarm_sync_sim
	```
</details>
<details><summary><b><a href="https://optim.aero/px4silsimulink.html">optimAero PX4</a></b> (🥉7 ·  ⭐ 22) - PX4 Simulink Software-In-Loop Simulation | For the public domain (F16 + Hexarotor). <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/optimAero/optimAeroPX4SIL) (🔀 9 · 📋 19 - 21% open · ⏱️ 13.02.2026):

	```
	git clone https://github.com/optimAero/optimAeroPX4SIL
	```
</details>
<details><summary><b><a href="https://github.com/aau-cns/Ardupilot_Multiagent_Simulation">Ardupilot_Multiagent_Simulation</a></b> (🥉6 ·  ⭐ 120) - Simulation environment for multiagent drone systems using Ardupilot, ROS 2, and Gazebo enabling users to spawn and.. <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/aau-cns/Ardupilot_Multiagent_Simulation) (🔀 29 · 📋 6 - 66% open · ⏱️ 31.03.2026):

	```
	git clone https://github.com/aau-cns/Ardupilot_Multiagent_Simulation
	```
</details>
<details><summary><b><a href="https://github.com/arplaboratory/RotorTM">RotorTM</a></b> (🥉6 ·  ⭐ 97 · 💤) -  <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/arplaboratory/RotorTM) (👨‍💻 3 · 🔀 15 · 📋 5 - 60% open · ⏱️ 09.06.2024):

	```
	git clone https://github.com/arplaboratory/RotorTM
	```
</details>
<details><summary><b><a href="https://github.com/gustavo-moura/itomori">Itomori</a></b> (🥉4 ·  ⭐ 13 · 💤) - Gymnasium environment for research of UAVs and risk constraints. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/gustavo-moura/itomori) (⏱️ 29.10.2024):

	```
	git clone https://github.com/gustavo-moura/itomori
	```
</details>
<details><summary><b><a href="https://github.com/cvzone/pysimverse">pysimverse</a></b> (🥉2 ·  ⭐ 11 · 💤) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/cvzone/pysimverse) (👨‍💻 2 · 🔀 2 · ⏱️ 05.04.2025):

	```
	git clone https://github.com/cvzone/pysimverse
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/uav.html">Matlab UAV Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=MathWorks%20Software%20License%20Agreement">MathWorks Software License Agreement</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.realflight.com/">Realflight</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20software%20license">proprietary software license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.x-plane.com/">X-plane</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://spleenlab.com/">Spleenlab simulator</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary">proprietary</a></code></summary>

- _No project information available._</details>
<details><summary>Show 1 hidden projects...</summary>

- <b><a href="https://github.com/microsoft/AirSim">airsim</a></b> (🥇32 ·  ⭐ 19K) - Open source simulator for autonomous vehicles built on Unreal Engine / Unity, from Microsoft AI & Research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Maritime Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for maritime robotics_

<details><summary><b><a href="https://github.com/osrf/vrx">Virtual RobotX</a></b> (🥇18 ·  ⭐ 770) - Virtual RobotX (VRX) resources. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/osrf/vrx) (👨‍💻 38 · 🔀 290 · 📋 610 - 11% open · ⏱️ 26.01.2026):

	```
	git clone https://github.com/osrf/vrx
	```
</details>
<details><summary><b><a href="https://umfieldrobotics.github.io/OceanSim/">Ocean Sim</a></b> (🥇15 ·  ⭐ 510) - [IROS 2025] OceanSim: A GPU-Accelerated Underwater Robot Perception Simulation Framework. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/umfieldrobotics/OceanSim) (👨‍💻 11 · 🔀 76 · 📋 31 - 22% open · ⏱️ 04.10.2026):

	```
	git clone https://github.com/umfieldrobotics/OceanSim
	```
</details>
<details><summary><b><a href="https://field-robotics-lab.github.io/dave.doc/">DAVE</a></b> (🥇15 ·  ⭐ 300 · 💤) - Project DAVE. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Field-Robotics-Lab/dave) (👨‍💻 25 · 🔀 81 · 📋 130 - 26% open · ⏱️ 04.12.2023):

	```
	git clone https://github.com/Field-Robotics-Lab/dave
	```
</details>
<details><summary><b><a href="https://stonefish.readthedocs.io/">Stonefish</a></b> (🥈14 ·  ⭐ 340) - Stonefish - an advanced C++ simulation library designed for (but not limited to) marine robotics. <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/patrykcieslak/stonefish) (👨‍💻 13 · 🔀 79 · 📋 51 - 19% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/patrykcieslak/stonefish
	```
</details>
<details><summary><b><a href="https://github.com/open-airlab/UNav-Sim">UNav-Sim</a></b> (🥈14 ·  ⭐ 340 · 💤) - Visually Realistic Underwater Robotics Simulator UNav-Sim. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/open-airlab/UNav-Sim) (👨‍💻 250 · 🔀 32 · 📋 18 - 27% open · ⏱️ 02.05.2025):

	```
	git clone https://github.com/open-airlab/UNav-Sim
	```
</details>
<details><summary><b><a href="https://byu-holoocean.github.io/holoocean-docs/">HoloOcean</a></b> (🥈14 ·  ⭐ 160) - A UE5 based simulator for marine perception and autonomy, with multi-agent communications and many common underwater.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Unreal%20Engine%20EULA">MIT and Unreal Engine EULA</a></code></summary>

- [GitHub]() (👨‍💻 11 · 🔀 83 · 📋 210 - 7% open · ⏱️ 03.11.2026):

	```
	git clone https://github.com/byu-holoocean/HoloOcean
	```
</details>
<details><summary><b><a href="https://github.com/moos-ivp/moos-ivp">Moos-ivp</a></b> (🥉13 ·  ⭐ 100) - MOOS-IvP is a set of modules for providing autonomy on robotic platforms, in particular autonomous marine vehicles. <code><a href="https://tldrlegal.com/search?q=GPLv3%20LGPLv3%20and%20Commercial">GPLv3 LGPLv3 and Commercial</a></code></summary>

- [GitHub](https://github.com/moos-ivp/moos-ivp) (👨‍💻 19 · 🔀 43 · 📋 16 - 37% open · ⏱️ 15.11.2026):

	```
	git clone https://github.com/moos-ivp/moos-ivp
	```
</details>
<details><summary><b><a href="https://github.com/smarc-project/smarc2">SMaRC 2</a></b> (🥉13 ·  ⭐ 22) - smarc ros2-humble main repository. <code><a href="https://tldrlegal.com/search?q=BSD-3-Clause%20and%20MIT">BSD-3-Clause and MIT</a></code></summary>

- [GitHub](https://github.com/smarc-project/smarc2) (👨‍💻 35 · 🔀 46 · 📥 48 · 📋 32 - 3% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/smarc-project/smarc2
	```
</details>
<details><summary><b><a href="https://github.com/MARUSimulator/marus-core">MARUSimulator</a></b> (🥉7 ·  ⭐ 31) - Marine simulator core assets for Unity. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/MARUSimulator/marus-core) (👨‍💻 8 · 🔀 8 · 📋 5 - 40% open · ⏱️ 05.03.2026):

	```
	git clone https://github.com/MARUSimulator/marus-core
	```
</details>
<details><summary><b><a href="http://dave-ros2.notion.site">DAVE for ROS 2</a></b> (🥉2) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub]():

	```
	git clone https://github.com/OES-Lab/dave
	```
</details>
<details><summary>Show 10 hidden projects...</summary>

- <b><a href="https://uuvsimulator.github.io/">UUV Simulator</a></b> (🥇15 ·  ⭐ 890 · 💤) - Gazebo/ROS packages for underwater robotics simulation. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://www.irs.uji.es/uwsim/wiki/index.php?title=Installing_UWSim">UWSim</a></b> (🥉12 ·  ⭐ 110 · 💤) - UWSim-NET, the Underwater Simulator. <code><a href="https://tldrlegal.com/search?q=GNU-gpl">GNU-gpl</a></code>
- <b><a href="https://github.com/uji-ros-pkg/underwater_simulation">UWSim-NET</a></b> (🥉11 ·  ⭐ 110 · 💤) - UWSim-NET, the Underwater Simulator. <code>Unlicensed</code>
- <b><a href="https://github.com/osrf/lrauv">LRAUV</a></b> (🥉10 ·  ⭐ 80 · 💤) - Packages for simulating the Tethys-class Long-Range AUV (LRAUV) from the Monterey Bay Aquarium Research Institute.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://github.com/freefloating-gazebo/freefloating_gazebo">Freefloating</a></b> (🥉8 ·  ⭐ 76 · 💤) - A Gazebo plugin to simulate underwater vehicles and visualize with UWsim. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
- <b><a href="https://github.com/srmauvsoftware/URSim">URSim</a></b> (🥉8 ·  ⭐ 68 · 💤) - Simulator for Unmanned Underwater Vehicles using ROS and Unity3D. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://github.com/iti-luebeck/MARS">MARS</a></b> (🥉7 ·  ⭐ 12 · 💤) - MArine Robotics Simulator - An online Hardware-in-the-Loop simulation environment for multiple AUVs and ASVs. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code>
- <b><a href="https://bitbucket.org/whoidsl/ds_sim/src/master/">ds sim</a></b> -  <code>Unlicensed</code>
- <b><a href="https://sourceforge.net/projects/usarsim/">USARSim</a></b> -  <code>Unlicensed</code>
- <b><a href="https://github.com/eirikhex/UW-MORSE">UW-Morse</a></b> (🥉-1 ·  ⭐ 2 · 💤) -  <code>Unlicensed</code>
</details>
<br>

## Automotive Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for automotive_

<details><summary><b><a href="https://carla.org/">Carla</a></b> (🥇29 ·  ⭐ 15K) - Open-source simulator for autonomous driving research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/carla-simulator/carla) (👨‍💻 220 · 🔀 4.6K · 📋 6.7K - 17% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/carla-simulator/carla
	```
</details>
<details><summary><b><a href="https://github.com/esmini/esmini">ESMINI</a></b> (🥈25 ·  ⭐ 950) - a basic OpenSCENARIO player. <code><a href="http://bit.ly/3postzC">MPL-2.0</a></code></summary>

- [GitHub](https://github.com/esmini/esmini) (👨‍💻 97 · 🔀 260 · 📥 74K · 📋 640 - 31% open · ⏱️ 05.11.2026):

	```
	git clone https://github.com/esmini/esmini
	```
</details>
<details><summary><b><a href="https://github.com/autowarefoundation/AWSIM">AWSim</a></b> (🥉20 ·  ⭐ 770) - Open sourced digital twin simulator for Autoware. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/autowarefoundation/AWSIM) (👨‍💻 36 · 🔀 130 · 📥 92K · 📋 140 - 25% open · ⏱️ 20.10.2026):

	```
	git clone https://github.com/tier4/AWSIM
	```
</details>
<details><summary>Show 2 hidden projects...</summary>

- <b><a href="https://github.com/lgsvl/simulator">SVL Simulator</a></b> (🥉20 ·  ⭐ 2.5K · 💤) - A ROS/ROS2 Multi-robot Simulator for Autonomous Vehicles. <code>Unlicensed</code>
- <b><a href="https://github.com/udacity/self-driving-car-sim">Self Driving Car</a></b> (🥉16 ·  ⭐ 4K · 💤) - A self-driving car simulator built with Unity. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Space Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for space robotics_

<details><summary><b><a href="https://github.com/AVSLab/basilisk">Basilisk</a></b> (🥇24 ·  ⭐ 430) - Astrodynamics simulation framework. <code><a href="http://bit.ly/3hkKRql">ISC</a></code></summary>

- [GitHub](https://github.com/AVSLab/basilisk) (👨‍💻 130 · 🔀 150 · 📋 600 - 4% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/AVSLab/basilisk
	```
</details>
<details><summary><b><a href="https://github.com/nasa/astrobee">Astrobee</a></b> (🥈19 ·  ⭐ 1.4K) - NASA Astrobee Robot Software. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/nasa/astrobee) (👨‍💻 22 · 🔀 370 · 📋 220 - 11% open · ⏱️ 02.12.2025):

	```
	git clone https://github.com/NASA/astrobee
	```
</details>
<details><summary><b><a href="https://github.com/OmniLRS/OmniLRS/wiki">OmiLRS</a></b> (🥈18 ·  ⭐ 280) - Omniverse Lunar Robotics Simulator. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/OmniLRS/OmniLRS) (👨‍💻 24 · 🔀 58 · 📋 47 - 29% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/OmniLRS/OmniLRS
	```
</details>
<details><summary><b><a href="https://avslab.github.io/bsk_rl/">BSK-RL</a></b> (🥉16 ·  ⭐ 130) - RL environments and tools for spacecraft autonomy research, built on Basilisk. Developed by the AVS Lab. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/AVSLab/bsk_rl) (👨‍💻 13 · 🔀 20 · 📋 180 - 14% open · ⏱️ 09.11.2026):

	```
	git clone https://github.com/AVSLab/bsk_rl
	```
</details>
<details><summary><b><a href="https://github.com/dimikout3/MarsExplorer">Mars Explorer</a></b> (🥉6 ·  ⭐ 81 · 💤) -  <code><a href="https://tldrlegal.com/search?q=no%20license">no license</a></code></summary>

- [GitHub](https://github.com/dimikout3/MarsExplorer) (👨‍💻 3 · 🔀 8 · 📋 10 - 40% open · ⏱️ 23.08.2022):

	```
	git clone https://github.com/dimikout3/MarsExplorer
	```
</details>
<details><summary><b><a href="https://github.com/PUTvision/LunarSim">LunarSim</a></b> (🥉5 ·  ⭐ 86 · 💤) - LunarSim: Lunar Rover Simulator Focused on High Visual Fidelity and ROS 2 Integration for Advanced Computer Vision.. <code><a href="https://tldrlegal.com/search?q=Missing">Missing</a></code></summary>

- [GitHub](https://github.com/PUTvision/LunarSim) (👨‍💻 2 · 🔀 11 · 📥 720 · 📋 8 - 75% open · ⏱️ 07.12.2023):

	```
	git clone https://github.com/PUTvision/LunarSim
	```
</details>
<br>

## AI training Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulations made for training for AI-agents like reinforcement learning_

<details><summary><b><a href="https://gymnasium.farama.org/">Gymnasium</a></b> (🥇41 ·  ⭐ 13K) - A standard API for single-agent reinforcement learning environments, with popular reference environments and related.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Farama-Foundation/Gymnasium) (👨‍💻 640 · 🔀 1.5K · 📦 24K · 📋 690 - 18% open · ⏱️ 12.11.2026):

	```
	git clone https://github.com/Farama-Foundation/Gymnasium
	```
- [PyPi](https://pypi.org/project/gymnasium) (📥 4.4M / month · 📦 2.1K · ⏱️ 05.10.2026):
	```
	pip install gymnasium
	```
</details>
<details><summary><b><a href="https://github.com/Genesis-Embodied-AI/genesis-world">Genesis</a></b> (🥇31 ·  ⭐ 30K) - Simulation platform for general-purpose robotics & embodied AI learning. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Genesis-Embodied-AI/genesis-world) (👨‍💻 120 · 🔀 2.9K · 📦 140 · 📋 1.2K - 10% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/Genesis-Embodied-AI/Genesis
	```
</details>
<details><summary><b><a href="https://isaac-sim.github.io/IsaacLab">NVIDIA Isaac Sim Isaac Lab</a></b> (🥇31 ·  ⭐ 8.7K) - Unified framework for robot learning with multi-physics/renderer support. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/isaac-sim/IsaacLab) (👨‍💻 300 · 🔀 3.9K · 📥 1.9K · 📋 2.8K - 7% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/isaac-sim/IsaacLab
	```
</details>
<details><summary><b><a href="https://github.com/mujocolab/mjlab">mjlab</a></b> (🥇31 ·  ⭐ 3.4K) - Isaac Lab API, powered by MuJoCo-Warp, for RL and robotics research. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/mujocolab/mjlab) (👨‍💻 67 · 🔀 590 · 📋 370 - 17% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/mujocolab/mjlab
	```
- [PyPi](https://pypi.org/project/mjlab) (📥 130K / month · 📦 28 · ⏱️ 15.11.2026):
	```
	pip install mjlab
	```
</details>
<details><summary><b><a href="https://robotics.farama.org/">Gymnasium Robotics</a></b> (🥈29 ·  ⭐ 2K) - A collection of robotics simulation environments for reinforcement learning. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Farama-Foundation/Gymnasium-Robotics) (👨‍💻 47 · 🔀 160 · 📦 360 · 📋 110 - 24% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/Farama-Foundation/Gymnasium-Robotics
	```
- [PyPi](https://pypi.org/project/gymnasium-robotics) (📥 90K / month · 📦 21 · ⏱️ 02.01.2026):
	```
	pip install gymnasium-robotics
	```
</details>
<details><summary><b><a href="https://metaworld.farama.org/">Metaworld</a></b> (🥈28 ·  ⭐ 1.9K) - Collections of robotics environments geared towards benchmarking multi-task and meta reinforcement learning. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Farama-Foundation/Metaworld) (👨‍💻 48 · 🔀 340 · 📦 130 · 📋 250 - 3% open · ⏱️ 12.11.2026):

	```
	git clone https://github.com/Farama-Foundation/Metaworld
	```
- [PyPi](https://pypi.org/project/metaworld) (📥 36K / month · 📦 7 · ⏱️ 28.06.2026):
	```
	pip install metaworld
	```
</details>
<details><summary><b><a href="https://ai2thor.allenai.org/">AI2-thor</a></b> (🥈24 ·  ⭐ 1.8K · 💤) - An open-source platform for Visual AI. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/allenai/ai2thor) (👨‍💻 53 · 🔀 290 · 📦 400 · 📋 650 - 42% open · ⏱️ 29.05.2025):

	```
	git clone https://github.com/allenai/ai2thor
	```
</details>
<details><summary><b><a href="https://playground.mujoco.org/">MuJoCo playground</a></b> (🥈22 ·  ⭐ 2.3K) - An open-source library for sim and real robot demos on top of MuJoCo. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google-deepmind/mujoco_playground) (👨‍💻 38 · 🔀 380 · 📋 210 - 42% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/google-deepmind/mujoco_playground/
	```
</details>
<details><summary><b><a href="https://metadriverse.github.io/metadrive-simulator/">MetaDrive</a></b> (🥈21 ·  ⭐ 1.3K) - MetaDrive: Lightweight driving simulator for everyone. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/metadriverse/metadrive) (👨‍💻 21 · 🔀 200 · 📥 120K · 📋 420 - 24% open · ⏱️ 16.10.2026):

	```
	git clone https://github.com/metadriverse/metadrive/
	```
</details>
<details><summary><b><a href="https://svl.stanford.edu/igibson/">IGibson</a></b> (🥈21 ·  ⭐ 820 · 💤) - A Simulation Environment to train Robots in Large Realistic Interactive Scenes. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/StanfordVL/iGibson) (👨‍💻 35 · 🔀 170 · 📥 500 · 📦 14 · 📋 350 - 39% open · ⏱️ 19.02.2023):

	```
	git clone https://github.com/StanfordVL/iGibson
	```
</details>
<details><summary><b><a href="https://jaxsim.readthedocs.io">jaxsim</a></b> (🥈21 ·  ⭐ 210) - A differentiable physics engine and multibody dynamics library for control and robot learning. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/gbionics/jaxsim) (👨‍💻 16 · 🔀 22 · 📦 4 · 📋 79 - 8% open · ⏱️ 02.11.2026):

	```
	git clone https://github.com/ami-iit/jaxsim
	```
- [PyPi](https://pypi.org/project/jaxsim) (📥 3.6K / month · ⏱️ 03.06.2026):
	```
	pip install jaxsim
	```
- [Conda](https://anaconda.org/conda-forge/jaxsim) (📥 23K · ⏱️ 29.04.2026):
	```
	conda install -c conda-forge jaxsim
	```
</details>
<details><summary><b><a href="https://aihabitat.org/">Habitat Sim</a></b> (🥈20 ·  ⭐ 3.8K) - A flexible, high-performance 3D simulator for Embodied AI research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/facebookresearch/habitat-sim) (👨‍💻 64 · 🔀 540 · 📋 850 - 24% open · ⏱️ 07.05.2026):

	```
	git clone https://github.com/facebookresearch/habitat-sim
	```
</details>
<details><summary><b><a href="https://maniskill.ai/">ManiSkill</a></b> (🥈19 ·  ⭐ 3.5K) - Manipulation Skill Framework, an open source GPU parallelized robotics simulator and benchmark. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/mani-skill/ManiSkill) (👨‍💻 70 · 🔀 540 · 📋 830 - 15% open · ⏱️ 02.08.2026):

	```
	git clone https://github.com/haosulab/ManiSkill
	```
</details>
<details><summary><b><a href="https://roboverseorg.github.io/">RoboVerse</a></b> (🥈19 ·  ⭐ 1.9K) - RoboVerse: Towards a Unified Platform, Dataset and Benchmark for Scalable and Generalizable Robot Learning. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/RoboVerseOrg/RoboVerse) (👨‍💻 51 · 🔀 170 · 📥 6 · 📋 180 - 31% open · ⏱️ 08.09.2026):

	```
	git clone https://github.com/RoboVerseOrg/RoboVerse
	```
</details>
<details><summary><b><a href="https://loco-mujoco.readthedocs.io/">LocoMuJoCo</a></b> (🥈19 ·  ⭐ 1.5K) - Imitation learning benchmark focusing on complex locomotion tasks using MuJoCo. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/robfiras/loco-mujoco) (👨‍💻 16 · 🔀 140 · 📦 8 · 📋 70 - 45% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/robfiras/loco-mujoco
	```
</details>
<details><summary><b><a href="https://sapien.ucsd.edu/">Sapien</a></b> (🥈19 ·  ⭐ 870) - SAPIEN Embodied AI Platform. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/haosulab/SAPIEN) (👨‍💻 20 · 🔀 82 · 📥 3.7K · 📦 270 · 📋 240 - 36% open · ⏱️ 08.10.2026):

	```
	git clone https://github.com/haosulab/SAPIEN
	```
</details>
<details><summary><b><a href="https://github.com/duburcqa/jiminy">Jiminy</a></b> (🥉18 ·  ⭐ 300 · 💤) - Jiminy: a fast and portable Python/C++ simulator of poly-articulated robots with OpenAI Gym interface for.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/duburcqa/jiminy) (👨‍💻 9 · 🔀 30 · 📋 130 - 28% open · ⏱️ 01.05.2025):

	```
	git clone https://github.com/duburcqa/jiminy
	```
- [PyPi](https://pypi.org/project/jiminy-py) (📥 2.6K / month · 📦 2 · ⏱️ 01.05.2025):
	```
	pip install jiminy-py
	```
</details>
<details><summary><b><a href="https://github.com/BYU-PCCL/holodeck">Holodeck</a></b> (🥉17 ·  ⭐ 600 · 💤) - High Fidelity Simulator for Reinforcement Learning and Robotics Research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/BYU-PCCL/holodeck) (👨‍💻 20 · 🔀 42 · 📦 10 · 📋 250 - 20% open · ⏱️ 30.04.2021):

	```
	git clone https://github.com/BYU-PCCL/holodeck
	```
</details>
<details><summary><b><a href="https://robocasa.ai/">RoboCasa</a></b> (🥉16 ·  ⭐ 1.9K) - RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/robocasa/robocasa) (👨‍💻 6 · 🔀 250 · 📋 200 - 28% open · ⏱️ 12.11.2026):

	```
	git clone https://github.com/robocasa/robocasa
	```
</details>
<details><summary><b><a href="http://gibsonenv.stanford.edu/">Gibson</a></b> (🥉16 ·  ⭐ 950) - Gibson Environments: Real-World Perception for Embodied Agents. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/StanfordVL/GibsonEnv) (👨‍💻 9 · 🔀 150 · 📋 120 - 40% open · ⏱️ 20.10.2026):

	```
	git clone https://github.com/StanfordVL/GibsonEnv
	```
</details>
<details><summary><b><a href="https://docs.kscale.dev/docs/ksim">K-Sim</a></b> (🥉16 ·  ⭐ 230 · 💤) - RL training library for humanoid locomotion and manipulation. Built on top of MuJoCo and JAX. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/kscalelabs/ksim) (👨‍💻 9 · 🔀 40 · 📋 33 - 27% open · ⏱️ 29.10.2025):

	```
	git clone https://github.com/kscalelabs/ksim
	```
- [PyPi](https://pypi.org/project/ksim) (📥 200 / month · ⏱️ 18.08.2025):
	```
	pip install ksim
	```
</details>
<details><summary><b><a href="https://www.dynsyslab.org/safe-robot-learning/">Safe Control Gym</a></b> (🥉15 ·  ⭐ 940) - PyBullet CartPole and Quadrotor environmentswith CasADi symbolic a priori dynamicsfor learning-based control and RL. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/learnsyslab/safe-control-gym) (👨‍💻 20 · 🔀 160 · 📋 62 - 9% open · ⏱️ 29.04.2026):

	```
	git clone https://github.com/utiasDSL/safe-control-gym
	```
</details>
<details><summary><b><a href="https://github.com/stepjam/PyRep">PyRep</a></b> (🥉15 ·  ⭐ 760 · 💤) - A toolkit for robot learning research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/stepjam/PyRep) (👨‍💻 20 · 🔀 170 · 📋 320 - 8% open · ⏱️ 02.07.2024):

	```
	git clone https://github.com/stepjam/PyRep
	```
</details>
<details><summary><b><a href="https://procthor.allenai.org/">ProcTHOR</a></b> (🥉14 ·  ⭐ 490 · 💤) - Scaling Embodied AI by Procedurally Generating Interactive 3D Houses. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/allenai/procthor) (👨‍💻 7 · 🔀 52 · 📦 14 · 📋 54 - 83% open · ⏱️ 14.12.2022):

	```
	git clone https://github.com/allenai/procthor
	```
</details>
<details><summary><b><a href="https://github.com/jr-robotics/robo-gym">robo-gym</a></b> (🥉14 ·  ⭐ 480 · 💤) - An open source toolkit for Distributed Deep Reinforcement Learning on real and simulated robots. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/jr-robotics/robo-gym) (👨‍💻 11 · 🔀 77 · 📦 10 · 📋 55 - 21% open · ⏱️ 22.07.2025):

	```
	git clone https://github.com/jr-robotics/robo-gym
	```
</details>
<details><summary><b><a href="https://unrealzoo.site/">UnrealZoo</a></b> (🥉13 ·  ⭐ 380) - [ICCV 2025 Highlights] Large-scale photo-realistic virtual worlds for embodied AI. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/UnrealZoo/unrealzoo-gym) (👨‍💻 6 · 🔀 32 · ⏱️ 17.10.2026):

	```
	git clone https://github.com/UnrealZoo/unrealzoo-gym
	```
</details>
<details><summary><b><a href="https://alloy.argonrobotics.ai/">Alloy</a></b> (🥉13 ·  ⭐ 60 · ➕) - GPU-parallel physics engine for robot learning that puts robots, cloth, cables, liquids, sand and soft bodies in one.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/argonrobotics-ai/alloy) (🔀 4 · ⏱️ 17.11.2026):

	```
	git clone https://github.com/argonrobotics-ai/alloy
	```
- [PyPi](https://pypi.org/project/alloy-engine) (📥 1.8K / month · ⏱️ 18.11.2026):
	```
	pip install alloy-engine
	```
</details>
<details><summary><b><a href="https://github.com/axelbr/racecar_gym/">racecar_gym</a></b> (🥉12 ·  ⭐ 240 · 💤) - A gym environment for a miniature racecar using the pybullet physics engine. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/axelbr/racecar_gym) (🔀 40 · 📥 7K · ⏱️ 18.09.2023):

	```
	git clone https://github.com/axelbr/racecar_gym/
	```
</details>
<details><summary><b><a href="https://github.com/isaac-sim/OmniIsaacGymEnvs">OmniIsaacGymEnvs</a></b> (🥉11 ·  ⭐ 1K · 💤) - Reinforcement Learning Environments for Omniverse Isaac Gym. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/isaac-sim/OmniIsaacGymEnvs) (👨‍💻 6 · 🔀 220 · 📋 170 - 69% open · ⏱️ 06.06.2024):

	```
	git clone https://github.com/isaac-sim/OmniIsaacGymEnvs
	```
</details>
<details><summary><b><a href="https://deepdrive.io/">Deepdrive</a></b> (🥉11 ·  ⭐ 930 · 💤) - Deepdrive is a simulator that allows anyone with a PC to push the state-of-the-art in self-driving. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/deepdrive/deepdrive) (👨‍💻 6 · 🔀 150 · 📋 69 - 57% open · ⏱️ 29.06.2020):

	```
	git clone https://github.com/deepdrive/deepdrive
	```
</details>
<details><summary><b><a href="https://pybullet.org/">PyBullet Gym</a></b> (🥉11 ·  ⭐ 890 · 💤) - Open-source implementations of OpenAI Gym MuJoCo environments for use with the OpenAI Gym Reinforcement Learning.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/benelot/pybullet-gym) (👨‍💻 14 · 🔀 120 · 📋 62 - 56% open · ⏱️ 29.03.2021):

	```
	git clone https://github.com/benelot/pybullet-gym
	```
</details>
<details><summary><b><a href="https://arnold-benchmark.github.io/">Arnold</a></b> (🥉9 ·  ⭐ 190 · 💤) - [ICCV 2023] ARNOLD: Language-Grounded Robot Manipulation with Continuous Object States in Realistic 3D Scenes. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/arnold-benchmark/arnold) (👨‍💻 3 · 🔀 16 · 📋 56 - 14% open · ⏱️ 01.04.2024):

	```
	git clone https://github.com/arnold-benchmark/arnold
	```
</details>
<details><summary><b><a href="https://some45bucks.github.io/IsaacLab-HARL/">IsaacLab-HARL</a></b> (🥉9 ·  ⭐ 9 · 💤) - Unified framework for robot learning built on NVIDIA Isaac Sim. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/some45bucks/IsaacLab-HARL) (👨‍💻 120 · 🔀 1 · ⏱️ 07.08.2025):

	```
	git clone https://github.com/some45bucks/IsaacLab-HARL
	```
</details>
<details><summary><b><a href="https://uaibot.github.io/">UAIbot</a></b> (🥉9 ·  ⭐ 7 · 💤) - The Python version of the UAIbot simulator. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/UAIbot/UAIbotPy) (👨‍💻 3 · 🔀 6 · 📥 4 · ⏱️ 08.07.2025):

	```
	git clone https://github.com/UAIbot/UAIbotPy
	```
</details>
<details><summary><b><a href="https://github.com/WhymustIhaveaname/elastra">Elastra</a></b> (🥉4 ·  ⭐ 1 · 🐣) - MuJoCo models of elastic surfaces, such as mattresses and trampolines, with code to train and test robot policies on.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/WhymustIhaveaname/elastra) (👨‍💻 2 · 🔀 1 · ⏱️ 11.11.2026):

	```
	git clone https://github.com/WhymustIhaveaname/elastra
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/reinforcement-learning.html">Reinforcement Learning Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary>Show 1 hidden projects...</summary>

- <b><a href="https://www.gymlibrary.dev/">Gym</a></b> (🥈27 ·  ⭐ 37K) - A toolkit for developing and comparing reinforcement learning algorithms. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Other Domain Specific Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Robotic simulators build for other domains like automotive or space robotics_

<details><summary><b><a href="https://github.com/qgallouedec/panda-gym">Panda-Gym</a></b> (🥇14 ·  ⭐ 770 · 💤) - Set of robotic environments based on PyBullet physics engine and gymnasium. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/qgallouedec/panda-gym) (👨‍💻 9 · 🔀 140 · 📋 74 - 13% open · ⏱️ 04.07.2024):

	```
	git clone https://github.com/qgallouedec/panda-gym
	```
</details>
<details><summary><b><a href="https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator">AutoDRIVE Simulator</a></b> (🥇14 ·  ⭐ 160 · 💤) - An Integrated Cyber-Physical Ecosystem for Autonomous Driving Research and Education. <code><a href="http://bit.ly/3rqEWVr">BSD-2</a></code></summary>

- [GitHub](https://github.com/Tinker-Twins/AutoDRIVE) (👨‍💻 3 · 🔀 44 · 📥 3.4K · ⏱️ 23.12.2024):

	```
	git clone https://github.com/Tinker-Twins/AutoDRIVE
	```
</details>
<details><summary><b><a href="https://github.com/skim0119/gym-softrobot">gym-softrobot</a></b> (🥇14 ·  ⭐ 46) - Softrobotics environment package for OpenAI Gymnasium. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/skim0119/gym-softrobot) (👨‍💻 3 · 🔀 9 · 📦 2 · 📋 8 - 25% open · ⏱️ 22.10.2026):

	```
	git clone https://github.com/skim0119/gym-softrobot/
	```
- [PyPi](https://pypi.org/project/gym-softrobot) (📥 120 / month · ⏱️ 21.09.2026):
	```
	pip install gym-softrobot
	```
</details>
<details><summary><b><a href="https://github.com/PKU-MARL/DexterousHands">DexterousHands</a></b> (🥈13 ·  ⭐ 1.1K · 💤) - This is a library that provides dual dexterous hand manipulation tasks through Isaac Gym. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/PKU-MARL/DexterousHands) (👨‍💻 9 · 🔀 130 · 📦 5 · 📋 49 - 75% open · ⏱️ 18.02.2025):

	```
	git clone https://github.com/PKU-MARL/DexterousHands
	```
</details>
<details><summary><b><a href="https://github.com/graspit-simulator/graspit">Graspit!</a></b> (🥉11 ·  ⭐ 220 · 💤) - The GraspIt! simulator. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/graspit-simulator/graspit) (👨‍💻 25 · 🔀 87 · 📥 130 · 📋 79 - 45% open · ⏱️ 10.07.2020):

	```
	git clone https://github.com/graspit-simulator/graspit
	```
</details>
<details><summary><b><a href="https://github.com/hello-robot/stretch_mujoco">Strech MuJoCo</a></b> (🥉11 ·  ⭐ 70) - This library provides a simulation stack for Stretch, built on MuJoCo. <code><a href="https://tldrlegal.com/search?q=BSD-3-Clause-Clear">BSD-3-Clause-Clear</a></code></summary>

- [GitHub](https://github.com/hello-robot/stretch_mujoco) (👨‍💻 8 · 🔀 29 · 📋 52 - 40% open · ⏱️ 07.10.2026):

	```
	git clone https://github.com/hello-robot/stretch_mujoco
	```
</details>
<details><summary><b><a href="https://github.com/tenfoldpaper/multipanda_ros2">multipanda_ros2</a></b> (🥉10 ·  ⭐ 91) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/tenfoldpaper/multipanda_ros2) (👨‍💻 17 · 🔀 41 · 📋 8 - 25% open · ⏱️ 28.10.2026):

	```
	git clone https://github.com/tenfoldpaper/multipanda_ros2
	```
</details>
<details><summary><b><a href="https://raw.org/research/inverse-kinematics-of-a-stewart-platform/">Stewart Platform Simulator</a></b> (🥉7 ·  ⭐ 47 · 💤) - The RAW inverse kinematics library for Stewart Platforms written in JavaScript. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/rawify/Stewart.js) (👨‍💻 1 · 🔀 17 · 📦 4 · ⏱️ 18.08.2025):

	```
	git clone https://github.com/rawify/Stewart.js
	```
</details>
<details><summary><b><a href="https://github.com/tjards/multi-agent_sim">multi-agent_sim</a></b> (🥉7 ·  ⭐ 47) - Fully open architecture implementation of modern multi-agent swarming techniques. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/tjards/multi-agent_sim) (👨‍💻 2 · 🔀 7 · ⏱️ 28.05.2026):

	```
	git clone https://github.com/tjards/multi-agent_sim
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/roadrunner.html">Roadrunner</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<br>

## Game engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_3D engines made for games but can be interfaced with robotic frameworks_

<details><summary><b><a href="https://godotengine.org/">Godot</a></b> (🥇45 ·  ⭐ 120K · 📈) - Godot Engine Multi-platform 2D and 3D game engine. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/godotengine/godot) (👨‍💻 4K · 🔀 27K · 📥 24M · 📦 24 · 📋 70K - 27% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/godotengine/godot
	```
</details>
<details><summary><b><a href="https://bevy.org/">Bevy</a></b> (🥈39 ·  ⭐ 50K) - A refreshingly simple data-driven game engine built in Rust. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/bevyengine/bevy) (👨‍💻 1.7K · 🔀 5K · 📦 27K · 📋 9.5K - 36% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/bevyengine/bevy
	```
</details>
<details><summary><b><a href="https://o3de.org/">O3DE</a></b> (🥈30 ·  ⭐ 9.9K) - Open 3D Engine (O3DE) is an Apache 2.0-licensed multi-platform 3D engine that enables developers and content creators.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Apache-2">MIT and Apache-2</a></code></summary>

- [GitHub](https://github.com/o3de/o3de) (👨‍💻 360 · 🔀 2.6K · 📥 13K · 📋 8.2K - 42% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://unity.com/">Unity</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Unity%20Subscription%20Plans">Unity Subscription Plans</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.unrealengine.com/">Unreal Engine</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=EULA">EULA</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.roblox.com/">Roblox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary">proprietary</a></code></summary>

- _No project information available._</details>
<br>

## Physics Engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Physics Engines that simulate multi-joint dynamics, gravity etc_

<details><summary><b><a href="http://stack-of-tasks.github.io/pinocchio/">Pinocchio</a></b> (🥇36 ·  ⭐ 3.9K) - A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives. <code><a href="http://bit.ly/3rqEWVr">BSD-2</a></code></summary>

- [GitHub](https://github.com/stack-of-tasks/pinocchio) (👨‍💻 130 · 🔀 560 · 📥 240K · 📋 1.2K - 8% open · ⏱️ 13.11.2026):

	```
	git clone https://github.com/stack-of-tasks/pinocchio
	```
- [PyPi](https://pypi.org/project/pin) (📥 1.1M / month · 📦 140 · ⏱️ 08.07.2026):
	```
	pip install pin
	```
</details>
<details><summary><b><a href="https://drake.mit.edu/">Drake</a></b> (🥇33 ·  ⭐ 4.3K · 📈) - Model-based design and verification for robotics. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/RobotLocomotion/drake) (👨‍💻 290 · 🔀 1.4K · 📥 170K · 📋 7K - 10% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/RobotLocomotion/drake
	```
</details>
<details><summary><b><a href="http://bulletphysics.org/">Bullet Physics SDK</a></b> (🥇28 ·  ⭐ 15K) - Bullet Physics SDK: real-time collision detection and multi-physics simulation for VR, games, visual effects,.. <code><a href="https://tldrlegal.com/search?q=zlib">zlib</a></code></summary>

- [GitHub](https://github.com/bulletphysics/bullet3) (👨‍💻 310 · 🔀 3K · 📥 5.6K · 📦 24 · 📋 2.2K - 18% open · ⏱️ 09.10.2026):

	```
	git clone https://github.com/bulletphysics/bullet3
	```
</details>
<details><summary><b><a href="https://projectchrono.org">Project CHRONO</a></b> (🥈27 ·  ⭐ 3.2K) - High-performance C++ library for multiphysics and multibody dynamics simulations. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/projectchrono/chrono) (👨‍💻 160 · 🔀 630 · 📥 9.6K · 📋 430 - 4% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/projectchrono/chrono
	```
</details>
<details><summary><b><a href="https://crates.io/crates/avian3d">Avian 3D</a></b> (🥈26 ·  ⭐ 3.3K · 📈) - ECS-driven 2D and 3D physics engine for the Bevy game engine. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/avianphysics/avian) (👨‍💻 96 · 🔀 300 · 📦 410 · 📋 430 - 43% open · ⏱️ 04.11.2026):

	```
	git clone https://github.com/Jondolf/avian
	```
</details>
<details><summary><b><a href="https://github.com/google-deepmind/mujoco_warp">MuJoCo Wrap</a></b> (🥈25 ·  ⭐ 1.6K) - GPU-optimized version of the MuJoCo physics simulator, designed for NVIDIA hardware. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google-deepmind/mujoco_warp) (👨‍💻 72 · 🔀 280 · 📋 470 - 27% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/google-deepmind/mujoco_warp
	```
</details>
<details><summary><b><a href="http://dartsim.github.io/">DART</a></b> (🥈25 ·  ⭐ 1.2K) - DART: Dynamic Animation and Robotics Toolkit. <code><a href="http://bit.ly/3rqEWVr">BSD-2</a></code></summary>

- [GitHub](https://github.com/dartsim/dart) (👨‍💻 80 · 🔀 300 · 📥 47 · 📦 9 · 📋 740 - 1% open · ⏱️ 25.10.2026):

	```
	git clone https://github.com/dartsim/dart
	```
</details>
<details><summary><b><a href="https://github.com/google/brax">BRAX</a></b> (🥈24 ·  ⭐ 3.3K) - Massively parallel rigidbody physics simulation on accelerator hardware. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google/brax) (👨‍💻 56 · 🔀 350 · 📦 630 · 📋 420 - 24% open · ⏱️ 20.10.2026):

	```
	git clone https://github.com/google/brax
	```
</details>
<details><summary><b><a href="https://github.com/JSBSim-Team/jsbsim">JSBSim</a></b> (🥈24 ·  ⭐ 2.4K) - An open source flight dynamics & control software library. <code><a href="https://tldrlegal.com/search?q=LGPL-2.1">LGPL-2.1</a></code></summary>

- [GitHub](https://github.com/JSBSim-Team/jsbsim) (👨‍💻 83 · 🔀 620 · 📥 42K · 📋 390 - 13% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/JSBSim-Team/jsbsim
	```
</details>
<details><summary><b><a href="https://simtk.org/home/simbody">Simbody</a></b> (🥉22 ·  ⭐ 2.6K) - High-performance C++ multibody dynamics/physics library for simulating articulated biomechanical and mechanical.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/simbody/simbody) (👨‍💻 71 · 🔀 490 · 📋 370 - 43% open · ⏱️ 05.11.2026):

	```
	git clone https://github.com/simbody/simbody
	```
</details>
<details><summary><b><a href="https://nvidia-omniverse.github.io/PhysX/">PhysX 5</a></b> (🥉21 ·  ⭐ 4.9K) - NVIDIA PhysX SDK. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/NVIDIA-Omniverse/PhysX) (👨‍💻 13 · 🔀 680 · 📥 110K · 📋 290 - 47% open · ⏱️ 13.11.2026):

	```
	git clone https://github.com/NVIDIA-Omniverse/PhysX
	```
</details>
<details><summary><b><a href="https://gazebosim.org/">TPE (part of gz-physics)</a></b> (🥉16 ·  ⭐ 95) - Abstract physics interface designed to support simulation and rapid development of robot applications. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/gazebosim/gz-physics) (👨‍💻 66 · 🔀 90 · 📋 220 - 50% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/gazebosim/gz-physics
	```
</details>
<details><summary><b><a href="https://nvidia-omniverse.github.io/PhysX/">PhysX SDK (legacy)</a></b> (🥉15 ·  ⭐ 3.6K · 💤) - NVIDIA PhysX SDK. <code><a href="https://tldrlegal.com/search?q=NVIDIA%20Omniverse%20License%20Agreement">NVIDIA Omniverse License Agreement</a></code></summary>

- [GitHub](https://github.com/NVIDIAGameWorks/PhysX) (👨‍💻 3 · 🔀 840 · 📋 640 - 51% open · ⏱️ 09.11.2022):

	```
	git clone https://github.com/NVIDIAGameWorks/PhysX
	```
</details>
<details><summary><b><a href="https://raisim.com/">RaiSim</a></b> (🥉15 ·  ⭐ 420) - Visit www.raisim.com. <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- [GitHub](https://github.com/raisimTech/raisimLib) (👨‍💻 17 · 🔀 110 · 📥 290 · 📋 410 - 14% open · ⏱️ 25.04.2026):

	```
	git clone https://github.com/raisimTech/raisimlib
	```
</details>
<details><summary><b><a href="http://www.ode.org/">ODE</a></b> (🥉11 ·  ⭐ 220 · 💤) - Open Dynamics Engine (ODE) github mirror from https://bitbucket.org/odedevs/ode. <code><a href="https://tldrlegal.com/search?q=gnu-gpl%20and%20BSD-3-clause">gnu-gpl and BSD-3-clause</a></code></summary>

- [GitHub](https://github.com/thomasmarsh/ODE) (👨‍💻 34 · 🔀 45 · 📋 3 - 66% open · ⏱️ 14.01.2024):

	```
	git clone https://github.com/thomasmarsh/ODE
	```
</details>
<details><summary><b><a href="https://github.com/NVIDIAGameWorks/FleX">FleX</a></b> (🥉9 ·  ⭐ 820 · 💤) -  <code><a href="https://tldrlegal.com/search?q=Nvidia%20Software%20License">Nvidia Software License</a></code></summary>

- [GitHub](https://github.com/NVIDIAGameWorks/FleX) (👨‍💻 2 · 🔀 120 · 📋 140 - 69% open · ⏱️ 15.04.2021):

	```
	git clone https://github.com/NVIDIAGameWorks/FleX
	```
</details>
<details><summary><b><a href="https://github.com/YunzhuLi/PyFleX">PyFleX</a></b> (🥉6 ·  ⭐ 150 · 💤) - Customized Python APIs for NVIDIA FleX. <code><a href="https://tldrlegal.com/search?q=Nvidia%20Software%20License">Nvidia Software License</a></code></summary>

- [GitHub](https://github.com/YunzhuLi/PyFleX) (👨‍💻 1 · 🔀 30 · 📋 10 - 40% open · ⏱️ 17.08.2021):

	```
	git clone https://github.com/YunzhuLi/PyFleX
	```
</details>
<details><summary><b><a href="https://www.algoryx.se/agx-dynamics/">AGX Dynamics by Algoryx</a></b> (🥉2) - AGX Dynamics, by Algoryx, is a modular physics simulation toolkit available in C++, C# and Python, on Windows, Mac and.. <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.cm-labs.com/en/vortex-studio/">Vortex</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=EULA">EULA</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="http://newtondynamics.com/">Newton Dynamics</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=zlib">zlib</a></code></summary>

- [GitHub]():

	```
	git clone https://github.com/newton-dynamics/newton-dynamics
	```
</details>
<details><summary><b><a href="https://www.havok.com/havok-physics/">Havok Physics</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Per-title%20licensing%20model">Per-title licensing model</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.mathworks.com/products/simscape.html">Simscape</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<br>

## Rendering engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Rendering engines for robotic simulators_

<details><summary><b><a href="https://docs.o3de.org/docs/atom-guide/">Atom</a></b> (🥇30 ·  ⭐ 9.9K) - Open 3D Engine (O3DE) is an Apache 2.0-licensed multi-platform 3D engine that enables developers and content creators.. <code><a href="https://tldrlegal.com/search?q=Apache-2.0%20and%20MIT">Apache-2.0 and MIT</a></code></summary>

- [GitHub](https://github.com/o3de/o3de) (👨‍💻 360 · 🔀 2.6K · 📥 13K · 📋 8.2K - 42% open · ⏱️ 17.11.2026):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://cyberbotics.com/">Wren (Webots)</a></b> (🥇30 ·  ⭐ 4.8K) - Webots Robot Simulator. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/cyberbotics/webots) (👨‍💻 160 · 🔀 2K · 📥 2M · 📋 2K - 12% open · ⏱️ 16.11.2026):

	```
	git clone https://github.com/cyberbotics/webots
	```
</details>
<details><summary><b><a href="https://www.ogre3d.org/">OGRE</a></b> (🥈29 ·  ⭐ 4.7K) - high-performance rendering backend (C++, Python, C#, Java). <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/OGRECave/ogre) (👨‍💻 350 · 🔀 1K · 📥 3.8K · 📋 1K - 16% open · ⏱️ 18.11.2026):

	```
	git clone https://github.com/OGRECave/ogre
	```
</details>
<details><summary><b><a href="http://pyrender.readthedocs.io/">PyRender</a></b> (🥉28 ·  ⭐ 1.5K · 💤) - Easy-to-use glTF 2.0-compliant OpenGL renderer for visualization of 3D scenes. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/mmatl/pyrender) (👨‍💻 23 · 🔀 260 · 📦 3.3K · 📋 260 - 71% open · ⏱️ 30.04.2022):

	```
	git clone https://github.com/mmatl/pyrender
	```
- [PyPi](https://pypi.org/project/pyrender) (📥 570K / month · 📦 37 · ⏱️ 18.02.2021):
	```
	pip install pyrender
	```
</details>
<details><summary><b><a href="https://www.khronos.org/">Vulkan</a></b> (🥉24 ·  ⭐ 3.8K) - Open-Source Vulkan C++ API. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/KhronosGroup/Vulkan-Hpp) (👨‍💻 110 · 🔀 370 · 📋 740 - 3% open · ⏱️ 09.11.2026):

	```
	git clone https://github.com/KhronosGroup/Vulkan-Hpp
	```
</details>
<details><summary><b><a href="https://www.khronos.org/">OpenGL</a></b> (🥉24 ·  ⭐ 3.8K) - Open-Source Vulkan C++ API. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/KhronosGroup/Vulkan-Hpp) (👨‍💻 110 · 🔀 370 · 📋 740 - 3% open · ⏱️ 09.11.2026):

	```
	git clone https://github.com/KhronosGroup/Vulkan-Hpp
	```
</details>
<details><summary><b><a href="https://docs.unity3d.com/Manual/NativePluginInterface.html">Unity Rendering Plugin</a></b> (🥉14 ·  ⭐ 950) - C++ Rendering Plugin example for Unity. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Unity-Technologies/NativeRenderingPlugin) (👨‍💻 13 · 🔀 190 · 📋 34 - 91% open · ⏱️ 10.08.2026):

	```
	git clone https://github.com/Unity-Technologies/NativeRenderingPlugin
	```
</details>
<br>

## Others

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

<details><summary><b><a href="https://highway-env.farama.org/">HighwayEnv</a></b> (🥇28 ·  ⭐ 3.3K) - A collection of environments for autonomous driving and tactical decision-making tasks. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Farama-Foundation/HighwayEnv) (👨‍💻 64 · 🔀 880 · 📋 500 - 8% open · ⏱️ 11.11.2026):

	```
	git clone https://github.com/Farama-Foundation/HighwayEnv
	```
- [PyPi](https://pypi.org/project/highway-env) (📥 6.1K / month · 📦 12 · ⏱️ 07.08.2026):
	```
	pip install highway-env
	```
</details>

---

## Resources

Here are the resources used to complete these list, if not directly contributed by others.

### Lists:
- [**Aerial Robotics Landscape** - Simulation](https://ros-aerial.github.io/aerial_robotic_landscape/simulation/): A linking website to all kinds of aerial robotic tooling
- [**Awesome Weekly Robotics list**](https://www.weeklyrobotics.com/awesome-wr): All kinds of useful links as featured in Weekly Robotics
- [**ROS discourse**](https://discourse.ros.org/search?q=simulation): shared simulators with the ROS community
- [**Awesome Robotics by Kiloreux**](https://github.com/kiloreux/awesome-robotics) A list of awesome robotics resources
- [**Awesome Robotics by ahundt**](https://github.com/ahundt/awesome-robotics)
- [**Awesome ros2 by fkromer**](https://github.com/fkromer/awesome-ros2?tab=readme-ov-file#readme)
- [**Awesome robotic tooling**](https://github.com/Ly0n/awesome-robotic-tooling?tab=readme-ov-file)
- [**Awesome robotics projects by mjyc**](https://github.com/mjyc/awesome-robotics-projects?tab=readme-ov-file)
- [**Awesome LLM Robotics by GT-RIPL**](https://github.com/GT-RIPL/Awesome-LLM-Robotics?tab=readme-ov-file#simulation-frameworks):
- [**Best-of lists**](https://best-of.org): Discover other best-of lists with awesome open-source projects on all kinds of topics.
- [**Robot-manipulation.org simulator list**](https://www.robot-manipulation.org/software/simulators)

### Repositories

* [Copper RS](https://github.com/copper-project/copper-rs)
* [Hello Robot](https://github.com/hello-robot)

### Blogs
* [**Navigating through the Robotic Simulation Landscape**](https://www.mcguirerobotics.com/blog/2025/04/17/navigating-through-the-robotic-simulation-landscape/): The blogpost by Kimberly McGuire that started this list
* [**Ekumen at FOSDEM 2025: Accelerating robotics development through simulation**](https://ekumenlabs.com/blog/posts/accelerate-robotic-dev-sim/): The blogpost by Ekumen about robotic simulation.

### Talks:
* [**FOSDem 2025 Robotics and Simulation**](https://fosdem.org/2025/schedule/event/fosdem-2025-6252-accelerating-robotics-development-through-simulation/): Talk about an overview of robotic simulators

### Papers:
- C. A. Dimmig et al., "Survey of Simulators for Aerial Robots: An Overview and In-Depth Systematic Comparisons," in IEEE Robotics & Automation Magazine, doi: 10.1109/MRA.2024.3433171 [ArXiv](https://arxiv.org/abs/2311.02296)
- Player, Timothy R., et al. "From concept to field tests: Accelerated development of multi-AUV missions using a high-fidelity faster-than-real-time simulator." 2023 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2023. [ArXiv](https://arxiv.org/abs/2311.10377)

## Contribution

### Contributors
Those that contributed to this list, proposed updates or have suggested new projects:

* Kimberly McGuire (@knmcguire)
* Mat Sadowski (@msadowski)
* Sebastian Castro (@sea-bass)
* Marek Kraft (@PUTvision)
* Fatemeh Pourhashem (?)
* Ramon Roche (@mrpollo)
* Robert Eisele (@infusion)
* Silvio Traversaro (@traversaro)
* Hugo Börjesson (@hugoberjesson)
* Neeraj Cherakara (@iamnambiar)
* @jmackay2
* Christoph Kammer (@ckammer87)
* Gokul Puthumanaillam (@gokulp01)
* Spicer Bak (@SBFRF)
* Mabel Zhang (@mabelzhang)
* Pedro Roque (@pPedro-Roque)
* Özer Özkahraman (@KKalem)
* Pierre Kancier (@khancyr)
* Tanmay/Chinmay Samak (@Tinker-Twins)
* Peixuan Shu (@Peixuan Shu)
* Jennifer Buehler (@JenniferBuehler)
* @MiaoDX
* Louis Le Lay (@louislelay)
* Matej Kováč (@multicast)
* Jon Skerlj (@jonskerlj)
* @AccessViolationEnjoyer
* Arne Baeyens (@abaeyens)
* Shisato Yano (@ShisatoYano)
* Sarahi Ortega (@arahiod22)
* David Čapek (@DavidCapek)
* @BlakePR

### How to Contribute

Contributions are encouraged and always welcome! If you like to add or update projects, choose one of the following ways:

- Open an issue by selecting one of the provided categories from the [issue page](https://github.com/knmcguire/best-of-robot-simulators/issues/new/choose) and fill in the requested information.
- Modify the [projects.yaml](https://github.com/knmcguire/best-of-robot-simulators/blob/main/projects.yaml) with your additions or changes, and submit a pull request. This can also be done directly via the [Github UI](https://github.com/knmcguire/best-of-robot-simulators/edit/main/projects.yaml).

If you like to contribute to or share suggestions regarding the project metadata collection or markdown generation, please refer to the [best-of-generator](https://github.com/best-of-lists/best-of-generator) repository. If you like to create your own best-of list, we recommend to follow [this guide](https://github.com/best-of-lists/best-of/blob/main/create-best-of-list.md).

For more information on how to add or update projects, please read the [contribution guidelines](https://github.com/knmcguire/best-of-robot-simulators/blob/main/CONTRIBUTING.md). By participating in this project, you agree to abide by its [Code of Conduct](https://github.com/knmcguire/best-of-robot-simulators/blob/main/.github/CODE_OF_CONDUCT.md).

## Other projects for Simulation Comparison

* [**Simulately**](https://simulately.wiki/) - Has some great tools for fluid and soft body physics simulators!

## License

[![CC0](https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg)](https://creativecommons.org/licenses/by-sa/4.0/)
