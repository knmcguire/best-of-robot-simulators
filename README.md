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
    <a href="#Contents" title="Project Count"><img src="https://img.shields.io/badge/projects-140-blue.svg?color=5ac4bf"></a>
    <a href="#Contribution" title="Contributions are welcome"><img src="https://img.shields.io/badge/contributions-welcome-green.svg"></a>
    <a href="https://github.com/knmcguire/best-of-robot-simulators/releases" title="Best-of Updates"><img src="https://img.shields.io/github/release-date/knmcguire/best-of-robot-simulators?color=green&label=updated"></a>
</p>

This curated list contains 140 awesome simulator projects with a total of 400K stars grouped into 11 categories. All projects are ranked by a project-quality score, which is calculated based on various metrics automatically collected from GitHub and different package managers. If you like to add or update projects, feel free to open an [issue](https://github.com/knmcguire/best-of-robot-simulators/issues/new/choose), submit a [pull request](https://github.com/knmcguire/best-of-robot-simulators/pulls), or directly edit the [projects.yaml](https://github.com/knmcguire/best-of-robot-simulators/edit/main/projects.yaml). Contributions are very welcome!

> 🧙‍♂️  Discover other [best-of lists](https://best-of.org) or [create your own](https://github.com/best-of-lists/best-of/blob/main/create-best-of-list.md).

### Definition Robotics Simulator
Here is a definition of a robotics simulator derived in [this blogpost](https://www.mcguirerobotics.com/blog/2025/04/17/navigating-through-the-robotic-simulation-landscape/)

> A robotic simulator is a software framework that provides a virtual environment, often leveraging different physics/rendering engines and sensor models, to model the robot's behavior, its interaction and perception with the simulated world for design, evaluative or data-generative purposes.

With:

* **virtual environment** - To provide the scenario for the simulated robot to act in, depending on the application, like an indoor building, forest, or lunar landscape.
* **behavior, its interaction and perception** - The simulated entity should be able to interact with and act upon that virtual environment or world through its simulated sensors and actuators.
* **physics/rendering engines and sensor models** - To be able to simulate those interactions and perceptions caused by the robot's behavior, to model how an object will slip while being grasped or the noise of the lidar ranges.
* **design, evaluative or data-generative** - To use this as a development tool, as part of continuous integration to assure quality, or to collect data that can be used for AI training purposes.

## Contents

- [Generic Robotics Simulators](#generic-robotics-simulators) _23 projects_
- [Robotic simulators in 2D](#robotic-simulators-in-2d) _4 projects_
- [Aerial Robotics Simulators](#aerial-robotics-simulators) _18 projects_
- [Maritime Robotics Simulators](#maritime-robotics-simulators) _19 projects_
- [Automotive Simulators](#automotive-simulators) _5 projects_
- [Space Robotics Simulators](#space-robotics-simulators) _4 projects_
- [AI training Simulators](#ai-training-simulators) _24 projects_
- [Other Domain Specific Simulators](#other-domain-specific-simulators) _5 projects_
- [Game engines](#game-engines) _5 projects_
- [Physics Engines](#physics-engines) _21 projects_
- [Rendering engines](#rendering-engines) _7 projects_

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

<details><summary><b><a href="https://mujoco.org/">mujoco</a></b> (🥇37 ·  ⭐ 10K) - Multi-Joint dynamics with Contact. A general purpose physics simulator. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google-deepmind/mujoco) (👨‍💻 92 · 🔀 1.1K · 📥 680K · 📦 4.7K · 📋 1.7K - 7% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/google-deepmind/mujoco
	```
</details>
<details><summary><b><a href="https://o3de.org/">O3DE for Robotics</a></b> (🥇30 ·  ⭐ 8.5K · 📈) - Open 3D Engine (O3DE) is an Apache 2.0-licensed multi-platform 3D engine that enables developers and content creators.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Apache-2">MIT and Apache-2</a></code></summary>

- [GitHub](https://github.com/o3de/o3de) (👨‍💻 310 · 🔀 2.3K · 📥 10K · 📋 7.8K - 42% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://www.cyberbotics.com/">Webots</a></b> (🥇28 ·  ⭐ 3.7K) - Webots Robot Simulator. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/cyberbotics/webots) (👨‍💻 140 · 🔀 1.8K · 📥 1.7M · 📋 1.9K - 11% open · ⏱️ 03.08.2025):

	```
	git clone https://github.com/cyberbotics/webots
	```
</details>
<details><summary><b><a href="https://gazebosim.org/home">Gazebo</a></b> (🥈20 ·  ⭐ 990) - Open source robotics simulator. The latest version of Gazebo. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/gazebosim/gz-sim) (👨‍💻 160 · 🔀 320 · 📋 990 - 43% open · ⏱️ 09.08.2025):

	```
	git clone https://github.com/gazebosim/gz-sim
	```
</details>
<details><summary><b><a href="https://openrave.org/">OpenRAVE</a></b> (🥈18 ·  ⭐ 780) - Open Robotics Automation Virtual Environment: An environment for testing, developing, and deploying robotics motion.. <code><a href="https://tldrlegal.com/search?q=Apache-2%20and%20LGPL-3">Apache-2 and LGPL-3</a></code></summary>

- [GitHub](https://github.com/rdiankov/openrave) (👨‍💻 120 · 🔀 320 · 📋 520 - 57% open · ⏱️ 16.08.2024):

	```
	git clone https://github.com/rdiankov/openrave
	```
</details>
<details><summary><b><a href="https://newton-physics.github.io/newton/">Newton (Physics)</a></b> (🥈17 ·  ⭐ 710 · 🐣) - An open-source, GPU-accelerated physics simulation engine built upon NVIDIA Warp, specifically targeting roboticists.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/newton-physics/newton) (👨‍💻 24 · 🔀 65 · 📋 250 - 40% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/newton-physics/newton
	```
</details>
<details><summary><b><a href="https://github.com/Unity-Technologies/Unity-Robotics-Hub">Unity Robotics Hub</a></b> (🥈16 ·  ⭐ 2.3K) - Central repository for tools, tutorials, resources, and documentation for robotics simulation in Unity. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) (👨‍💻 13 · 🔀 440 · 📋 200 - 25% open · ⏱️ 26.11.2024):

	```
	git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub
	```
</details>
<details><summary><b><a href="https://github.com/RobotecAI/ros2-for-unity">Ros2 For Unity</a></b> (🥉15 ·  ⭐ 560) - High-performance ROS2 solution for Unity3D. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/RobotecAI/ros2-for-unity) (👨‍💻 12 · 🔀 64 · 📥 7.4K · 📋 93 - 40% open · ⏱️ 30.09.2024):

	```
	git clone https://github.com/RobotecAI/ros2-for-unity
	```
</details>
<details><summary><b><a href="https://www.argos-sim.info/">ARGoS</a></b> (🥉15 ·  ⭐ 290) - A parallel, multi-engine simulator for heterogeneous swarm robotics. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/ilpincy/argos3) (👨‍💻 25 · 🔀 110 · 📋 130 - 21% open · ⏱️ 03.05.2025):

	```
	git clone https://github.com/ilpincy/argos3
	```
</details>
<details><summary><b><a href="https://developer.nvidia.com/isaac/sim">NVIDIA Isaac Sim</a></b> (🥉14 ·  ⭐ 1.2K · 🐣) - NVIDIA Isaac Sim is an open-source application on NVIDIA Omniverse for developing, simulating, and testing AI-driven.. <code><a href="https://tldrlegal.com/search?q=Apache%202.0%20and%20NVIDIA%20Omniverse%20License%20Agreement">Apache 2.0 and NVIDIA Omniverse License Agreement</a></code></summary>

- [GitHub](https://github.com/isaac-sim/IsaacSim) (👨‍💻 5 · 🔀 140 · 📋 91 - 46% open · ⏱️ 08.08.2025):

	```
	git clone https://github.com/isaac-sim/IsaacSim
	```
</details>
<details><summary><b><a href="https://github.com/code-iai/ROSIntegration">ROS1 Intergration for Unreal 4</a></b> (🥉14 ·  ⭐ 440) - Unreal Engine Plugin to enable ROS Support. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/code-iai/ROSIntegration) (👨‍💻 32 · 🔀 130 · 📋 130 - 39% open · ⏱️ 11.02.2025):

	```
	git clone https://github.com/code-iai/ROSIntegration
	```
</details>
<details><summary><b><a href="http://coppeliarobotics.com/">CoppeliaSim core library</a></b> (🥉12 ·  ⭐ 120) - CoppeliaSim core library. <code><a href="https://tldrlegal.com/search?q=gnu-gpl">gnu-gpl</a></code></summary>

- [GitHub](https://github.com/CoppeliaRobotics/coppeliaSimLib) (👨‍💻 3 · 🔀 43 · 📋 21 - 14% open · ⏱️ 01.08.2025):

	```
	git clone https://github.com/CoppeliaRobotics/coppeliaSimLib
	```
</details>
<details><summary><b><a href="http://robwork.dk/">Robwork</a></b> (🥉8 ·  ⭐ 33 · 💤) - RobWork is a collection of C++ libraries for simulation and control of robot systems, see http://robwork.dk To get.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitLab](https://gitlab.com/sdurobotics/RobWork) (🔀 39 · 📋 110 - 15% open · ⏱️ 07.04.2016):

	```
	git clone https://gitlab.com/sdurobotics/RobWork
	```
</details>
<details><summary><b><a href="https://gitlab.com/robocup-sim/SimSpark">SimSpark</a></b> (🥉7 ·  ⭐ 21) - A generic physical simulator. <code><a href="https://tldrlegal.com/search?q=Missing">Missing</a></code></summary>

- [GitLab](https://gitlab.com/robocup-sim/SimSpark) (🔀 8 · 📋 56 - 51% open · ⏱️ 13.10.2017):

	```
	git clone https://gitlab.com/robocup-sim/SimSpark
	```
</details>
<details><summary><b><a href="http://coppeliarobotics.com/">CoppeliaSim</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://new.abb.com/products/robotics/software-and-digital/robotstudio">RobotBuilder</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Commercial%20software">Commercial software</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://robodk.com/">RoboDK</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.mathworks.com/products/robotics.html">MATLAB Robotics Systems Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.robotec.ai/products">RoSi</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary>Show 4 hidden projects...</summary>

- <b><a href="https://classic.gazebosim.org/">Gazebo Classic</a></b> (🥈21 ·  ⭐ 1.3K) - Gazebo classic. For the latest version, see https://github.com/gazebosim/gz-sim. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://blog.openai.com/roboschool/">Roboschool</a></b> (🥈20 ·  ⭐ 2.2K · 💤) - DEPRECATED: Open-source software for robot simulation, integrated with OpenAI Gym. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
- <b><a href="http://morse-simulator.github.io/">Morse</a></b> (🥈16 ·  ⭐ 360 · 💤) - The Modular OpenRobots Simulation Engine. <code><a href="https://tldrlegal.com/search?q=OFL-1.1">OFL-1.1</a></code>
- <b><a href="https://simbad.sourceforge.net/">Simbad</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=GNU-gpl2">GNU-gpl2</a></code>
</details>
<br>

## Robotic simulators in 2D

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Robotic simulators that only work in a 2D environment, for instance navigation_

<details><summary><b><a href="https://ir-sim.readthedocs.io/en">IR-SIM</a></b> (🥇21 ·  ⭐ 570) - A Python based lightweight robot simulator for the development of algorithms in robotics navigation, control, and.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/hanruihua/ir-sim) (👨‍💻 9 · 🔀 69 · 📦 12 · 📋 30 - 13% open · ⏱️ 09.08.2025):

	```
	git clone https://github.com/hanruihua/ir-sim
	```
- [PyPi](https://pypi.org/project/ir-sim):
	```
	pip install ir-sim
	```
</details>
<details><summary><b><a href="https://pyrobosim.readthedocs.io/">pyrobosim</a></b> (🥈18 ·  ⭐ 340) - ROS 2 enabled 2D mobile robot simulator for behavior prototyping. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/sea-bass/pyrobosim) (👨‍💻 18 · 🔀 59 · 📦 7 · 📋 120 - 4% open · ⏱️ 11.08.2025):

	```
	git clone https://github.com/sea-bass/pyrobosim
	```
- [PyPi](https://pypi.org/project/pyrobosim):
	```
	pip install pyrobosim
	```
</details>
<details><summary><b><a href="https://github.com/MRPT/mvsim">mvsim</a></b> (🥉16 ·  ⭐ 340) - Vehicle and mobile robotics simulator. C++ & Python API. Use it as a standalone application or via ROS 1 or ROS 2. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/MRPT/mvsim) (👨‍💻 9 · 🔀 50 · 📋 28 - 28% open · ⏱️ 06.07.2025):

	```
	git clone https://github.com/MRPT/mvsim
	```
</details>
<details><summary><b><a href="https://flatland-simulator.readthedocs.io/">Flatland</a></b> (🥉12 ·  ⭐ 120 · 💤) - A 2D robot simulator for ROS. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/avidbots/flatland) (👨‍💻 14 · 🔀 41 · 📋 26 - 50% open · ⏱️ 07.05.2024):

	```
	git clone https://github.com/avidbots/flatland
	```
</details>
<br>

## Aerial Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for aerial robotics_

<details><summary><b><a href="https://cosys-lab.github.io/Cosys-AirSim/">Cosys-AirSim</a></b> (🥇20 ·  ⭐ 200) - AirSim is a simulator for drones, cars and more, built on Unreal Engine. We expand it with new implementations and.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Cosys-Lab/Cosys-AirSim) (👨‍💻 260 · 🔀 72 · 📥 4.8K · 📋 75 - 49% open · ⏱️ 16.04.2025):

	```
	git clone https://github.com/Cosys-Lab/Cosys-AirSim
	```
</details>
<details><summary><b><a href="https://github.com/ethz-asl/rotors_simulator">RotorS</a></b> (🥈18 ·  ⭐ 1.4K · 💤) - RotorS is a UAV gazebo simulator. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/ethz-asl/rotors_simulator) (👨‍💻 49 · 🔀 680 · 📋 380 - 42% open · ⏱️ 06.07.2021):

	```
	git clone https://github.com/ethz-asl/rotors_simulator
	```
</details>
<details><summary><b><a href="https://uzh-rpg.github.io/flightmare/">Flightmare</a></b> (🥈17 ·  ⭐ 1.2K · 💤) - An Open Flexible Quadrotor Simulator. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/uzh-rpg/flightmare) (👨‍💻 7 · 🔀 360 · 📥 5.5K · 📋 170 - 65% open · ⏱️ 15.05.2023):

	```
	git clone https://github.com/uzh-rpg/flightmare
	```
</details>
<details><summary><b><a href="https://utiasdsl.github.io/gym-pybullet-drones/">Gym Pybullet Drones</a></b> (🥈16 ·  ⭐ 1.6K) - PyBullet Gymnasium environments for single and multi-agent reinforcement learning of quadcopter control. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/utiasDSL/gym-pybullet-drones) (👨‍💻 17 · 🔀 430 · 📋 220 - 50% open · ⏱️ 30.06.2025):

	```
	git clone https://github.com/utiasDSL/gym-pybullet-drones
	```
</details>
<details><summary><b><a href="https://pegasussimulator.github.io/PegasusSimulator/">Pegasus Simulator</a></b> (🥈16 ·  ⭐ 520) - A framework built on top of NVIDIA Isaac Sim for simulating drones with PX4 support and much more. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/PegasusSimulator/PegasusSimulator) (👨‍💻 4 · 🔀 92 · 📋 52 - 38% open · ⏱️ 20.07.2025):

	```
	git clone https://github.com/PegasusSimulator/PegasusSimulator
	```
</details>
<details><summary><b><a href="https://flightgoggles.mit.edu/">FlightGoggles</a></b> (🥈15 ·  ⭐ 430 · 💤) - A framework for photorealistic hardware-in-the-loop agile flight simulation using Unity3D and ROS. Developed by MIT.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/mit-aera/FlightGoggles) (👨‍💻 9 · 🔀 97 · 📥 1.6K · 📋 170 - 11% open · ⏱️ 01.04.2024):

	```
	git clone https://github.com/mit-aera/FlightGoggles
	```
</details>
<details><summary><b><a href="https://ntnu-arl.github.io/aerial_gym_simulator/">Aerial Gym Simulator</a></b> (🥉14 ·  ⭐ 540) - Aerial Gym Simulator - Isaac Gym Simulator for Aerial Robots. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/ntnu-arl/aerial_gym_simulator) (👨‍💻 4 · 🔀 85 · 📋 38 - 23% open · ⏱️ 07.07.2025):

	```
	git clone https://github.com/ntnu-arl/aerial_gym_simulator
	```
</details>
<details><summary><b><a href="https://github.com/spencerfolk/rotorpy">rotorpy</a></b> (🥉14 ·  ⭐ 180) - A multirotor simulator with aerodynamics for education and research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/spencerfolk/rotorpy) (👨‍💻 5 · 🔀 41 · 📥 11 · 📋 17 - 5% open · ⏱️ 07.08.2025):

	```
	git clone https://github.com/spencerfolk/rotorpy
	```
</details>
<details><summary><b><a href="https://github.com/PX4/jMAVSim">jMAVSim</a></b> (🥉14 ·  ⭐ 94 · 💤) - Simple multirotor simulator with MAVLink protocol support. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/PX4/jMAVSim) (👨‍💻 28 · 🔀 210 · 📋 52 - 30% open · ⏱️ 17.12.2021):

	```
	git clone https://github.com/PX4/jMAVSim
	```
</details>
<details><summary><b><a href="https://github.com/gsilano/CrazyS">CrazyS</a></b> (🥉13 ·  ⭐ 170 · 💤) - CrazyS is an extension of the ROS package RotorS, aimed to modeling, developing and integrating the Crazyflie 2.0. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/gsilano/CrazyS) (👨‍💻 2 · 🔀 84 · 📋 88 - 10% open · ⏱️ 11.08.2022):

	```
	git clone https://github.com/gsilano/CrazyS
	```
</details>
<details><summary><b><a href="https://github.com/kousheekc/isaac_drone_racer">Isaac Drone Racer</a></b> (🥉10 ·  ⭐ 110 · 🐣) - Isaac Drone Racer is a reinforcement learning framework for high speed autonomous drone racing, built on top of.. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/kousheekc/isaac_drone_racer) (🔀 22 · 📋 3 - 33% open · ⏱️ 20.06.2025):

	```
	git clone https://github.com/kousheekc/isaac_drone_racer
	```
</details>
<details><summary><b><a href="https://www.flightgear.org/">Flightgear</a></b> (🥉8 ·  ⭐ 54 · 💤) - FlightGear open-source flight simulator [flightgear.org](https://www.flightgear.org). <code><a href="https://tldrlegal.com/search?q=gnu-gpl2">gnu-gpl2</a></code></summary>

- [GitLab](https://gitlab.com/flightgear/flightgear) (🔀 39 · 📋 320 - 48% open · ⏱️ 04.03.2015):

	```
	git clone https://gitlab.com/flightgear/flightgear
	```
</details>
<details><summary><b><a href="https://github.com/arplaboratory/RotorTM">RotorTM</a></b> (🥉6 ·  ⭐ 71 · 💤) -  <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/arplaboratory/RotorTM) (👨‍💻 3 · 🔀 14 · 📋 4 - 50% open · ⏱️ 09.06.2024):

	```
	git clone https://github.com/arplaboratory/RotorTM
	```
</details>
<details><summary><b><a href="https://github.com/shupx/swarm_sync_sim">swarm_sync_sim</a></b> (🥉6 ·  ⭐ 11) - swarm_sync_sim (also known as sss) is an ultra-lightweight, ROS-based simulator for robotic swarms, including.. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/shupx/swarm_sync_sim) (👨‍💻 6 · 🔀 1 · ⏱️ 18.03.2025):

	```
	git clone https://github.com/shupx/swarm_sync_sim
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/uav.html">Matlab UAV Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=MathWorks%20Software%20License%20Agreement">MathWorks Software License Agreement</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.realflight.com/">Realflight</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20software%20license">proprietary software license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.x-plane.com/">X-plane</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary>Show 1 hidden projects...</summary>

- <b><a href="https://github.com/microsoft/AirSim">airsim</a></b> (🥇31 ·  ⭐ 17K) - Open source simulator for autonomous vehicles built on Unreal Engine / Unity, from Microsoft AI & Research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Maritime Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for maritime robotics_

<details><summary><b><a href="https://github.com/osrf/vrx">Virtual RobotX</a></b> (🥇21 ·  ⭐ 540) - Virtual RobotX (VRX) resources. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/osrf/vrx) (👨‍💻 36 · 🔀 230 · 📋 550 - 7% open · ⏱️ 24.07.2025):

	```
	git clone https://github.com/osrf/vrx
	```
</details>
<details><summary><b><a href="https://field-robotics-lab.github.io/dave.doc/">DAVE</a></b> (🥈15 ·  ⭐ 250 · 💤) - Project DAVE. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Field-Robotics-Lab/dave) (👨‍💻 25 · 🔀 70 · 📋 130 - 23% open · ⏱️ 04.12.2023):

	```
	git clone https://github.com/Field-Robotics-Lab/dave
	```
</details>
<details><summary><b><a href="https://stonefish.readthedocs.io/">Stonefish</a></b> (🥈15 ·  ⭐ 170) - Stonefish - an advanced C++ simulation library designed for (but not limited to) marine robotics. <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub](https://github.com/patrykcieslak/stonefish) (👨‍💻 11 · 🔀 37 · 📋 37 - 16% open · ⏱️ 10.06.2025):

	```
	git clone https://github.com/patrykcieslak/stonefish
	```
</details>
<details><summary><b><a href="https://github.com/open-airlab/UNav-Sim">UNav-Sim</a></b> (🥈14 ·  ⭐ 230) - Visually Realistic Underwater Robotics Simulator UNav-Sim. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/open-airlab/UNav-Sim) (👨‍💻 250 · 🔀 21 · 📋 14 - 14% open · ⏱️ 02.05.2025):

	```
	git clone https://github.com/open-airlab/UNav-Sim
	```
</details>
<details><summary><b><a href="https://github.com/smarc-project/smarc2">SMaRC 2</a></b> (🥈14 ·  ⭐ 10) - smarc ros2-humble main repository. <code><a href="https://tldrlegal.com/search?q=BSD-3-Clause%20and%20MIT">BSD-3-Clause and MIT</a></code></summary>

- [GitHub](https://github.com/smarc-project/smarc2) (👨‍💻 22 · 🔀 26 · 📥 43 · 📋 26 - 34% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/smarc-project/smarc2
	```
</details>
<details><summary><b><a href="https://github.com/moos-ivp/moos-ivp">Moos-ivp</a></b> (🥉11 ·  ⭐ 27) - MOOS-IvP is a set of modules for providing autonomy on robotic platforms, in particular autonomous marine vehicles. <code><a href="https://tldrlegal.com/search?q=GPLv3%20LGPLv3%20and%20Commercial">GPLv3 LGPLv3 and Commercial</a></code></summary>

- [GitHub](https://github.com/moos-ivp/moos-ivp) (👨‍💻 17 · 🔀 20 · 📋 11 - 54% open · ⏱️ 11.08.2025):

	```
	git clone https://github.com/moos-ivp/moos-ivp
	```
</details>
<details><summary><b><a href="https://github.com/umfieldrobotics/OceanSim">Ocean Sim</a></b> (🥉10 ·  ⭐ 260 · 🐣) - [IROS 2025] OceanSim: A GPU-Accelerated Underwater Robot Perception Simulation Framework. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/umfieldrobotics/OceanSim) (👨‍💻 4 · 🔀 30 · 📋 11 - 54% open · ⏱️ 21.06.2025):

	```
	git clone https://github.com/umfieldrobotics/OceanSim
	```
</details>
<details><summary><b><a href="https://byu-holoocean.github.io/holoocean-docs/">HoloOcean</a></b> (🥉10 ·  ⭐ 50) - A UE5 based simulator for marine perception and autonomy, with multi-agent communications and many common underwater.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Unreal%20Engine%20EULA">MIT and Unreal Engine EULA</a></code></summary>

- [GitHub]() (👨‍💻 8 · 🔀 21 · 📋 150 - 18% open · ⏱️ 14.05.2025):

	```
	git clone https://github.com/byu-holoocean/HoloOcean
	```
</details>
<details><summary><b><a href="https://github.com/MARUSimulator/marus-core">MARUSimulator</a></b> (🥉7 ·  ⭐ 20) - Marine simulator core assets for Unity. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/MARUSimulator/marus-core) (👨‍💻 8 · 🔀 4 · 📋 5 - 40% open · ⏱️ 07.02.2025):

	```
	git clone https://github.com/MARUSimulator/marus-core
	```
</details>
<details><summary>Show 10 hidden projects...</summary>

- <b><a href="https://uuvsimulator.github.io/">UUV Simulator</a></b> (🥇16 ·  ⭐ 790 · 💤) - Gazebo/ROS packages for underwater robotics simulation. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://www.irs.uji.es/uwsim/wiki/index.php?title=Installing_UWSim">UWSim</a></b> (🥈12 ·  ⭐ 110 · 💤) - UWSim-NET, the Underwater Simulator. <code><a href="https://tldrlegal.com/search?q=GNU-gpl">GNU-gpl</a></code>
- <b><a href="https://github.com/uji-ros-pkg/underwater_simulation">UWSim-NET</a></b> (🥉11 ·  ⭐ 110 · 💤) - UWSim-NET, the Underwater Simulator. <code>Unlicensed</code>
- <b><a href="https://github.com/osrf/lrauv">LRAUV</a></b> (🥉10 ·  ⭐ 71) - Packages for simulating the Tethys-class Long-Range AUV (LRAUV) from the Monterey Bay Aquarium Research Institute.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://github.com/freefloating-gazebo/freefloating_gazebo">Freefloating</a></b> (🥉9 ·  ⭐ 74 · 💤) - A Gazebo plugin to simulate underwater vehicles and visualize with UWsim. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
- <b><a href="https://github.com/srmauvsoftware/URSim">URSim</a></b> (🥉8 ·  ⭐ 66 · 💤) - Simulator for Unmanned Underwater Vehicles using ROS and Unity3D. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://github.com/iti-luebeck/MARS">MARS</a></b> (🥉7 ·  ⭐ 12 · 💤) - MArine Robotics Simulator - An online Hardware-in-the-Loop simulation environment for multiple AUVs and ASVs. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code>
- <b><a href="https://bitbucket.org/whoidsl/ds_sim/src/master/">ds sim</a></b> -  <code>Unlicensed</code>
- <b><a href="https://sourceforge.net/projects/usarsim/">USARSim</a></b> -  <code>Unlicensed</code>
- <b><a href="https://github.com/eirikhex/UW-MORSE">UW-Morse</a></b> (🥉-1 ·  ⭐ 1 · 💤) -  <code>Unlicensed</code>
</details>
<br>

## Automotive Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for automotive_

<details><summary><b><a href="https://carla.org/">Carla</a></b> (🥇32 ·  ⭐ 13K) - Open-source simulator for autonomous driving research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/carla-simulator/carla) (👨‍💻 180 · 🔀 4K · 📦 1.1K · 📋 6.1K - 18% open · ⏱️ 08.08.2025):

	```
	git clone https://github.com/carla-simulator/carla
	```
</details>
<details><summary><b><a href="https://github.com/esmini/esmini">ESMINI</a></b> (🥈25 ·  ⭐ 860) - a basic OpenSCENARIO player. <code><a href="http://bit.ly/3postzC">MPL-2.0</a></code></summary>

- [GitHub](https://github.com/esmini/esmini) (👨‍💻 85 · 🔀 230 · 📥 28K · 📋 580 - 31% open · ⏱️ 08.07.2025):

	```
	git clone https://github.com/esmini/esmini
	```
</details>
<details><summary><b><a href="https://github.com/tier4/AWSIM">AWSim</a></b> (🥉24 ·  ⭐ 600) - Open source simulator for self-driving vehicles. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/tier4/AWSIM) (👨‍💻 32 · 🔀 97 · 📥 65K · 📋 110 - 32% open · ⏱️ 07.08.2025):

	```
	git clone https://github.com/tier4/AWSIM
	```
</details>
<details><summary>Show 2 hidden projects...</summary>

- <b><a href="https://github.com/lgsvl/simulator">SVL Simulator</a></b> (🥉21 ·  ⭐ 2.4K · 💤) - A ROS/ROS2 Multi-robot Simulator for Autonomous Vehicles. <code>Unlicensed</code>
- <b><a href="https://github.com/udacity/self-driving-car-sim">Self Driving Car</a></b> (🥉19 ·  ⭐ 4K · 💤) - A self-driving car simulator built with Unity. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Space Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for space robotics_

<details><summary><b><a href="https://github.com/AVSLab/basilisk">Basilisk</a></b> (🥇22 ·  ⭐ 230) - Astrodynamics simulation framework. <code><a href="http://bit.ly/3hkKRql">ISC</a></code></summary>

- [GitHub](https://github.com/AVSLab/basilisk) (👨‍💻 110 · 🔀 79 · 📋 380 - 23% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/AVSLab/basilisk
	```
</details>
<details><summary><b><a href="https://github.com/nasa/astrobee">Astrobee</a></b> (🥈20 ·  ⭐ 1.2K · 💤) - NASA Astrobee Robot Software. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/nasa/astrobee) (👨‍💻 22 · 🔀 340 · 📋 210 - 6% open · ⏱️ 03.07.2024):

	```
	git clone https://github.com/NASA/astrobee
	```
</details>
<details><summary><b><a href="https://github.com/OmniLRS/OmniLRS/wiki">OmiLRS</a></b> (🥉10 ·  ⭐ 110) - Omniverse Lunar Robotics Simulator. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/OmniLRS/OmniLRS) (👨‍💻 8 · 🔀 24 · 📋 23 - 39% open · ⏱️ 23.07.2025):

	```
	git clone https://github.com/OmniLRS/OmniLRS
	```
</details>
<details><summary><b><a href="https://github.com/PUTvision/LunarSim">LunarSim</a></b> (🥉5 ·  ⭐ 42 · 💤) - LunarSim: Lunar Rover Simulator Focused on High Visual Fidelity and ROS 2 Integration for Advanced Computer Vision.. <code><a href="https://tldrlegal.com/search?q=Missing">Missing</a></code></summary>

- [GitHub](https://github.com/PUTvision/LunarSim) (👨‍💻 2 · 🔀 6 · 📥 330 · 📋 4 - 50% open · ⏱️ 07.12.2023):

	```
	git clone https://github.com/PUTvision/LunarSim
	```
</details>
<br>

## AI training Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulations made for training for AI-agents like reinforcement learning_

<details><summary><b><a href="https://gymnasium.farama.org/">Gymnasium</a></b> (🥇32 ·  ⭐ 9.9K) - An API standard for single-agent reinforcement learning environments, with popular reference environments and related.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Farama-Foundation/Gymnasium) (👨‍💻 550 · 🔀 1.1K · 📦 20K · 📋 540 - 12% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/Farama-Foundation/Gymnasium
	```
- [PyPi](https://pypi.org/project/gymnasium):
	```
	pip install gymnasium
	```
</details>
<details><summary><b><a href="https://github.com/Genesis-Embodied-AI/Genesis">Genesis</a></b> (🥇29 ·  ⭐ 27K) - A generative world for general-purpose robotics & embodied AI learning. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Genesis-Embodied-AI/Genesis) (👨‍💻 63 · 🔀 2.4K · 📦 96 · 📋 760 - 14% open · ⏱️ 13.08.2025):

	```
	git clone https://github.com/Genesis-Embodied-AI/Genesis
	```
</details>
<details><summary><b><a href="https://isaac-sim.github.io/IsaacLab">NVIDIA Isaac Sim Isaac Lab</a></b> (🥈27 ·  ⭐ 4.6K) - Unified framework for robot learning built on NVIDIA Isaac Sim. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/isaac-sim/IsaacLab) (👨‍💻 140 · 🔀 2.1K · 📋 1.7K - 14% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/isaac-sim/IsaacLab
	```
</details>
<details><summary><b><a href="https://ai2thor.allenai.org/">AI2-thor</a></b> (🥈24 ·  ⭐ 1.5K) - An open-source platform for Visual AI. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/allenai/ai2thor) (👨‍💻 49 · 🔀 250 · 📦 380 · 📋 640 - 41% open · ⏱️ 29.05.2025):

	```
	git clone https://github.com/allenai/ai2thor
	```
</details>
<details><summary><b><a href="https://maniskill.ai/">ManiSkill</a></b> (🥈22 ·  ⭐ 1.9K) - SAPIEN Manipulation Skill Framework, an open source GPU parallelized robotics simulator and benchmark, led by Hillbot,.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/haosulab/ManiSkill) (👨‍💻 48 · 🔀 320 · 📋 660 - 14% open · ⏱️ 04.08.2025):

	```
	git clone https://github.com/haosulab/ManiSkill
	```
</details>
<details><summary><b><a href="https://aihabitat.org/">Habitat Sim</a></b> (🥈20 ·  ⭐ 3.1K) - A flexible, high-performance 3D simulator for Embodied AI research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/facebookresearch/habitat-sim) (👨‍💻 62 · 🔀 460 · 📋 830 - 23% open · ⏱️ 24.04.2025):

	```
	git clone https://github.com/facebookresearch/habitat-sim
	```
</details>
<details><summary><b><a href="https://jaxsim.readthedocs.io">jaxsim</a></b> (🥈20 ·  ⭐ 140) - A differentiable physics engine and multibody dynamics library for control and robot learning. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/ami-iit/jaxsim) (👨‍💻 13 · 🔀 17 · 📦 3 · 📋 72 - 19% open · ⏱️ 04.08.2025):

	```
	git clone https://github.com/ami-iit/jaxsim
	```
- [PyPi](https://pypi.org/project/jaxsim):
	```
	pip install jaxsim
	```
- [Conda](https://anaconda.org/conda-forge/jaxsim) (📥 10K · ⏱️ 29.04.2025):
	```
	conda install -c conda-forge jaxsim
	```
</details>
<details><summary><b><a href="https://sapien.ucsd.edu/">Sapien</a></b> (🥈19 ·  ⭐ 620) - SAPIEN Embodied AI Platform. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/haosulab/SAPIEN) (👨‍💻 15 · 🔀 54 · 📥 3.9K · 📦 230 · 📋 200 - 28% open · ⏱️ 17.07.2025):

	```
	git clone https://github.com/haosulab/SAPIEN
	```
</details>
<details><summary><b><a href="https://playground.mujoco.org/">MuJoCo playground</a></b> (🥈18 ·  ⭐ 1.3K) - An open-source library for GPU-accelerated robot learning and sim-to-real transfer. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google-deepmind/mujoco_playground) (🔀 170 · 📋 100 - 22% open · ⏱️ 01.08.2025):

	```
	git clone https://github.com/google-deepmind/mujoco_playground/
	```
</details>
<details><summary><b><a href="https://github.com/BYU-PCCL/holodeck">Holodeck</a></b> (🥉17 ·  ⭐ 590 · 💤) - High Fidelity Simulator for Reinforcement Learning and Robotics Research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/BYU-PCCL/holodeck) (👨‍💻 19 · 🔀 42 · 📦 10 · 📋 240 - 20% open · ⏱️ 30.04.2021):

	```
	git clone https://github.com/BYU-PCCL/holodeck
	```
</details>
<details><summary><b><a href="https://docs.kscale.dev/docs/ksim">K-Sim</a></b> (🥉17 ·  ⭐ 190) - RL training library for humanoid locomotion and manipulation. Built on top of MuJoCo and JAX. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/kscalelabs/ksim) (👨‍💻 8 · 🔀 25 · 📋 27 - 55% open · ⏱️ 10.08.2025):

	```
	git clone https://github.com/kscalelabs/ksim
	```
- [PyPi](https://pypi.org/project/ksim):
	```
	pip install ksim
	```
</details>
<details><summary><b><a href="https://loco-mujoco.readthedocs.io/">LocoMuJoCo</a></b> (🥉16 ·  ⭐ 1.1K) - Imitation learning benchmark focusing on complex locomotion tasks using MuJoCo. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/robfiras/loco-mujoco) (👨‍💻 13 · 🔀 120 · 📦 8 · 📋 64 - 43% open · ⏱️ 30.05.2025):

	```
	git clone https://github.com/robfiras/loco-mujoco
	```
</details>
<details><summary><b><a href="https://metadriverse.github.io/metadrive-simulator/">MetaDrive</a></b> (🥉16 ·  ⭐ 970) - MetaDrive: Lightweight driving simulator for everyone. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/metadriverse/metadrive) (🔀 140 · 📥 47K · 📋 390 - 21% open · ⏱️ 12.05.2025):

	```
	git clone https://github.com/metadriverse/metadrive/
	```
</details>
<details><summary><b><a href="https://roboverseorg.github.io/">RoboVerse</a></b> (🥉15 ·  ⭐ 1.4K · 🐣) - RoboVerse: Towards a Unified Platform, Dataset and Benchmark for Scalable and Generalizable Robot Learning. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/RoboVerseOrg/RoboVerse) (👨‍💻 25 · 🔀 110 · 📥 1 · 📋 140 - 47% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/RoboVerseOrg/RoboVerse
	```
</details>
<details><summary><b><a href="http://gibsonenv.stanford.edu/">Gibson</a></b> (🥉15 ·  ⭐ 920 · 💤) - Gibson Environments: Real-World Perception for Embodied Agents. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/StanfordVL/GibsonEnv) (👨‍💻 9 · 🔀 140 · 📋 120 - 39% open · ⏱️ 12.05.2021):

	```
	git clone https://github.com/StanfordVL/GibsonEnv
	```
</details>
<details><summary><b><a href="https://www.dynsyslab.org/safe-robot-learning/">Safe Control Gym</a></b> (🥉15 ·  ⭐ 750 · 📈) - PyBullet CartPole and Quadrotor environmentswith CasADi symbolic a priori dynamicsfor learning-based control and RL. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/utiasDSL/safe-control-gym) (👨‍💻 20 · 🔀 130 · 📋 57 - 12% open · ⏱️ 07.08.2025):

	```
	git clone https://github.com/utiasDSL/safe-control-gym
	```
</details>
<details><summary><b><a href="https://github.com/stepjam/PyRep">PyRep</a></b> (🥉15 ·  ⭐ 740 · 💤) - A toolkit for robot learning research. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/stepjam/PyRep) (👨‍💻 20 · 🔀 160 · 📋 300 - 0% open · ⏱️ 02.07.2024):

	```
	git clone https://github.com/stepjam/PyRep
	```
</details>
<details><summary><b><a href="https://procthor.allenai.org/">ProcTHOR</a></b> (🥉14 ·  ⭐ 380 · 💤) - Scaling Embodied AI by Procedurally Generating Interactive 3D Houses. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/allenai/procthor) (👨‍💻 6 · 🔀 32 · 📦 13 · 📋 51 - 84% open · ⏱️ 14.12.2022):

	```
	git clone https://github.com/allenai/procthor
	```
</details>
<details><summary><b><a href="https://deepdrive.io/">Deepdrive</a></b> (🥉12 ·  ⭐ 920 · 💤) - Deepdrive is a simulator that allows anyone with a PC to push the state-of-the-art in self-driving. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/deepdrive/deepdrive) (👨‍💻 6 · 🔀 150 · 📋 64 - 54% open · ⏱️ 29.06.2020):

	```
	git clone https://github.com/deepdrive/deepdrive
	```
</details>
<details><summary><b><a href="https://robocasa.ai/">RoboCasa</a></b> (🥉12 ·  ⭐ 880) - RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/robocasa/robocasa) (🔀 92 · 📋 140 - 25% open · ⏱️ 23.04.2025):

	```
	git clone https://github.com/robocasa/robocasa
	```
</details>
<details><summary><b><a href="https://pybullet.org/">PyBullet Gym</a></b> (🥉11 ·  ⭐ 860 · 💤) - Open-source implementations of OpenAI Gym MuJoCo environments for use with the OpenAI Gym Reinforcement Learning.. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/benelot/pybullet-gym) (👨‍💻 14 · 🔀 120 · 📋 57 - 52% open · ⏱️ 29.03.2021):

	```
	git clone https://github.com/benelot/pybullet-gym
	```
</details>
<details><summary><b><a href="https://arnold-benchmark.github.io/">Arnold</a></b> (🥉9 ·  ⭐ 170 · 💤) - [ICCV 2023] Official code repository for ARNOLD benchmark. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/arnold-benchmark/arnold) (👨‍💻 3 · 🔀 15 · 📋 53 - 9% open · ⏱️ 01.04.2024):

	```
	git clone https://github.com/arnold-benchmark/arnold
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/reinforcement-learning.html">Reinforcement Learning Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary>Show 1 hidden projects...</summary>

- <b><a href="https://www.gymlibrary.dev/">Gym</a></b> (🥇33 ·  ⭐ 36K · 💤) - A toolkit for developing and comparing reinforcement learning algorithms. <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Other Domain Specific Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Robotic simulators build for other domains like automotive or space robotics_

<details><summary><b><a href="https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator">AutoDRIVE Simulator</a></b> (🥇13 ·  ⭐ 110) - An Integrated Cyber-Physical Ecosystem for Autonomous Driving Research and Education. <code><a href="http://bit.ly/3rqEWVr">BSD-2</a></code></summary>

- [GitHub](https://github.com/Tinker-Twins/AutoDRIVE) (👨‍💻 3 · 🔀 22 · 📥 2.1K · ⏱️ 23.12.2024):

	```
	git clone https://github.com/Tinker-Twins/AutoDRIVE
	```
</details>
<details><summary><b><a href="https://github.com/hello-robot/stretch_mujoco">Strech MuJoCo</a></b> (🥈12 ·  ⭐ 50) - This library provides a simulation stack for Stretch, built on MuJoCo. <code><a href="https://tldrlegal.com/search?q=BSD-3-Clause-Clear">BSD-3-Clause-Clear</a></code></summary>

- [GitHub](https://github.com/hello-robot/stretch_mujoco) (👨‍💻 6 · 🔀 15 · 📋 47 - 40% open · ⏱️ 25.06.2025):

	```
	git clone https://github.com/hello-robot/stretch_mujoco
	```
</details>
<details><summary><b><a href="https://github.com/graspit-simulator/graspit">Graspit!</a></b> (🥉11 ·  ⭐ 200 · 💤) - The GraspIt! simulator. <code>Unlicensed</code></summary>

- [GitHub](https://github.com/graspit-simulator/graspit) (👨‍💻 25 · 🔀 79 · 📥 110 · 📋 76 - 43% open · ⏱️ 10.07.2020):

	```
	git clone https://github.com/graspit-simulator/graspit
	```
</details>
<details><summary><b><a href="https://raw.org/research/inverse-kinematics-of-a-stewart-platform/">Stewart Platform Simulator</a></b> (🥉7 ·  ⭐ 38) - The RAW inverse kinematics library for Stewart Platforms written in JavaScript. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/rawify/Stewart.js) (🔀 15 · 📦 4 · ⏱️ 11.07.2025):

	```
	git clone https://github.com/rawify/Stewart.js
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/roadrunner.html">Roadrunner</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<br>

## Game engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_3D engines made for games but can be interfaced with robotic frameworks_

<details><summary><b><a href="https://godotengine.org/">Godot</a></b> (🥇45 ·  ⭐ 100K) - Godot Engine Multi-platform 2D and 3D game engine. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/godotengine/godot) (👨‍💻 3.4K · 🔀 22K · 📥 11M · 📦 21 · 📋 58K - 20% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/godotengine/godot
	```
</details>
<details><summary><b><a href="https://bevy.org/">Bevy</a></b> (🥈39 ·  ⭐ 41K) - A refreshingly simple data-driven game engine built in Rust. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/bevyengine/bevy) (👨‍💻 1.4K · 🔀 4K · 📦 25K · 📋 7.2K - 33% open · ⏱️ 13.08.2025):

	```
	git clone https://github.com/bevyengine/bevy
	```
</details>
<details><summary><b><a href="https://o3de.org/">O3DE</a></b> (🥉30 ·  ⭐ 8.5K · 📈) - Open 3D Engine (O3DE) is an Apache 2.0-licensed multi-platform 3D engine that enables developers and content creators.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Apache-2">MIT and Apache-2</a></code></summary>

- [GitHub](https://github.com/o3de/o3de) (👨‍💻 310 · 🔀 2.3K · 📥 10K · 📋 7.8K - 42% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://unity.com/">Unity</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Unity%20Subscription%20Plans">Unity Subscription Plans</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.unrealengine.com/">Unreal Engine</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=EULA">EULA</a></code></summary>

- _No project information available._</details>
<br>

## Physics Engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Physics Engines that simulate multi-joint dynamics, gravity etc_

<details><summary><b><a href="https://drake.mit.edu/">Drake</a></b> (🥇32 ·  ⭐ 3.7K) - Model-based design and verification for robotics. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/RobotLocomotion/drake) (👨‍💻 270 · 🔀 1.2K · 📥 130K · 📋 6.5K - 10% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/RobotLocomotion/drake
	```
</details>
<details><summary><b><a href="https://projectchrono.org">Project CHRONO</a></b> (🥇28 ·  ⭐ 2.5K) - High-performance C++ library for multiphysics and multibody dynamics simulations. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/projectchrono/chrono) (👨‍💻 130 · 🔀 500 · 📥 8.9K · 📋 320 - 31% open · ⏱️ 13.08.2025):

	```
	git clone https://github.com/projectchrono/chrono
	```
</details>
<details><summary><b><a href="http://bulletphysics.org/">Bullet Physics SDK</a></b> (🥇27 ·  ⭐ 14K) - Bullet Physics SDK: real-time collision detection and multi-physics simulation for VR, games, visual effects,.. <code><a href="https://tldrlegal.com/search?q=zlib">zlib</a></code></summary>

- [GitHub](https://github.com/bulletphysics/bullet3) (👨‍💻 310 · 🔀 2.9K · 📥 5.6K · 📦 21 · 📋 2K - 11% open · ⏱️ 23.04.2025):

	```
	git clone https://github.com/bulletphysics/bullet3
	```
</details>
<details><summary><b><a href="https://github.com/google/brax">BRAX</a></b> (🥈26 ·  ⭐ 2.8K) - Massively parallel rigidbody physics simulation on accelerator hardware. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google/brax) (👨‍💻 43 · 🔀 300 · 📦 520 · 📋 390 - 21% open · ⏱️ 01.08.2025):

	```
	git clone https://github.com/google/brax
	```
</details>
<details><summary><b><a href="https://crates.io/crates/avian3d">Avian 3D</a></b> (🥈25 ·  ⭐ 2.3K · 📉) - ECS-driven 2D and 3D physics engine for the Bevy game engine. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/Jondolf/avian) (👨‍💻 66 · 🔀 170 · 📦 340 · 📋 320 - 43% open · ⏱️ 10.08.2025):

	```
	git clone https://github.com/Jondolf/avian
	```
</details>
<details><summary><b><a href="https://github.com/JSBSim-Team/jsbsim">JSBSim</a></b> (🥈24 ·  ⭐ 1.7K) - An open source flight dynamics & control software library. <code><a href="https://tldrlegal.com/search?q=LGPL-2.1">LGPL-2.1</a></code></summary>

- [GitHub](https://github.com/JSBSim-Team/jsbsim) (👨‍💻 70 · 🔀 490 · 📥 30K · 📋 360 - 10% open · ⏱️ 07.08.2025):

	```
	git clone https://github.com/JSBSim-Team/jsbsim
	```
</details>
<details><summary><b><a href="https://simtk.org/home/simbody">Simbody</a></b> (🥈22 ·  ⭐ 2.4K) - High-performance C++ multibody dynamics/physics library for simulating articulated biomechanical and mechanical.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/simbody/simbody) (👨‍💻 61 · 🔀 470 · 📋 340 - 40% open · ⏱️ 04.08.2025):

	```
	git clone https://github.com/simbody/simbody
	```
</details>
<details><summary><b><a href="http://dartsim.github.io/">DART</a></b> (🥈22 ·  ⭐ 990) - DART: Dynamic Animation and Robotics Toolkit. <code><a href="http://bit.ly/3rqEWVr">BSD-2</a></code></summary>

- [GitHub](https://github.com/dartsim/dart) (👨‍💻 73 · 🔀 280 · 📦 8 · 📋 670 - 23% open · ⏱️ 09.08.2025):

	```
	git clone https://github.com/dartsim/dart
	```
</details>
<details><summary><b><a href="https://github.com/google-deepmind/mujoco_warp">MuJoCo Wrap</a></b> (🥈18 ·  ⭐ 670 · 🐣) - GPU-optimized version of the MuJoCo physics simulator, designed for NVIDIA hardware. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/google-deepmind/mujoco_warp) (👨‍💻 23 · 🔀 64 · 📋 170 - 30% open · ⏱️ 13.08.2025):

	```
	git clone https://github.com/google-deepmind/mujoco_warp
	```
</details>
<details><summary><b><a href="https://nvidia-omniverse.github.io/PhysX/">PhysX 5</a></b> (🥉16 ·  ⭐ 4.1K) - NVIDIA PhysX SDK. <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub](https://github.com/NVIDIA-Omniverse/PhysX) (👨‍💻 7 · 🔀 490 · 📋 160 - 24% open · ⏱️ 22.07.2025):

	```
	git clone https://github.com/NVIDIA-Omniverse/PhysX
	```
</details>
<details><summary><b><a href="https://nvidia-omniverse.github.io/PhysX/">PhysX SDK (legacy)</a></b> (🥉15 ·  ⭐ 3.4K · 💤) - NVIDIA PhysX SDK. <code><a href="https://tldrlegal.com/search?q=NVIDIA%20Omniverse%20License%20Agreement">NVIDIA Omniverse License Agreement</a></code></summary>

- [GitHub](https://github.com/NVIDIAGameWorks/PhysX) (👨‍💻 3 · 🔀 780 · 📋 620 - 50% open · ⏱️ 09.11.2022):

	```
	git clone https://github.com/NVIDIAGameWorks/PhysX
	```
</details>
<details><summary><b><a href="https://gazebosim.org/">TPE (part of gz-physics)</a></b> (🥉15 ·  ⭐ 72) - Abstract physics interface designed to support simulation and rapid development of robot applications. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/gazebosim/gz-physics) (👨‍💻 47 · 🔀 47 · 📋 150 - 50% open · ⏱️ 06.08.2025):

	```
	git clone https://github.com/gazebosim/gz-physics
	```
</details>
<details><summary><b><a href="https://raisim.com/">RaiSim</a></b> (🥉14 ·  ⭐ 380) - Visit www.raisim.com. <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- [GitHub](https://github.com/raisimTech/raisimLib) (👨‍💻 16 · 🔀 98 · 📥 220 · 📋 400 - 14% open · ⏱️ 19.05.2025):

	```
	git clone https://github.com/raisimTech/raisimlib
	```
</details>
<details><summary><b><a href="http://www.ode.org/">ODE</a></b> (🥉11 ·  ⭐ 170 · 💤) - Open Dynamics Engine (ODE) github mirror from https://bitbucket.org/odedevs/ode. <code><a href="https://tldrlegal.com/search?q=gnu-gpl%20and%20BSD-3-clause">gnu-gpl and BSD-3-clause</a></code></summary>

- [GitHub](https://github.com/thomasmarsh/ODE) (👨‍💻 34 · 🔀 34 · 📋 3 - 66% open · ⏱️ 14.01.2024):

	```
	git clone https://github.com/thomasmarsh/ODE
	```
</details>
<details><summary><b><a href="https://github.com/NVIDIAGameWorks/FleX">FleX</a></b> (🥉9 ·  ⭐ 730 · 💤) -  <code><a href="https://tldrlegal.com/search?q=Nvidia%20Software%20License">Nvidia Software License</a></code></summary>

- [GitHub](https://github.com/NVIDIAGameWorks/FleX) (👨‍💻 2 · 🔀 99 · 📋 130 - 68% open · ⏱️ 15.04.2021):

	```
	git clone https://github.com/NVIDIAGameWorks/FleX
	```
</details>
<details><summary><b><a href="https://github.com/YunzhuLi/PyFleX">PyFleX</a></b> (🥉7 ·  ⭐ 130 · 💤) - Customized Python APIs for NVIDIA FleX. <code><a href="https://tldrlegal.com/search?q=Nvidia%20Software%20License">Nvidia Software License</a></code></summary>

- [GitHub](https://github.com/YunzhuLi/PyFleX) (🔀 27 · 📋 10 - 40% open · ⏱️ 17.08.2021):

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

<details><summary><b><a href="https://docs.o3de.org/docs/atom-guide/">Atom</a></b> (🥇30 ·  ⭐ 8.5K · 📈) - Open 3D Engine (O3DE) is an Apache 2.0-licensed multi-platform 3D engine that enables developers and content creators.. <code><a href="https://tldrlegal.com/search?q=Apache-2.0%20and%20MIT">Apache-2.0 and MIT</a></code></summary>

- [GitHub](https://github.com/o3de/o3de) (👨‍💻 310 · 🔀 2.3K · 📥 10K · 📋 7.8K - 42% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://www.ogre3d.org/">OGRE</a></b> (🥈28 ·  ⭐ 4.3K) - scene-oriented, flexible 3D engine (C++, Python, C#, Java). <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/OGRECave/ogre) (👨‍💻 340 · 🔀 1K · 📥 3.7K · 📋 910 - 15% open · ⏱️ 24.07.2025):

	```
	git clone https://github.com/OGRECave/ogre
	```
</details>
<details><summary><b><a href="https://cyberbotics.com/">Wren (Webots)</a></b> (🥈28 ·  ⭐ 3.7K) - Webots Robot Simulator. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/cyberbotics/webots) (👨‍💻 140 · 🔀 1.8K · 📥 1.7M · 📋 1.9K - 11% open · ⏱️ 03.08.2025):

	```
	git clone https://github.com/cyberbotics/webots
	```
</details>
<details><summary><b><a href="https://www.khronos.org/">Vulkan</a></b> (🥉24 ·  ⭐ 3.4K) - Open-Source Vulkan C++ API. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/KhronosGroup/Vulkan-Hpp) (👨‍💻 94 · 🔀 330 · 📋 630 - 1% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/KhronosGroup/Vulkan-Hpp
	```
</details>
<details><summary><b><a href="https://www.khronos.org/">OpenGL</a></b> (🥉24 ·  ⭐ 3.4K) - Open-Source Vulkan C++ API. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub](https://github.com/KhronosGroup/Vulkan-Hpp) (👨‍💻 94 · 🔀 330 · 📋 630 - 1% open · ⏱️ 12.08.2025):

	```
	git clone https://github.com/KhronosGroup/Vulkan-Hpp
	```
</details>
<details><summary><b><a href="http://pyrender.readthedocs.io/">PyRender</a></b> (🥉20 ·  ⭐ 1.4K · 💤) - Easy-to-use glTF 2.0-compliant OpenGL renderer for visualization of 3D scenes. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/mmatl/pyrender) (👨‍💻 23 · 🔀 240 · 📦 3.1K · 📋 240 - 68% open · ⏱️ 30.04.2022):

	```
	git clone https://github.com/mmatl/pyrender
	```
- [PyPi](https://pypi.org/project/pyrender):
	```
	pip install pyrender
	```
</details>
<details><summary><b><a href="https://docs.unity3d.com/Manual/NativePluginInterface.html">Unity Rendering Plugin</a></b> (🥉13 ·  ⭐ 900) - C++ Rendering Plugin example for Unity. <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub](https://github.com/Unity-Technologies/NativeRenderingPlugin) (👨‍💻 13 · 🔀 180 · 📋 29 - 89% open · ⏱️ 07.05.2025):

	```
	git clone https://github.com/Unity-Technologies/NativeRenderingPlugin
	```
</details>

---

## Resources

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

### How to Contribute

Contributions are encouraged and always welcome! If you like to add or update projects, choose one of the following ways:

- Open an issue by selecting one of the provided categories from the [issue page](https://github.com/knmcguire/best-of-robot-simulators/issues/new/choose) and fill in the requested information.
- Modify the [projects.yaml](https://github.com/knmcguire/best-of-robot-simulators/blob/main/projects.yaml) with your additions or changes, and submit a pull request. This can also be done directly via the [Github UI](https://github.com/knmcguire/best-of-robot-simulators/edit/main/projects.yaml).

If you like to contribute to or share suggestions regarding the project metadata collection or markdown generation, please refer to the [best-of-generator](https://github.com/best-of-lists/best-of-generator) repository. If you like to create your own best-of list, we recommend to follow [this guide](https://github.com/best-of-lists/best-of/blob/main/create-best-of-list.md).

For more information on how to add or update projects, please read the [contribution guidelines](https://github.com/knmcguire/best-of-robot-simulators/blob/main/CONTRIBUTING.md). By participating in this project, you agree to abide by its [Code of Conduct](https://github.com/knmcguire/best-of-robot-simulators/blob/main/.github/CODE_OF_CONDUCT.md).

## License

[![CC0](https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg)](https://creativecommons.org/licenses/by-sa/4.0/)
