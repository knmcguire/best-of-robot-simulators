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
    <a href="#Contents" title="Project Count"><img src="https://img.shields.io/badge/projects-110-blue.svg?color=5ac4bf"></a>
    <a href="#Contribution" title="Contributions are welcome"><img src="https://img.shields.io/badge/contributions-welcome-green.svg"></a>
    <a href="https://github.com/knmcguire/best-of-robot-simulators/releases" title="Best-of Updates"><img src="https://img.shields.io/github/release-date/knmcguire/best-of-robot-simulators?color=green&label=updated"></a>
</p>

This curated list contains 110 awesome simulator projects with a total of 230 stars grouped into 11 categories. All projects are ranked by a project-quality score, which is calculated based on various metrics automatically collected from GitHub and different package managers. If you like to add or update projects, feel free to open an [issue](https://github.com/knmcguire/best-of-robot-simulators/issues/new/choose), submit a [pull request](https://github.com/knmcguire/best-of-robot-simulators/pulls), or directly edit the [projects.yaml](https://github.com/knmcguire/best-of-robot-simulators/edit/main/projects.yaml). Contributions are very welcome!

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
- [Robotic simulators in 2D](#robotic-simulators-in-2d) _3 projects_
- [Aerial Robotics Simulators](#aerial-robotics-simulators) _15 projects_
- [Maritime Robotics Simulators](#maritime-robotics-simulators) _8 projects_
- [Automotive Simulators](#automotive-simulators) _1 projects_
- [Space Robotics Simulators](#space-robotics-simulators) _2 projects_
- [AI training Simulators](#ai-training-simulators) _28 projects_
- [Other Domain Specific Simulators](#other-domain-specific-simulators) _3 projects_
- [Game engines](#game-engines) _6 projects_
- [Physics Engines](#physics-engines) _17 projects_
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

<details><summary><b><a href="http://robwork.dk/">Robwork</a></b> (🥇8 ·  ⭐ 33 · 💤) - RobWork is a collection of C++ libraries for simulation and control of robot systems, see http://robwork.dk To get.. <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitLab](https://gitlab.com/sdurobotics/RobWork) (🔀 39 · 📋 110 - 15% open · ⏱️ 07.04.2016):

	```
	git clone https://gitlab.com/sdurobotics/RobWork
	```
</details>
<details><summary><b><a href="https://gitlab.com/robocup-sim/SimSpark">SimSpark</a></b> (🥇7 ·  ⭐ 22 · 💤) - A generic physical simulator. <code><a href="https://tldrlegal.com/search?q=Missing">Missing</a></code></summary>

- [GitLab](https://gitlab.com/robocup-sim/SimSpark) (🔀 8 · 📋 57 - 49% open · ⏱️ 13.10.2017):

	```
	git clone https://gitlab.com/robocup-sim/SimSpark
	```
</details>
<details><summary><b><a href="https://mujoco.org/">mujoco</a></b> (🥇7) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 130 · 📦 5.7K):

	```
	git clone https://github.com/google-deepmind/mujoco
	```
</details>
<details><summary><b><a href="https://www.argos-sim.info/">ARGoS</a></b> (🥈3) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 25):

	```
	git clone https://github.com/ilpincy/argos3
	```
</details>
<details><summary><b><a href="https://o3de.org/">O3DE for Robotics</a></b> (🥈3) -  <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Apache-2">MIT and Apache-2</a></code></summary>

- [GitHub]() (👨‍💻 340):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://gazebosim.org/home">Gazebo</a></b> (🥈2) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 190):

	```
	git clone https://github.com/gazebosim/gz-sim
	```
</details>
<details><summary><b><a href="https://openrave.org/">OpenRAVE</a></b> (🥈2) -  <code><a href="https://tldrlegal.com/search?q=Apache-2%20and%20LGPL-3">Apache-2 and LGPL-3</a></code></summary>

- [GitHub]() (👨‍💻 120):

	```
	git clone https://github.com/rdiankov/openrave
	```
</details>
<details><summary><b><a href="https://urlab-sim.github.io/UnrealRoboticsLab/">Unreal Robotics Lab</a></b> (🥈2) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub]() (👨‍💻 3):

	```
	git clone https://github.com/URLab-Sim/UnrealRoboticsLab
	```
</details>
<details><summary><b><a href="https://www.cyberbotics.com/">Webots</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 140):

	```
	git clone https://github.com/cyberbotics/webots
	```
</details>
<details><summary><b><a href="http://coppeliarobotics.com/">CoppeliaSim core library</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=gnu-gpl">gnu-gpl</a></code></summary>

- [GitHub]() (👨‍💻 3):

	```
	git clone https://github.com/CoppeliaRobotics/coppeliaSimLib
	```
</details>
<details><summary><b><a href="http://coppeliarobotics.com/">CoppeliaSim</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://developer.nvidia.com/isaac/sim">NVIDIA Isaac Sim</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Apache%202.0%20and%20NVIDIA%20Omniverse%20License%20Agreement">Apache 2.0 and NVIDIA Omniverse License Agreement</a></code></summary>

- [GitHub]() (👨‍💻 5):

	```
	git clone https://github.com/isaac-sim/IsaacSim
	```
</details>
<details><summary><b><a href="https://new.abb.com/products/robotics/software-and-digital/robotstudio">RobotBuilder</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Commercial%20software">Commercial software</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://robodk.com/">RoboDK</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.mathworks.com/products/robotics.html">MATLAB Robotics Systems Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://www.robotec.ai/products">RoSi</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://newton-physics.github.io/newton/">Newton (Physics)</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 85):

	```
	git clone https://github.com/newton-physics/newton
	```
</details>
<details><summary><b><a href="https://prototwin.com/">ProtoTwin</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=proprietary%20license">proprietary license</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://gears.aposteriori.com.sg/">Gears</a></b> (🥉1) -  <code><a href="http://bit.ly/2M0xdwT">GPL-3.0</a></code></summary>

- [GitHub]() (👨‍💻 14):

	```
	git clone https://github.com/QuirkyCort/gears
	```
</details>
<details><summary>Show 4 hidden projects...</summary>

- <b><a href="https://classic.gazebosim.org/">Gazebo Classic</a></b> (🥈4) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="http://morse-simulator.github.io/">Morse</a></b> (🥈2) -  <code><a href="https://tldrlegal.com/search?q=OFL-1.1">OFL-1.1</a></code>
- <b><a href="https://blog.openai.com/roboschool/">Roboschool</a></b> (🥈2) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
- <b><a href="https://simbad.sourceforge.net/">Simbad</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=GNU-gpl2">GNU-gpl2</a></code>
</details>
<br>

## Robotic simulators in 2D

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Robotic simulators that only work in a 2D environment, for instance navigation_

<details><summary><b><a href="https://ir-sim.readthedocs.io/en">IR-SIM</a></b> (🥇5) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 15 · 📦 16):

	```
	git clone https://github.com/hanruihua/ir-sim
	```
- [PyPi](https://pypi.org/project/ir-sim) (📥 1.6K / month):
	```
	pip install ir-sim
	```
</details>
<details><summary><b><a href="https://pyrobosim.readthedocs.io/">pyrobosim</a></b> (🥈3) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 20 · 📦 9):

	```
	git clone https://github.com/sea-bass/pyrobosim
	```
- [PyPi](https://pypi.org/project/pyrobosim) (📥 500 / month):
	```
	pip install pyrobosim
	```
</details>
<details><summary><b><a href="https://flatland-simulator.readthedocs.io/">Flatland</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 14):

	```
	git clone https://github.com/avidbots/flatland
	```
</details>
<br>

## Aerial Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for aerial robotics_

<details><summary><b><a href="https://www.flightgear.org/">Flightgear</a></b> (🥇8 ·  ⭐ 74 · 💤) - FlightGear open-source flight simulator [flightgear.org](https://www.flightgear.org). <code><a href="https://tldrlegal.com/search?q=gnu-gpl2">gnu-gpl2</a></code></summary>

- [GitLab](https://gitlab.com/flightgear/flightgear) (🔀 56 · 📋 610 - 39% open · ⏱️ 04.03.2015):

	```
	git clone https://gitlab.com/flightgear/flightgear
	```
</details>
<details><summary><b><a href="https://cosys-lab.github.io/Cosys-AirSim/">Cosys-AirSim</a></b> (🥈4) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 270):

	```
	git clone https://github.com/Cosys-Lab/Cosys-AirSim
	```
</details>
<details><summary><b><a href="https://codexlabsllc.github.io/Colosseum/">Colosseum</a></b> (🥈4) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 260):

	```
	git clone https://github.com/CodexLabsLLC/Colosseum
	```
</details>
<details><summary><b><a href="https://uzh-rpg.github.io/flightmare/">Flightmare</a></b> (🥈2) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 7):

	```
	git clone https://github.com/uzh-rpg/flightmare
	```
</details>
<details><summary><b><a href="https://flightgoggles.mit.edu/">FlightGoggles</a></b> (🥈2) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 9):

	```
	git clone https://github.com/mit-aera/FlightGoggles
	```
</details>
<details><summary><b><a href="https://utiasdsl.github.io/gym-pybullet-drones/">Gym Pybullet Drones</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 24):

	```
	git clone https://github.com/utiasDSL/gym-pybullet-drones
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
<details><summary><b><a href="https://pegasussimulator.github.io/PegasusSimulator/">Pegasus Simulator</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 6):

	```
	git clone https://github.com/PegasusSimulator/PegasusSimulator
	```
</details>
<details><summary><b><a href="https://ntnu-arl.github.io/aerial_gym_simulator/">Aerial Gym Simulator</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 4):

	```
	git clone https://github.com/ntnu-arl/aerial_gym_simulator
	```
</details>
<details><summary><b><a href="http://wfk.io/neuroflight/">Gymfc</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]():

	```
	git clone https://github.com/wil3/gymfc/
	```
</details>
<details><summary><b><a href="https://iamaisim.github.io/ProjectAirSim/">Project AirSim</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 17):

	```
	git clone https://github.com/iamaisim/ProjectAirSim
	```
</details>
<details><summary><b><a href="https://optim.aero/px4silsimulink.html">optimAero PX4</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]():

	```
	git clone https://github.com/optimAero/optimAeroPX4SIL
	```
</details>
<br>

## Maritime Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for maritime robotics_

<details><summary><b><a href="https://byu-holoocean.github.io/holoocean-docs/">HoloOcean</a></b> (🥇13 ·  ⭐ 98) - A UE5 based simulator for marine perception and autonomy, with multi-agent communications and many common underwater.. <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Unreal%20Engine%20EULA">MIT and Unreal Engine EULA</a></code></summary>

- [GitHub]() (👨‍💻 11 · 🔀 47 · 📋 180 - 10% open · ⏱️ 05.02.2026):

	```
	git clone https://github.com/byu-holoocean/HoloOcean
	```
</details>
<details><summary><b><a href="http://dave-ros2.notion.site">DAVE for ROS 2</a></b> (🥈2) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub]():

	```
	git clone https://github.com/OES-Lab/dave
	```
</details>
<details><summary><b><a href="https://field-robotics-lab.github.io/dave.doc/">DAVE</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 25):

	```
	git clone https://github.com/Field-Robotics-Lab/dave
	```
</details>
<details><summary><b><a href="https://stonefish.readthedocs.io/">Stonefish</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 13):

	```
	git clone https://github.com/patrykcieslak/stonefish
	```
</details>
<details><summary>Show 4 hidden projects...</summary>

- <b><a href="https://uuvsimulator.github.io/">UUV Simulator</a></b> (🥈3) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code>
- <b><a href="https://www.irs.uji.es/uwsim/wiki/index.php?title=Installing_UWSim">UWSim</a></b> (🥈2) -  <code><a href="https://tldrlegal.com/search?q=GNU-gpl">GNU-gpl</a></code>
- <b><a href="https://bitbucket.org/whoidsl/ds_sim/src/master/">ds sim</a></b> -  <code>Unlicensed</code>
- <b><a href="https://sourceforge.net/projects/usarsim/">USARSim</a></b> -  <code>Unlicensed</code>
</details>
<br>

## Automotive Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for automotive_

<details><summary><b><a href="https://carla.org/">Carla</a></b> (🥇7) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 200 · 📦 1.1K):

	```
	git clone https://github.com/carla-simulator/carla
	```
</details>
<br>

## Space Robotics Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulator frameworks made especially for space robotics_

<details><summary><b><a href="https://github.com/OmniLRS/OmniLRS/wiki">OmiLRS</a></b> (🥇2) -  <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub]() (👨‍💻 20):

	```
	git clone https://github.com/OmniLRS/OmniLRS
	```
</details>
<details><summary><b><a href="https://avslab.github.io/bsk_rl/">BSK-RL</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 10):

	```
	git clone https://github.com/AVSLab/bsk_rl
	```
</details>
<br>

## AI training Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Simulations made for training for AI-agents like reinforcement learning_

<details><summary><b><a href="https://gymnasium.farama.org/">Gymnasium</a></b> (🥇16) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 590 · 📦 23K):

	```
	git clone https://github.com/Farama-Foundation/Gymnasium
	```
- [PyPi](https://pypi.org/project/gymnasium) (📥 8.4M / month):
	```
	pip install gymnasium
	```
</details>
<details><summary><b><a href="https://robotics.farama.org/">Gymnasium Robotics</a></b> (🥇9) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 41 · 📦 350):

	```
	git clone https://github.com/Farama-Foundation/Gymnasium-Robotics
	```
- [PyPi](https://pypi.org/project/gymnasium-robotics) (📥 40K / month):
	```
	pip install gymnasium-robotics
	```
</details>
<details><summary><b><a href="https://github.com/mujocolab/mjlab">mjlab</a></b> (🥇8) -  <code><a href="http://bit.ly/3nYMfla">Apache-2</a></code></summary>

- [GitHub]() (👨‍💻 53):

	```
	git clone https://github.com/mujocolab/mjlab
	```
- [PyPi](https://pypi.org/project/mjlab) (📥 89K / month):
	```
	pip install mjlab
	```
</details>
<details><summary><b><a href="https://metaworld.farama.org/">Metaworld</a></b> (🥇8) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 46 · 📦 110):

	```
	git clone https://github.com/Farama-Foundation/Metaworld
	```
- [PyPi](https://pypi.org/project/metaworld) (📥 35K / month):
	```
	pip install metaworld
	```
</details>
<details><summary><b><a href="https://jaxsim.readthedocs.io">jaxsim</a></b> (🥈7) - A differentiable physics engine and multibody dynamics library for control and robot learning. <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 16 · 📦 4):

	```
	git clone https://github.com/ami-iit/jaxsim
	```
- [PyPi](https://pypi.org/project/jaxsim) (📥 1.2K / month):
	```
	pip install jaxsim
	```
- [Conda](https://anaconda.org/conda-forge/jaxsim) (📥 20K · ⏱️ 29.04.2026):
	```
	conda install -c conda-forge jaxsim
	```
</details>
<details><summary><b><a href="https://ai2thor.allenai.org/">AI2-thor</a></b> (🥈5) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 53 · 📦 400):

	```
	git clone https://github.com/allenai/ai2thor
	```
</details>
<details><summary><b><a href="https://isaac-sim.github.io/IsaacLab">NVIDIA Isaac Sim Isaac Lab</a></b> (🥈4) -  <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub]() (👨‍💻 240):

	```
	git clone https://github.com/isaac-sim/IsaacLab
	```
</details>
<details><summary><b><a href="https://aihabitat.org/">Habitat Sim</a></b> (🥈4) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 63 · 📦 76):

	```
	git clone https://github.com/facebookresearch/habitat-sim
	```
</details>
<details><summary><b><a href="https://sapien.ucsd.edu/">Sapien</a></b> (🥈4) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 20 · 📦 250):

	```
	git clone https://github.com/haosulab/SAPIEN
	```
</details>
<details><summary><b><a href="https://svl.stanford.edu/igibson/">IGibson</a></b> (🥉3) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 34 · 📦 14):

	```
	git clone https://github.com/StanfordVL/iGibson
	```
</details>
<details><summary><b><a href="https://some45bucks.github.io/IsaacLab-HARL/">IsaacLab-HARL</a></b> (🥉3) -  <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub]() (👨‍💻 120):

	```
	git clone https://github.com/some45bucks/IsaacLab-HARL
	```
</details>
<details><summary><b><a href="https://pybullet.org/">PyBullet Gym</a></b> (🥉2) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 14):

	```
	git clone https://github.com/benelot/pybullet-gym
	```
</details>
<details><summary><b><a href="https://procthor.allenai.org/">ProcTHOR</a></b> (🥉2) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 7 · 📦 14):

	```
	git clone https://github.com/allenai/procthor
	```
</details>
<details><summary><b><a href="https://robocasa.ai/">RoboCasa</a></b> (🥉2) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code></summary>

- [GitHub]() (👨‍💻 5):

	```
	git clone https://github.com/robocasa/robocasa
	```
</details>
<details><summary><b><a href="https://docs.kscale.dev/docs/ksim">K-Sim</a></b> (🥉2) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 9):

	```
	git clone https://github.com/kscalelabs/ksim
	```
- [PyPi](https://pypi.org/project/ksim) (📥 180 / month):
	```
	pip install ksim
	```
</details>
<details><summary><b><a href="https://maniskill.ai/">ManiSkill</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 69):

	```
	git clone https://github.com/haosulab/ManiSkill
	```
</details>
<details><summary><b><a href="https://roboverseorg.github.io/">RoboVerse</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 47):

	```
	git clone https://github.com/RoboVerseOrg/RoboVerse
	```
</details>
<details><summary><b><a href="https://loco-mujoco.readthedocs.io/">LocoMuJoCo</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 15 · 📦 8):

	```
	git clone https://github.com/robfiras/loco-mujoco
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/reinforcement-learning.html">Reinforcement Learning Toolbox</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://unrealzoo.site/">UnrealZoo</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 6):

	```
	git clone https://github.com/UnrealZoo/unrealzoo-gym
	```
</details>
<details><summary><b><a href="https://deepdrive.io/">Deepdrive</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 6):

	```
	git clone https://github.com/deepdrive/deepdrive
	```
</details>
<details><summary><b><a href="https://www.dynsyslab.org/safe-robot-learning/">Safe Control Gym</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 20):

	```
	git clone https://github.com/utiasDSL/safe-control-gym
	```
</details>
<details><summary><b><a href="http://gibsonenv.stanford.edu/">Gibson</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 9):

	```
	git clone https://github.com/StanfordVL/GibsonEnv
	```
</details>
<details><summary><b><a href="https://metadriverse.github.io/metadrive-simulator/">MetaDrive</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]():

	```
	git clone https://github.com/metadriverse/metadrive/
	```
</details>
<details><summary><b><a href="https://arnold-benchmark.github.io/">Arnold</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 3):

	```
	git clone https://github.com/arnold-benchmark/arnold
	```
</details>
<details><summary><b><a href="https://playground.mujoco.org/">MuJoCo playground</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]():

	```
	git clone https://github.com/google-deepmind/mujoco_playground/
	```
</details>
<details><summary><b><a href="https://uaibot.github.io/">UAIbot</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 3):

	```
	git clone https://github.com/UAIbot/UAIbotPy
	```
</details>
<details><summary>Show 1 hidden projects...</summary>

- <b><a href="https://www.gymlibrary.dev/">Gym</a></b> (🥈6) -  <code><a href="http://bit.ly/34MBwT8">MIT</a></code>
</details>
<br>

## Other Domain Specific Simulators

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Robotic simulators build for other domains like automotive or space robotics_

<details><summary><b><a href="https://raw.org/research/inverse-kinematics-of-a-stewart-platform/">Stewart Platform Simulator</a></b> (🥈1) -  <code>Unlicensed</code></summary>

- [GitHub]() (📦 4):

	```
	git clone https://github.com/rawify/Stewart.js
	```
</details>
<details><summary><b><a href="https://www.mathworks.com/products/roadrunner.html">Roadrunner</a></b> (🥈1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator">AutoDRIVE Simulator</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 3):

	```
	git clone https://github.com/Tinker-Twins/AutoDRIVE
	```
</details>
<br>

## Game engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_3D engines made for games but can be interfaced with robotic frameworks_

<details><summary><b><a href="https://bevy.org/">Bevy</a></b> (🥇10) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 1.6K · 📦 26K):

	```
	git clone https://github.com/bevyengine/bevy
	```
</details>
<details><summary><b><a href="https://godotengine.org/">Godot</a></b> (🥈5) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 3.9K · 📦 24):

	```
	git clone https://github.com/godotengine/godot
	```
</details>
<details><summary><b><a href="https://o3de.org/">O3DE</a></b> (🥈3) -  <code><a href="https://tldrlegal.com/search?q=MIT%20and%20Apache-2">MIT and Apache-2</a></code></summary>

- [GitHub]() (👨‍💻 340):

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

<details><summary><b><a href="http://stack-of-tasks.github.io/pinocchio/">Pinocchio</a></b> (🥇7) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 120):

	```
	git clone https://github.com/stack-of-tasks/pinocchio
	```
- [PyPi](https://pypi.org/project/pin) (📥 1.1M / month):
	```
	pip install pin
	```
</details>
<details><summary><b><a href="http://bulletphysics.org/">Bullet Physics SDK</a></b> (🥈5) -  <code><a href="https://tldrlegal.com/search?q=zlib">zlib</a></code></summary>

- [GitHub]() (👨‍💻 310 · 📦 24):

	```
	git clone https://github.com/bulletphysics/bullet3
	```
</details>
<details><summary><b><a href="https://crates.io/crates/avian3d">Avian 3D</a></b> (🥈5) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 88 · 📦 380):

	```
	git clone https://github.com/Jondolf/avian
	```
</details>
<details><summary><b><a href="https://drake.mit.edu/">Drake</a></b> (🥈4) -  <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub]() (👨‍💻 280):

	```
	git clone https://github.com/RobotLocomotion/drake
	```
</details>
<details><summary><b><a href="https://projectchrono.org">Project CHRONO</a></b> (🥈4) -  <code><a href="http://bit.ly/3aKzpTv">BSD-3</a></code></summary>

- [GitHub]() (👨‍💻 150):

	```
	git clone https://github.com/projectchrono/chrono
	```
</details>
<details><summary><b><a href="http://www.ode.org/">ODE</a></b> (🥈2) -  <code><a href="https://tldrlegal.com/search?q=gnu-gpl%20and%20BSD-3-clause">gnu-gpl and BSD-3-clause</a></code></summary>

- [GitHub]() (👨‍💻 34):

	```
	git clone https://github.com/thomasmarsh/ODE
	```
</details>
<details><summary><b><a href="http://dartsim.github.io/">DART</a></b> (🥈2) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 78 · 📦 9):

	```
	git clone https://github.com/dartsim/dart
	```
</details>
<details><summary><b><a href="https://www.algoryx.se/agx-dynamics/">AGX Dynamics by Algoryx</a></b> (🥈2) - AGX Dynamics, by Algoryx, is a modular physics simulation toolkit available in C++, C# and Python, on Windows, Mac and.. <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- _No project information available._</details>
<details><summary><b><a href="https://simtk.org/home/simbody">Simbody</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 62):

	```
	git clone https://github.com/simbody/simbody
	```
</details>
<details><summary><b><a href="https://gazebosim.org/">TPE (part of gz-physics)</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 59):

	```
	git clone https://github.com/gazebosim/gz-physics
	```
</details>
<details><summary><b><a href="https://nvidia-omniverse.github.io/PhysX/">PhysX SDK (legacy)</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=NVIDIA%20Omniverse%20License%20Agreement">NVIDIA Omniverse License Agreement</a></code></summary>

- [GitHub]() (👨‍💻 3):

	```
	git clone https://github.com/NVIDIAGameWorks/PhysX
	```
</details>
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
<details><summary><b><a href="https://raisim.com/">RaiSim</a></b> (🥉1) -  <code><a href="https://tldrlegal.com/search?q=Proprietary%20Software%20License">Proprietary Software License</a></code></summary>

- [GitHub]() (👨‍💻 17):

	```
	git clone https://github.com/raisimTech/raisimlib
	```
</details>
<details><summary><b><a href="https://nvidia-omniverse.github.io/PhysX/">PhysX 5</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 11):

	```
	git clone https://github.com/NVIDIA-Omniverse/PhysX
	```
</details>
<br>

## Rendering engines

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

_Rendering engines for robotic simulators_

<details><summary><b><a href="http://pyrender.readthedocs.io/">PyRender</a></b> (🥇12) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 23 · 📦 3.2K):

	```
	git clone https://github.com/mmatl/pyrender
	```
- [PyPi](https://pypi.org/project/pyrender) (📥 740K / month):
	```
	pip install pyrender
	```
</details>
<details><summary><b><a href="https://docs.o3de.org/docs/atom-guide/">Atom</a></b> (🥈3) -  <code><a href="https://tldrlegal.com/search?q=Apache-2.0%20and%20MIT">Apache-2.0 and MIT</a></code></summary>

- [GitHub]() (👨‍💻 340):

	```
	git clone https://github.com/o3de/o3de
	```
</details>
<details><summary><b><a href="https://www.ogre3d.org/">OGRE</a></b> (🥈2) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 340):

	```
	git clone https://github.com/OGRECave/ogre
	```
</details>
<details><summary><b><a href="https://www.khronos.org/">Vulkan</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 110):

	```
	git clone https://github.com/KhronosGroup/Vulkan-Hpp
	```
</details>
<details><summary><b><a href="https://www.khronos.org/">OpenGL</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 110):

	```
	git clone https://github.com/KhronosGroup/Vulkan-Hpp
	```
</details>
<details><summary><b><a href="https://cyberbotics.com/">Wren (Webots)</a></b> (🥉1) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 140):

	```
	git clone https://github.com/cyberbotics/webots
	```
</details>
<details><summary><b><a href="https://docs.unity3d.com/Manual/NativePluginInterface.html">Unity Rendering Plugin</a></b> -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 13):

	```
	git clone https://github.com/Unity-Technologies/NativeRenderingPlugin
	```
</details>
<br>

## Others

<a href="#contents"><img align="right" width="15" height="15" src="https://git.io/JtehR" alt="Back to top"></a>

<details><summary><b><a href="https://highway-env.farama.org/">HighwayEnv</a></b> (🥇8 · 📉) -  <code>Unlicensed</code></summary>

- [GitHub]() (👨‍💻 57 · 📦 310):

	```
	git clone https://github.com/Farama-Foundation/HighwayEnv
	```
- [PyPi](https://pypi.org/project/highway-env) (📥 8K / month):
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
