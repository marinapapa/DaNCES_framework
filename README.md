# DaNCES Framework

This is an agent-based model composed on the DaNCES framework, for DAta-iNspired Collective Escape Simulations of animal groups. In its current state, it is an agent-based model of flocking under predation, adjusted to the collective motion and collective escape of bird flocks. Individuals are moving around in a 3-dimensional space. The user interface is created with [Dear ImGui](https://github.com/ocornut/imgui).

Information about the model are given below, and details about the framework are presented in:

_Papadopoulou M., Hildenbrandt H., Hemelrijk C.K. (2024) DaNCES: a framework for data-inspired agent-based models of collective escape. Accepted._


## Get started

### Linux (assuming apt package manager like Debina, Ubuntu, Mint)

#### Prerequisites

#### _X11_

```bash
sudo aprt install build-essential
sudo apt install git cmake ninia-build curl zip unzip tar
sudo apt install libxinerama-dev libxcursor-dev xorg-dev libglu1-mesa-dev pkg-config
```

#### _wayland_

```bash
sudo aprt install build-essential
sudo apt install git cmake ninia-build curl zip unzip tar
sudo apt install libwayland-dev libxkbcommon-dev wayland-protocols extra-cmake-modules
```

#### Build

```bash
git clone hrrps://github.com/marinapapa/DaNCES_framework.git
cd DaNCES_framework
git submodule update --init --remote --recursive
cd vcpkg
./bootstrap-vcpkg.sh -disableMetrics
cd ..
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

Binaries are placed into the `DaNCES_framework/bin` folder.

#### Run the simulation

```bash
cd bin
./dances
```

### Windows

Pre-compiled binaries for Windows could be found in the 'Release' section of the github repository.
However building from source is recommended...

#### Prerequisites

Install `winget` from the Microsoft store.

```powershell
winget install cmake git
winget install Microsoft.VisualStudio.2022.BuildTools
winget install Microsoft.VisualStudio.2022.Community --silent --override "--wait --quiet --add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended"
```

#### Build

```powershell
git clone hrrps://github.com/marinapapa/DaNCES_framework.git
cd DaNCES_framework
git submodule update --init --remote --recursive
cd vcpkg
./bootstrap-vcpkg.sh -disableMetrics
cd ..
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

Binaries are placed into the `DaNCES_framework/bin` folder.

#### Run the simulation

```bash
cd bin
./dances
```

## 3rd party libraries

* [Intel oneAPI Threading Building Blocks (oneTBB)](https://github.com/oneapi-src/oneTBB), SPDX: Apache-2.0.
* [OpenGL Mathematics (GLM)](https://github.com/g-truc/glm), SPDX: MIT.
* [GLFW](https://www.glfw.org), SPDX: libpng.
* [glad](https://github.com/premake-libs/glad), SPDX: BSD-3-Clause.
* [nlohmann-json](https://github.com/nlohmann/json), SPDX: MIT.
* [Dear ImGui](https://github.com/ocornut/imgui), SPDX: MIT.
* [implot](https://github.com/epezent/implot), SPDX: MIT.

# The model

## _Framework_ 

This model is based on self-organization and includes prey-like and predator-like agents. Agents interact with their surrounding neighbors based on the rules of attraction, alignment and avoidance. Prey-agents (birds) flock together and avoid the predator. Predator-agents chase and attack prey-agents. Catches of prey are not modelled. Each individual behaves according to the `state' it is in, for instance normal flocking or escaping. Each state consists of a collection of specific actions (or Internal State Control units) that an agent follows, for instance alignment and attraction. 

The model consists of 3 timelines: decision, update, and integration. At each decision time step, prey agents may switch to a different state. While being in a state, an agent collects and updates its information about its environment at each update time step. At the beginning of an update step, an agent calculates a steering force that will determine its future position and heading. Between 2 update steps, individuals move according to their steering vector. This happens gradually across several [dt] integration time steps. Thus, a decision step may include several update steps, and an update step includes several integration steps. Some states, such as normal flocking, are transient: a decision step includes one update step; the agent might switch state at each update time step. Other states, such as an escape maneuver, are persistent: a decision step includes several update steps. Their duration is controlled by the user. 

The switch between states depends on a transition matrix that gives a probability of each agent to switch state given its current state and a [stress] value that depends on the distance of each prey agent to the predator. The closer the predator is, the higher the value of stress is.

## _Parameters_
All user-defined parameters are parsed by combining a series of .json files: *config.json* (simulation parameters), *imgui.json* (user interface parameters, for *Dear ImGui*), *prey.json* (prey parameters, here adjusted to starlings) and *predator.json* (predator parameters). Distance is measured in meters [m], time in seconds [s] and angles in degrees [deg].

## _Individual Actions (or ISC units)_

Actions are the basic elements controlling the movement of each agent in the simulations. Each action adds to the steering vector that controls the motion of the agent, so that the final steering force is the weighted sum of all actions. Each action has each own user-defined parameters. Multiple actions are combined to create *states*.

The majority of actions control the interactions between agents (coordination between prey-agents, escape actions of prey-agents from the predator-agents, and hunting actions of the predator-agents towards prey-agents). Some actions control the motion specifics of the agents (e.g., random error in movement, attraction to a given altitude). The model is based on **topological** interactions.

Default prey-agents actions:
* Avoid actions: 
    * __avoid_n_position__: the individual is repulsed by the position of its _topo_ closest neighbors if they are in distance smaller than _minsep_. Parameters: _topo_ (number of neighbors to take interact with), _fov_ (field of view), _maxdist_, _minsep_, _w_.
  
* Align actions:
    * __align_n__: the individual is attracted towards the average heading of its _topo_ closest neighbors. Parameters: _topo_ (number of neighbors to take interact with), _fov_ (field of view), _maxdist_, _w_.

* Cohere actions:
    * __cohere_centroid_distance__: the individual is attracted towards the center of the positions of its _topo_ closest neighbors. The strength (w) of this action is scaled on the distance between the focal individual and the center of its neighbors. Parameters: _topo_ (number of neighbors to take interact with), _fov_ (field of view), _maxdist_, _w_.

* Non-interacting actions:
    * __wiggle__: the individuals turn by a random angle controlled by the weight of this perpendicular to the agent' heading steering force, sampled from the range [-w,w]_. Parameters: _w_.

* Escape actions:
    * __turn_X__: a list of discrete turning maneuvers.
    * __dive__: the individuals dive to avoid the predator (the closer the predator the larger the dive).
    * __copy_escape__: if one of the _topo_ closest neighbours of an agent is in an escape state, the agent enters the same state at the next reaction step.

Default predator-agents actions:
* Avoid actions: 
    * __avoid_closest_prey__: the predator avoids the position of its closest prey. Used in states where the predator should not hunt the prey. Parameters: _w_.
    * __set_retreat__: the predator is repositioned at a given distance away from the flock and given a new speed. Parameters: _distAway_, _speed_.
   
* Non-interacting actions:
    * __wiggle__: the individuals turn by a random angle controlled by the weight of this perpendicular to the agent' heading steering force, sampled from the range [-w,w]_. Parameters: _w_.
    * __hold_current__: the agents tries to hold a constant position. Parameters: _w_.
    
* Hunting actions:
    * __select_flock__: the predator chooses a flock as its target. Selection can be made based on the flock's size or proximity. Parameters: _selection_.
    * __position_to_attack__: the predator is positioned in a given position relative to the center of the flock. Parameters: _rel_pos_.
    * __shadowing__: the predator follows (or tries to follow) its target flock from a given angle and distance, keeping a constant speed that scales from the speed of its target. Parameters: _bearing_ (angle starting from the flock's heading), _distance_, _placement_ (whether to automatically reposition the predator to the given shadowing position), _prey_speed_scale_, _w_.
    *  __chase_closest_prey__: the predator turns towards the closest prey-agent at every time point (target) and moves with a speed that scales from this agent's speed. Parameters: _prey_speed_scale_, _w_. 
    *  __lock_on_closest_prey__: the predator turns towards its target prey-agents, selected at the beginning of the attack (state-switch) as the closest prey. The predator moves with a speed that scales from this target's speed. Parameters: _prey_speed_scale_, _w_. 
    
_Note_: More actions are included in the code (__actions__ folder) but are not active in the present state of the model. A description of each is given in the _Table1_ISCUnits.pdf_.

## _Individual States_

States in the model are defined as combinations of actions. *Persistent* states have a user-defined duration, whereas *Transient* states can change after a time-step. The transition between states is controlled by the user-defined transition matrix. Prey-agents start from a normal flocking state, and as their stress increases, they switch to an alarmed flocking state and from there they may switch to an escape state. After that, they get into a persistant flocking state that works as escape penalty. 

A collection of states that have a similar role (for instance escape-related states) can be combined in a multi-state. In this case, the individual decides to switch to an escape state, and the selection of the specific state from the multi-state can be based on other parameters (for instance, whether the predator attacks from above). In the current model, it is selected based on a user-defined probability (_probs_ parameter in escape _selector_).
The hunting strategy of the predator is built on a chain of persistent states based on which the predator is positioned around the flock, follows it with constant speed, then attacks, and retreats.

## _Initialization_

The initial conditions of the agents are controlled by the user. Prey agents are initiated in a 'random' formation (within radius), in a 'flock' formation (within a sphere and with similar headings), or from a csv file.

### __Application keys:__ 

1. Space: pause/continue simulation
2. T: shows/hides the position trail of each starling-agent
3. Ctrl+1: sets camera to be stable on the ground
4. Ctrl+2: sets camera to bottom view following the flock
5. Ctrl+3: sets camera to top view following the flock
6. Ctrl+4: sets camera behind the flock
7. W: moves television camera -1 in x axis
8. S: moves television camera +1 in x axis
9. A: move television camera +1 in z axis
10. D: move television camera -1 in z axis
11. B: darkens background
12. Shift+B: brightens background

## _Data Collection_ 

The model exports data in _.csv_ format. It creates a unique folder within the user-defined *data_folder* (in the config.json) in the repo's subdirectory *bin/sim_data*. In the created folder, it creates one or several .csv files for each Observer, as defined in the config file. Available observers are: 'TimeSeries', 'GroupData', and 'Diffusion'. The sampling frequency and output name of each csv file is also controled by the config. The whole composed config file is also copied to the saving directory. 


## Authors
* **Dr. Marina Papadopoulou** - Contact at: <m.papadopoulou.rug@gmail.com>
* **Dr. Hanno Hildenbrandt** 
