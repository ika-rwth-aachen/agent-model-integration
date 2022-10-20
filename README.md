# Driver model for SET Level 4to5
> CI pipeline results:  
> * [**Latest FMU** (Ubuntu 20.04 LTS)](../-/jobs/artifacts/master/raw/lib/ikaDriverAgent.fmu?job=deploy_to_artifacts) 

This driver model is a closed loop agent model that reacts on other traffic participants and is able to perform basic maneuvers. 
The model is based on the work done by [1] and got extended with an OSI adapter. The model described in [1] is available on [GitHub](https://github.com/ika-rwth-aachen/SimDriver).
The driver model was developed by the institute for automotive engineering, RWTH Aachen University within the scope of the SET Level project.

## Modeling Approach
In the following, the basic maneuvers and the implementation concept are outlined.

### Framework
Before describing the model itself, its framework is briefly described. 
The implementation uses the [OSI Sensor Model Packaging (OSMP)](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging) framework to pack the library as a standardized [FMU](https://fmi-standard.org/). This way, the model may be integrated in any simulation platform that supports the [Open Simulation Interface (OSI)](https://github.com/OpenSimulationInterface/open-simulation-interface) and FMI.
Fig. 1 illustrates the wrapping around the actual behavior model to end up with an encapsulated FMU. The input of the FMU consists of an `osi3::SensorView`  for the environment representation and an `osi3::TrafficCommand` which holds information on the agent's task in the simulation run. On the output side the simulator can either use the provided `osi3::TrafficUpdate` to manage the updated pose of the agent or forward the generated `sl4to5::DynamicsRequest` message to another module that then calculates an `osi3::TrafficUpdate` from that.  
Inside the FMU, internal interfaces are used to feed the ika behavior model and then calculate its new position with a simple vehicle model and controllers for pedal values and the steering angle.

![osmp](Documentation/drivermodel_osmp.png)  
Fig. 1: OSMP wrapping of the driver model  

### Behavior Model
The model core is hosted on [GitHub](https://github.com/ika-rwth-aachen/SimDriver) and its basic structure and features are described in this section.
#### Information Flow
An extensive discussion of Fig. 2 can be found in [1]. However, the basic concept of the driver model shall be outlined. 
On the left side of Fig. 2 the input interface is shown. It consists of information on the environment (static + dynamic), the route and the ego vehicle. Inside the model these signals are processed the *Perception* layer. This is currently just a "pass-through" layer, but it would be possible to model the driver's perception ability by disturbing the signals.  
The *Processing* layer takes the environment and traffic data and enriches them, e.g., with TTC or THW measures. Then, the most suitable maneuver is selected and modeled by conscious guiding variables (e.g. a time headway to a leading vehicle that should be maintained). Conscious variables are controlled by the sub-conscious variables acceleration and curvature (*Note:* `Z-micro` corresponds to the `sl4to5::DynamicsRequest` message here).  
The *Action* column is actually located outside the "ika Agent Model" block from Fig. 1, but modeled in the most right block of Fig. 1.

![architecutre](Documentation/04_architecture-en.svg)  
Fig. 2: Behavior model architecture (taken from [2])  

#### Basic Maneuvers
This section should help enlighten some blocks within the *Processing* column of Fig. 2. The driver model is implemented such that basic driving maneuvers are modeled which enables the model to perform most driving tasks that are required in urban scenarios (cf. [1]). Those capabilities or basic maneuvers are illustrated as a state diagram in Fig. 3.

![states](Documentation/states-original.svg)  
Fig. 3: Behavior model basic maneuvers (taken from [2])  

**TODO: brief description of state diagram**

## Parametrization
Currently, the model can only be parameterized directly in the source code. A solution for a FMU based approach will be done by the end of SET Level.  

In the source file [IkaAgent.cpp](src/IkaAgent.cpp) within the `init` function all parameters can be adjusted. Possible parameters of interest might be:
```C++
drParam->velocity.vComfort // The default velocity the driver reaches on a straight road ( in *m/s*)
drParam->velocity.thwMax // The maximum time headway the driver starts to react (in *s*)
drParam->follow.timeHeadway  // The time headway the driver tries to reach during following (in *s*)
```

## Interface

### OSI Input Fields
The following fields are required as OSI inputs for the driver model

```
sensor_view
  host_vehicle_id
  global_ground_truth
    moving_object
      base --> all except base_polygon
      id
      assigned_lane_id --> Is deprecated in osi. Will be changed to classification soon
      vehicle_classification --> maybe fill that as well. The deprecated signal above will be changed
    lane
      id
      classification
        type --> type_intersection is important
        is_host_vehicle_lane
        centerline
        centerline_is_driving_direction
        left_adjacent_lane_id
        right_adjacent_lane_id
        lane_pairing
        right_lane_boundary_id
        left_lane_boundary_id
        subtype
    lane_boundary
      id
      boundary_line
      classification --> not that important for now, but maybe in the future 
    traffic_sign
        main_sign
          base
            position
          classification
            assigned_lane
            type
            value
    traffic_light
      base --> all except base_polygon
      id
      classification
        color
        icon
        assigned_lane_id
    road_marking --> not right now but for stop lines in the future

traffic_command
  action --> the following actions can be considered right now
    acquire_global_position_action
    # path and trajectory are implemented the same as acquire position:
    # the last point of the list is taken and a path along the centerlines
    # ist planned. So it is not really a follow path/trajectory action
    follow_path_action 
    follow_trajectory_action
    speed_action --> the desired velocity is updated
```
### OSI Output Fields
The following fields are filled from the driver model and can be used by the simulator.

```
osi3
  # Values computed by a simple vehicle model and PID controllers for pedal and steering
  traffic_update
    position (x,y)
    velocity (x,y)
    acceleration (x,y)
    orientation (yaw)
    orientation_rate (yaw)

# Can be used when a separate dynamic module is used
# Note: these are *desired* values from the behavior model
setlevel4to5
  dynamic_request
    curvature_target
    longitudinal_acceleration_target
```
## Build
Brief build instructions for compilation with `MSYS2 MinGW 64-bit` on Windows.  
### Initialize Repository
Befor starting the build process, the repositorie's submodules need to be downloaded
```
git submodule update --init --recursive
```

### Protobuf dependency
Due to the usage of the CMake feature 'ExternalProject_Add()', there is no need to download and build protobuf from source source anymore. 
  
### Windows Build FMU
1. Start `MSYS2 MinGW 64-bit` shell
1. Create a `build` directory and enter it:
    ```
    mkdir build && cd build
    ```  

2. Execute CMake:
    ```
    cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Release ..
    ```  

    Please note: The default build directory for the `FMU` is the subfolder `lib/`. If a specific `FMU` output dir shall be used, set the variable `FMU_OUTDIR`, e.g.
    ```
    cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Release -DFMU_OUTDIR=<dir> ..
    ```  

3. Compile the library:
    ```
    make.exe
    ```
    Optional: `make.exe -j4` for building on multiple cores (replace `4` with an arbitrary number).

### Linux Build FMU
1. Create a `build` directory and enter it:
    ```
    mkdir build && cd build
    ```  

2. Execute CMake:
    ```
    cmake -DCMAKE_BUILD_TYPE=Release ..
    ```  

    Please note: The default build directory for the `FMU` is the subfolder `lib/`. If a specific `FMU` output dir shall be used, set the variable `FMU_OUTDIR`, e.g.
    ```
    cmake -DCMAKE_BUILD_TYPE=Release -DFMU_OUTDIR=<dir> ..
    ```  

3. Compile the library:
    ```
    make
    ```
    Optional: `make -j4` for building on multiple cores (replace `4` with an arbitrary number).

## Debugging
The external FMU parameter `debug` enables debugging log information in the `${workspace}/debug` folder as `json` file and holds information about `horizon`, `vehicle_state` and `driver_state` at each timestep.

In addition the plot python scripts in [scripts](scripts) can be used to visualize the debug information with `matplotlib`.
## Licensing
**Distributed under the [MIT License](LICENSE).**

## Credits
This work received funding from the research project
"[SET Level](https://setlevel.de/)" of the [PEGASUS ](https://pegasus-family.de) project family, promoted by the German Federal
Ministry for Economic Affairs and Energy based on a decision of the German Bundestag.
| SET Level | PEGASUS Family | BMWi |
|-----------|----------------|------|
| <a href="https://setlevel.de"><img src="https://setlevel.de/assets/logo-setlevel.svg" width="100" /></a> | <a href="https://pegasus-family.de"><img src="https://setlevel.de/assets/logo-pegasus-family.svg" width="100" /></a> | <a href="https://www.bmwi.de/Redaktion/DE/Textsammlungen/Technologie/fahrzeug-und-systemtechnologien.html"><img src="https://setlevel.de/assets/logo-bmwk-en.svg" width="100" /></a> |

## References
[1] *System Design of an Agent Model for the Closed-Loop Simulation of Relevant Scenarios in the Development of ADS*, 29th Aachen Colloquium 2020, 07.10.2020, Aachen. Jens Klimke, E.Go Moove GmbH; Daniel Becker, Institut für Kraftfahrzeuge (ika); Univ.-Prof. Dr.-Ing. Lutz Eckstein, Insitut für Kraftfahrzeuge (ika)

[2] *Agentenmodell für die Closed-Loop-Simulation von Verkehrszenarien*, ATZelektronik 05 Mai 2021, 16. Jahrgang, S.42-46. Daniel Becker, Jens Klimke, Lutz Eckstein. Link: https://www.springerprofessional.de/agentenmodell-fuer-die-closed-loop-simulation-von-verkehrsszenar/19141908

