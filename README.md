# Driver Model with OSI 
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

![osmp](doc/drivermodel_osmp.png)  
Fig. 1: OSMP wrapping of the driver model  

### Behavior Model
The model core is hosted on [GitHub](https://github.com/ika-rwth-aachen/SimDriver) and its basic structure and features are described in this section.
#### Information Flow
An extensive discussion of Fig. 2 can be found in [1]. However, the basic concept of the driver model shall be outlined. 
On the left side of Fig. 2 the input interface is shown. It consists of information on the environment (static + dynamic), the route and the ego vehicle. Inside the model these signals are processed the *Perception* layer. This is currently just a "pass-through" layer, but it would be possible to model the driver's perception ability by disturbing the signals.  
The *Processing* layer takes the environment and traffic data and enriches them, e.g., with TTC or THW measures. Then, the most suitable maneuver is selected and modeled by conscious guiding variables (e.g. a time headway to a leading vehicle that should be maintained). Conscious variables are controlled by the sub-conscious variables acceleration and curvature (*Note:* `Z-micro` corresponds to the `sl4to5::DynamicsRequest` message here).  
The *Action* column is actually located outside the "ika Agent Model" block from Fig. 1, but modeled in the most right block of Fig. 1.

![architecutre](doc/04_architecture-en.svg)  
Fig. 2: Behavior model architecture (taken from [2])  

#### Basic Maneuvers
This section should help enlighten some blocks within the *Processing* column of Fig. 2. The driver model is implemented such that basic driving maneuvers are modeled which enables the model to perform most driving tasks that are required in urban scenarios (cf. [1]).

![states](doc/states-original.svg)  
Fig. 3: Behavior model basic maneuvers (taken from [2])  

## Licensing
**Distributed under the [MIT License](LICENSE).**

## Credits
This work received funding from the research project
"[SET Level](https://setlevel.de/)" of the [PEGASUS ](https://pegasus-family.de) project family, promoted by the German Federal
Ministry for Economic Affairs and Energy based on a decision of the German Bundestag.
| SET Level | PEGASUS Family | BMWi |
|-----------|----------------|------|
| <a href="https://setlevel.de"><img src="https://setlevel.de/assets/logo-setlevel.svg" width="100" /></a> | <a href="https://pegasus-family.de"><img src="https://setlevel.de/assets/logo-pegasus-family.svg" width="100" /></a> | <a href="https://www.bmwi.de/Redaktion/DE/Textsammlungen/Technologie/fahrzeug-und-systemtechnologien.html"><img src="https://setlevel.de/assets/logo-bmwi-en.svg" width="100" /></a> |


## References
[1] *System Design of an Agent Model for the Closed-Loop Simulation of Relevant Scenarios in the Development of ADS*, 29th Aachen Colloquium 2020, 07.10.2020, Aachen. Jens Klimke, E.Go Moove GmbH; Daniel Becker, Institut für Kraftfahrzeuge (ika); Univ.-Prof. Dr.-Ing. Lutz Eckstein, Insitut für Kraftfahrzeuge (ika)

[2] *Agentenmodell für die Closed-Loop-Simulation von Verkehrszenarien*, ATZelektronik 05 Mai 2021, 16. Jahrgang, S.42-46. Daniel Becker, Jens Klimke, Lutz Eckstein. Link: https://www.springerprofessional.de/agentenmodell-fuer-die-closed-loop-simulation-von-verkehrsszenar/19141908

