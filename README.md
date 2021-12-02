# Driver Model with OSI 
This driver model is a closed loop agent model that reacts on other traffic participants and is able to perform basic maneuvers. 
The model is based on the work done by [1] and got extended with an OSI adapter. The model described in [1] is available on [GitHub](https://github.com/ika-rwth-aachen/SimDriver).
The driver model was developed by the institute for automotive engineering, RWTH Aachen University within the scope of the SET Level project.

# Modeling Approach
In the following, the basic maneuvers and the implementation concept are outlined.

## Framework
Before describing the model itself, its framework is briefly described. 
The implementation uses the [OSI Sensor Model Packaging (OSMP)](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging) framework to pack the library as a standardized [FMU](https://fmi-standard.org/). This way, the model may be integrated in any simulation platform that supports the [Open Simulation Interface (OSI)](https://github.com/OpenSimulationInterface/open-simulation-interface) and FMI.
Fig. 1 illustrates the wrapping around the actual behavior model to end up with an encapsulated FMU.  

![osmp](doc/drivermodel_osmp.png)  
Fig. 1: OSMP wrapping of the driver model  

# References
[1] *System Design of an Agent Model for the Closed-Loop Simulation of Relevant Scenarios in the Development of ADS*, 29th Aachen Colloquium 2020, 07.10.2020, Aachen. Jens Klimke, E.Go Moove GmbH; Daniel Becker, Institut für Kraftfahrzeuge (ika); Univ.-Prof. Dr.-Ing. Lutz Eckstein, Insitut für Kraftfahrzeuge (ika)
