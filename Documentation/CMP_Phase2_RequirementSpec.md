## OsiPedestrian: Credible Modelling Process
This Document is based on the Credible Modelling Process (CMP) and fills it from the perspective of the OsiPedestrian. The process description can be found [here](https://gitlab.sl4to5.de/deliverables/credible-simulation-process/credible-simulation-process/-/blob/0182678762e6e3f9910246913259ae5c9fa7313b/credible_simulation_process.md#introduction).

### ToDo / übergreifende Punkte:
* [ ] Dummy

# Phase 2: Requirement Specification

> In this process phase, the requirements for OsiPedestrianling task and the associated goals and criteria are determined from the engineering task, also the description of the product and its relevant environment in which it is operated. An essential step is the clarification of the general conditions, relevant assumptions and requirements that the simulation has to fulfill. The requirements of OsiPedestrianling task are broken down to the individual requirements of the test cases, model, simulation & test environment, and quality assurance.
> 
> * Requirement specification of Modelling task & objectives
> * Requirement specifications for test cases, model, simulation & test environment, quality assurance


## Inputs

> * Requirement specifications for Modelling task and the according environment
>   * Part of design specifications can be included
> * Description of product, environment (meant is the environment of the real product, e.g. for car -> road, climate zone, user group, frequency of use, …)
> * Development objectives of specific engineering task
> * Description of Simulation Task from Single Decision Process or corresponding process
> * Requirements, specifications of topics above
> * Criticality of decision of specific engineering task
> * Company specific regulations regarding Quality Assurance

**Repos of [SUC1](https://gitlab.sl4to5.de/deliverables/use-case-definitions/usecase1/-/blob/master/Milestone2/Documentation/SUC1_CSP_Phase2_RequirementSpec.md) , [SUC2](https://gitlab.sl4to5.de/deliverables/use-case-definitions/usecase2/-/blob/master/Milestone2/Documentation/SUC2_CSP_Phase2_RequirementSpec.md) , [SUC3](https://gitlab.sl4to5.de/deliverables/use-case-definitions/use-case-3/-/blob/master/Milestone2/Documentation/SUC3_CSP_Phase2_RequirementSpec.md)**

**Collection of all Inputs**

### SUC1

#### Übergreifende Anforderungen an alle Modelle / OSMP-FMUs
*  Double Buffering für alle Ausgänge
*  FMU interne Fehler zur Laufzeit sollten erkannt und über den FMU Logging Mechanismus ausgegeben werden. Folgende Fehler sind dabei mindestens zu „entdecken“ und auszugeben:
    *  Fehlende OSI Signale (Eingänge) ohne die das Modell nicht funktionieren kann
    *  Division durch Null (z.B. bei Koordinatensystem Transformation)
    *  OSI Deserialisierungsfehler (z.B. Puffer zu klein, Inkonsistente Eingangsdaten) 
*  Nutzung von Koordinatensystemen und Transformationen gemäß abgestimmten Konventionen (derzeit in Bearbeitung von FZD, etc.)

#### Fußgängermodell

* SUC1-FGM01: Das Fußgängermodell muss über eine Initialisierungsnachricht (tbd) initialisierbar sein.
> Karte, Initialposition, Initialgeschwindigkeit, ... etc.
* SUC1-FGM02: Das Fußgängermodell soll eine konkrete Routenvorgabe umsetzen können. Varianten:
> Vorgabe Zielposition  
> Vorgabe Zielposition und Zielzeitpunkt  
> Vorgabe Zielposition und Bewegungsgeschwindigkeit  
> Zusätzlich zur Zielposition können in jedem Fall noch weitere Routenpunkte vorgegeben werden   
> Eine Routenvorgabe kann zu jedem Zeitpunkt während der Simulation erfolgen und auch überschrieben werden  
> Detaillierte Anforderungen werden in TP3 diskutiert  
* SUC1-FGM03: Das Fußgängermodell soll sich frei auf den Gehwegen bewegen können.
> Ist eine Route vorgegeben, soll der Route gefolgt werden  
> Ist keine Route vorgegeben, soll der Fußgänger eine zufällige Route wählen oder stehen bleiben (tbd, für SUC nicht relevant)  
* SUC1-FGM04: Das Fußgängermodell muss sich an geltende Verkehrsregeln halten.
> Hier insbesondere Vorfahrsregeln (RvL, Vorfahrtsschilder, Zebrastreifen)
* SUC1-FGM05: Das Fußgängermodell soll ein Umschalten des Vehaltens während der Simulation ermöglichen
> Ignorieren von Verkehrsregeln (ja/nein)  
> Weitere Vorgaben gemäß TP3 / TP2  (nicht SUC relevant)
* SUC1-FGM06: Das Fußgängermodell soll verschiedene Bewegungsmodi unterstützen
> Gehen, Joggen, Rennen, spontanes beschleunigen  (ggf. durch Vorgabe Bewegungsgeschwindigkeit abbildbar)
> Weitere Vorgaben gemäß TP3 / TP2  (nicht SUC relevant)  
* SUC1-FGM07: Das Fußgängermodell soll ein Umschalten der Bewegunsmodi während der Simulation ermöglichen
* SUC1-FGM08: Das Fußgängermodell soll Informationen über die Blickrichtung, Armbewegung (tbd) geben können.
* SUC1-FGM09: Das Fußgängermodell muss als OSMP FMU bereitgestellt werden.
* SUC1-FGM10: Das Fußgängermodell muss die OSI::Sensorview als Input verarbeiten können.
> (inkl. Ausgabe SensorViewConfiguration)
* SUC1-FGM11: Das Fußgängermodell muss die Umsetzung des für SUC1 definierten Szenarios ermöglichen.

### SUC2

#### Anforderungen Modelle und Parameter
*  Modelldokumentation muss Angaben zu Grenzwerten bei Simulationszeiten der Modelle enthalten
*  Sensorik und HADf sollen alle mit identischer Simulationsschrittweite von 10 ms ausgeführt werden
*  Bei der Implementierung von OSMP Ausgängen ist die Umsetzung von Double Buffering vorzusehen 
**  https://opensimulationinterface.github.io/osi-documentation/osi-sensor-model-packaging/doc/specification.html
**  https://gitlab.sl4to5.de/deliverables/model/sensor/ideal-object-based-model/-/blob/master/src/OSMPFramework.cpp#L120
**  https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/blob/8c2d33f4e2e211da20378f39d8058bcc07d7a966/examples/OSMPDummySensor/OSMPDummySensor.cpp#L149
*   FMU interne Fehler zur Laufzeit sollten erkannt und über den FMU Logging Mechanismus ausgegeben werden. Folgende Fehler sind dabei mindestens zu „entdecken“ und auszugeben:
**	Fehlende OSI Signale (Eingänge) ohne die das Modell nicht funktionieren kann
**	Division durch Null 
***	Z.B. bei Koordinatensystem Transformation
**	OSI Deserialisierungsfehler 
***	Puffer zu klein
***	Inkonsistente Eingangsdaten (mögliche Ursache: falsche OSMP Size Information, …)
*   Abgestimmte, soweit wie möglich einheitliche, Koordinatensysteme für die Schnittstellen bzw. abgestimmte Koordinatentransformationen

#### Agentenmodell Fußgänger
*  Fußgänger als OSMP-FMU, Windows-fähig (64-bit) oder mit Windows-kompatiblem/plattfomrunabhängigem Source-Code
*  Init: Initialisierung der FMU über FMU-Parameter (siehe https://gitlab.sl4to5.de/deliverables/architecture/osi-sensor-model-packaging/-/merge_requests/2). 
*  Reset/Neuinitialisierung während der Simulation möglich (um Neustart des Szenarios oder
	Szenario(varianten)wechseln bei laufender Simulationsumgebung zu unterstützen)Init/Input: Routenvorgabe wird eingelesen und umgesetzt (Vorgabe Zielposition, Zielzeitpunkt, ggf. mit Routenpunkten, genaue Anforderungen werden in TP3 definiert, OSI Message tbd) 
*  Input: osi:SensorView, TrajectoryCommand via osi::TrafficCommand
*  Output: osi::TrafficUpdate
*  Bewegung auf Gehwegen nach Vorgegebener Trajektorie (Deklaration in OSI tbd)
*  Keine Anforderungen an Animation des Fußgängers

### SUC3



## Process Step Execution

> * Clarification of framework conditions, the relevant assumptions, and the requirements that the simulation (simulation & test environment and model) must fulfill
> * Mapping of requirements to test cases, model & parameters, simulation environment
> * Definition of requirements specifications for process quality

**Collect doing, decisions and reasoning here**


## Outputs

> * Overall Requirement specification for modelling task (also requirements for the overall simulation environment, including operation range, which have to be checked and tested)
> * Requirement specification test cases
> * Requirement specification models (+ parameters)
> * Requirement specification simulation & test environment
> * Requirements according to criticality of decision of simulation task
> * Requirement specification for process quality



**Collection of all Outputs**

| Previous Step | Next Step |
| ------ | ------ |
| [**Analyze**](Documentation/CMP_Phase1_Analyze.md) | [**Design Spec**](Documentation/CMP_Phase3_DesignSpec.md) |
