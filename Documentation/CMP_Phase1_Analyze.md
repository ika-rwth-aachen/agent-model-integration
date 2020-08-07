## OsiPedestrian: Credible Modelling Process
This Document is based on the Credible Modelling Process (CMP) and fills it from the perspective of the OsiPedestrian. The process description can be found [here](https://gitlab.sl4to5.de/deliverables/credible-simulation-process/credible-simulation-process/-/blob/0182678762e6e3f9910246913259ae5c9fa7313b/credible_simulation_process.md#introduction).

### ToDo / übergreifende Punkte:
* [ ] Dummy


# Phase 1: Analyze the Engineering Task

> In this process phase, the information from the higher-level Credible Simulation Process is further specified and detailed with regard to the engineering task. It includes a description of product and relevant environment (meant is the environment of the real product, e.g. for car -> road, climate zone, user group, frequency of use, …), development objectives, KPI, and further criteria. The content handled in this phase may be extended, but there must be no inconsistencies with the content of the Credible Simulation Process, otherwise a change request must be issued for the Credible Simulation Process.
> 
> * Description of system under test (product) and relevant environment (meant is the environment of the real product, e.g. for car -> road, climate zone, user group, frequency of use, …), development objectives, KPI, Criteria


## Inputs

> * Description of product, environment (meant is the environment of the real product, e.g. for car -> road, climate zone, user group, frequency of use, …)
> * Overall Development objectives
> * Overall Requirements, specifications of topics above
> * Criticality of decision of engineering task

**Repos of [SUC1](https://gitlab.sl4to5.de/deliverables/use-case-definitions/usecase1/-/blob/master/Milestone2/Documentation/SUC1_CSP_Phase1_Analyze.md) , [SUC2](https://gitlab.sl4to5.de/deliverables/use-case-definitions/usecase2/-/blob/master/Milestone2/Documentation/SUC2_CSP_Phase1_Analyze.md) , [SUC3](https://gitlab.sl4to5.de/deliverables/use-case-definitions/use-case-3/-/blob/master/Milestone2/Documentation/SUC3_CSP_Phase1_Analyze.md)**

**Collection of all Inputs**

### SUC1

### Beschreibung des zu entwickelnden Produkts
Es wird hier hypothetisch angenommen, dass eine hochautomatisierte Fahrfunktion L4/L5 für den urbanen Verkehr entwickelt wird.

#### Übergreifende Entwicklungsziele
Die zu entwickelnde Fahrfunktion muss unter anderem die sichere Überquerung von Kreuzungen beherrschen. Die dafür notwendigen Fähigkeiten werden während des Entwicklungsprozesses kontinuierlich getestet und sind letztendlich auch Teil der Absicherung. Eine Methode zur Identifikation von relevanten Szenarien für das entwicklungsbegleitende Testen sowie die Absicherung stellt die Kritikalitätsanalyse dar.

Eine expemplarische Kritikalitätsanalyse soll im Rahmen des **Simulation Use Case 1** (SUC1) aufgebaut und durchgeführt werden.

#### Kritikalitätsanalyse

Die Kritikalitätsanalyse beschreibt die systematische Untersuchung des Szenarienraums auf kritische Verkehrssituationen zur Identifikation von kritikalitätserhöhenden Parametern / Kombinationen von Parametern des Szenarios und auch des untersuchten Systems entsprechend einer oder mehrer Kritikalitätsmaße.

Eine erweiterte Beschreibung und Einordnung des SUC1 im Zusammenspiel mit SUC2 und 3 ist zu finden unter:  
https://gitlab.sl4to5.de/pm/gesamtprojekt/documents/-/blob/50957d50641464b6dc1c5dbd072011a724ed1b1f/10_Projektinput/Usecases-Description.adoc

### SUC2

#### Beschreibung des zu entwickelnden Produktes
Es wird hier hypothetisch angenommen, dass eine hochautomatisierte Fahrfunktion L4/L5 für den urbanen Verkehr entwickelt wird.
Das System-Under-Test (SUT) in SUC2 ist die HADf.

#### Übergreifende Entwicklungsziele
SUC2 verfolgt für MS2 als primäres Ziel den Beitrag der Simulation für den Test von Fahrzeugsystemen. Der Test steht zwischen zwei Entwicklungsphasen.
Die zu testende Fahrfunktion soll unter anderem sicher über Kreuzungen fahren können. 

Eine erweiterte Beschreibung und Einordnung des SUC2 im Zusammenspiel mit SUC1 und 3 ist zu finden unter:  
https://gitlab.sl4to5.de/pm/gesamtprojekt/documents/-/blob/50957d50641464b6dc1c5dbd072011a724ed1b1f/10_Projektinput/Usecases-Description.adoc

##### Testziel
Das Testziel ist ein sicheres Verhalten der HADf trotz unvollständiger Umweltwahrnehmung (unterschiedliche Objektinformationen der Sensoren) zu demonstrieren.

##### Demonstrationsziel für die Simulation
Die Simulation ist aus unterschiedlichen Simulationsmodulen/-modellen (aus unterschiedlichen Quellen, nicht proprietär) zusammengesetzt. 
Fahrzeugsystem bestehend aus einer Automation (Fahrzeugautomationslogik + Motion Control + Sensorfusion), mind. 2 Sensormodellen und einem Fahrdynamikmodell.

### SUC3 

#### Beschreibung des zu entwickelnden Produkts
Es wird hier hypothetisch angenommen, dass eine Komponente für hochautomatisierte Systemen L4/L5 für den urbanen Verkehr entwickelt wird.
Unter Komponenten sind Camera, Radar, Lidar, und ggf. Lenkung als Beispiel hier genommen

#### Übergreifende Entwicklungsziele
Die zu entwickelnde Sensor Komponente soll unter anderem ihren Anteil der Umweltwahrneumng leisten können. 
Diese Leistungsfähigkeiten werden während des Entwicklungsprozesses kontinuierlich getestet und sind letztendlich auch Teil der Absicherung. 

#### Simulationsaufgabe
Entwicklungsbegleitendes Testen und Verifikation / Validation einer isolierten Komponente ohne Wechselwirkung

## Process Step Execution

> * Further specification, clarification of the specific engineering task
> * This process step is part of the according engineering task. Information is needed for seamless traceability of simulation task 
> * The content handled in this phase may be extended, but there must be no inconsistencies with the content of the Single Decision Process, otherwise a change request must be issued for the Single Decision Process.

**Collect doing, decisions and reasoning here**



**Collection of all Outputs**

| Previous Step | Next Step |
| ------ | ------ |
| [**Overview**](../../tree/master) | [**Next Step: Requirements Spec**](Documentation/CMP_Phase2_RequirementSpec.md) |