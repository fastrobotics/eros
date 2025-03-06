# Arm & Disarm

## Overview

The purpose of the Arm/Disarm logic is to disable defined actuators/set default values on those actuators when the system is NOT in an "ARMED" state.

## Configuration
- To enable the Default Ready To Arm ROS Publisher, enable the Publisher before the call to "preinitialize_basenode" via: `enable_ready_to_arm_pub(true);`
- The Safety Node needs to be configured and running.  See: [Safety Node](../../../nodes/SafetyNode/doc/SafetyNode.md).

## Operation
The following state machine describes the operation of the Arm/Disarm Logic:

```mermaid
    graph TD;
    DisarmedCannotArm[DISARMED/CANNOT ARM]
    Disarming[DISARMING]
    Disarmed[DISARMED]
    Arming[ARMING]
    Armed[ARMED]
    Start@{ shape: circle, label: " " }
    Start --> | Boot | DisarmedCannotArm
    DisarmedCannotArm --> | All Ready To Arm == True | Disarmed
    Disarmed --> | System Command-Arm | Arming
    Arming -->  | timer > Arming Time | Armed
    Armed --> | System Command-Disarm | Disarming
    Disarming --> | timer > Disarming Time| Disarmed
    Armed --> | Any Ready To Arm == False OR timeout | DisarmedCannotArm
    Arming --> | Any Ready To Arm == False OR timeout  | DisarmedCannotArm
    Disarmed --> | Any Ready To Arm == False OR timeout  | DisarmedCannotArm
    Disarming --> | Any Ready To Arm == False OR timeout  | DisarmedCannotArm

    style DisarmedCannotArm fill:#f00,stroke:#000,color:#000,stroke-width:4px
    style Disarmed fill:#0f0,stroke:#000,color:#000,stroke-width:4px
    style Arming fill:#ff8000,stroke:#000,color:#000,stroke-width:4px
    style Armed fill:#00f,stroke:#000,color:#000,stroke-width:4px
    style Disarming fill:#ff8000,stroke:#000,color:#000,stroke-width:4px

    click DisarmedCannotArm href "https://mermaid.js.org/syntax/flowchart.html" " "
    click Disarming href "https://mermaid.js.org/syntax/flowchart.html" " "
    click Disarmed href "https://mermaid.js.org/syntax/flowchart.html" " "
    click Arming href "https://mermaid.js.org/syntax/flowchart.html" " "
    click Armed href "https://mermaid.js.org/syntax/flowchart.html" " "

```



1. Boot: System is: "DISARMED/CANNOTARM"
2. Safety Node monitors all ready to arm signals.  When all ready to arm signals are "True", Safety Node will change node to "DISARMED".
3. When a System Command is received to "ARM", the Safety Node will transition to "ARMING" and then "ARMED".
4. When a System Command is received to "DISARM", the Safety Node will transition to "DISARMING" and then "DISARMED".
5. If at any time any of the Safety Node Ready to Arm Monitors change to "False", the Safety Node will change state to "DISARMED/CANNOT ARM".  
6. If at any time any of the Safety Node Ready to Arm Monitors "timeout" by not receiving a signal (current limit is 5 seconds), the Safety Node will change state to "DISARMED/CANNOT ARM".
7. If the system is in "DISARMED/CANNOT ARM", it will not change state at all until those conditions are rectified.
## States

### DISARMED-CANNOT ARM
- Default State at Boot

### DISARMING

### DISARMED

### ARMING

### ARMED

## Software Design
![](../../output/Legend.png)

### Class Diagrams
![](../../../include/eros/doc/output/ArmedStateManagerClassDiagram.png)

### Sequence Diagrams
![](../../../include/eros/doc/output/ArmedStateManagerSequenceDiagram.png)