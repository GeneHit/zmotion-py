# zmotion-py
Python bindings of the ZMotion control card library

Zmotion office: http://www.zmotionglobal.com/

## Motion Control Card Overview

A motion control card is a specialized piece of hardware used to control the 
movement of mechanical systems. Below is a simple diagram of a motion control system:

```mermaid
graph LR
    subgraph Control_Flow
        A[Computer] --->|Commands|---> B[Motion Control Card]
        B --->|Control Signals|---> C[Motor Driver]
        C --->|Power|---> D[Motor]
        D --->|Movement|---> E[Mechanical System]
    end
    subgraph Feedback_Loop
        E --->|Feedback|---> F[Sensor]
        F --->|Data|---> B ---> A
    end
```

This setup allows for precise and efficient control of mechanical movements in 
various applications such as robotics, CNC machines, and automated manufacturing systems.

## Difference Between Manual Compilation and Runtime Compilation

| **Manual Compilation** | **Runtime Compilation** |
|------------------------|-------------------------|
| The code is compiled before execution. | The code is compiled during execution. |
| Generally results in faster execution since the code is already translated into machine language. | May introduce overhead due to the compilation process happening at runtime. |
| Compilation errors are caught before the program runs. | Errors may occur during execution, which can be harder to debug. |
| The compiled binary is specific to the architecture it was compiled for. | Allows for optimizations based on the current execution context. |

Choose the method based on the requirements of the application.