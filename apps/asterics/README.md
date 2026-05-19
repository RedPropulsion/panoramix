# AsterICS

The AsterICS is the main flight controller. Its functionality may be summed up with this automata:

```mermaid
stateDiagram-v2
    [*] --> Boot

    Boot --> Idle

    Idle --> Calibration: CMD_BEGIN_CALIBRATION
    Calibration --> Idle: CMD_DONE

    Idle --> Manual: CMD_BEGIN_MANUAL
    Manual --> Idle: CMD_DONE

    Idle --> Stream: CMD_BEGIN_STREAM
    Stream --> Idle: CMD_DONE

    Idle --> Armed: CMD_ARM + arm switch
    Armed --> Idle: CMD_DISARM

    Armed --> Launch: simulation (acceleration)
```

## State description

All states must provide visual feedback via ledring

- **Boot**
    - (Zephyr Boot).
    - Load parameters from memory.

- **Idle**
    - Listen for commands.

- **Calibration**
    - Calibrate sensors and/or actuators.

- **Manual**
    - Wiggle actuators.
    - Burn pyrocharges.

- **Stream**
    - Enable serial streaming of sensor data.

- **Armed**
    - Ready to launch!

- **Launch**:
    - 🚀

