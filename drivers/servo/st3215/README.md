# Waveshare ST3215 Serial Bus Servo Driver Project Overview

A [servo motor](https://en.wikipedia.org/wiki/Servomotor) is essentially a specialized rotary actuator designed for high-precision control of angular position, velocity, and acceleration. Unlike a standard DC motor, which spins freely as long as power is applied, a servo is built to move to a specific angle and maintain it with high accuracy. For a visual demonstration and a deeper explanation of how these components function, [this video](https://www.youtube.com/watch?v=1WnGv-DPexc) provides an excellent foundation, while the [ST3215 wiki](https://www.waveshare.com/wiki/ST3215_Servo) covers the specific hardware specifications for the component used in this project.

## PWM and Serial Communication

There are two primary methods for controlling a servo. Traditional servos rely on Pulse Width Modulation (PWM), where the width of a timed electrical pulse (like 1ms or 2ms every ~20ms) dictates the motor's position. While PWM is simple and widely supported, it is essentially a one-way street; the microcontroller sends a command, but the servo cannot "talk back" to report its actual position, temperature, or status.<br>
The Waveshare ST3215 used in this project is instead a Serial Bus servo: by using a digital [UART](https://www.circuitbasics.com/basics-uart-communication/) protocol (great illustrations at this link), many of these servos can be "daisy-chained" together on a single bus (with PWM each one requires its own wire). More importantly, serial servos allow two-way telemetry: other thank making it rotate, we can query the ST3215 for real-time feedback, such as its current angle, internal temperature, and load.
* Source: [Serial servos guide](https://www.kevsrobots.com/blog/serial-servos.html).

## Communication Protocol

The ST3215 operates on an asynchronous serial protocol, typically running at a high-speed baud rate of 1 Mbps (Actually no??). Communication is structured around command packets, as detailed in the [protocol manual](https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf). These packets follow a strict architecture: 
A typical command packet follows this structure: `[Header: 0xFF 0xFF] [ID] [Length] [Instruction] [Parameters] [Checksum]`.\\
The dual-byte header (`0xFF 0xFF`) is followed by the unique ID of the target servo, the data length, the specific instruction (such as a write or read command), the parameters, and finally a checksum to ensure data integrity.

## Hardware

The ST3215 protocol manual specifically says that communication is asynchronous duplex: the controller sends out the instruction package and the servo returns the response package, and of course it's important to ensure that each component waits for its turn and that data isn't transmitted at the same time, causing a collision. To control the servo we use the [STM32 Nucleo-H723ZG](https://www.st.com/en/evaluation-tools/nucleo-h723zg.html) microcontroller, specifically pins PD5 (TX) and PD6 (RX) for the data stream, while PD7 (Enable) serves as a hardware switch. By toggling the Enable pin, the driver can manually switch the circuit between transmit and receive modes to prevent data collisions. For details on the wiring check out the [bus servo control circuit](https://www.waveshare.com/w/upload/d/d3/Bus_servo_control_circuit.pdf) design.

* must add details on physical connections and electronics (?)

## Goal of the project

The goal of this project is to build a driver (a C API) with the Zephyr RTOS ecosystem for the ST3215 Waveshare Servo. The user will interact with a clean C API: the driver will handle the details described before for the communication protocol and provide simple functions to set the servo angle and retrieve real-time position feedback and also the other functions mentioned in section 1.3 of the protocol manual. Error handling will also be a concern, in particular the checksum and correct message structure control for each packet and also making sure that the communication flows without collision and within a certain time lapse (timeout).

# Software Structure

The architecture of the project tries to follow Zephyr's best practices for application development

## (TODO: rewrite) File structure and description

Following is the file structure and description of the `appRedPropulsion` folder:

* `nucleo_h723zg.overlay`: a [devicetree](https://docs.zephyrproject.org/latest/build/dts/index.html#devicetree) overlay file that has the purpose of configuring the hardware the application is going to use, in this case the physical pins like the `usart2` (PD5 as TX, PD6 as  as a RX) and pin PD7 for direction control (TR/RX Enable).
* `CMakeLists.txt`: tells the build system where to find the other application files, and links the application directory with Zephyr’s CMake build system.
* `st3215.h`: a hedaer file defining the device struct, protocol constants (such as header, instructions and register addresses) and API function signatures.
* `st3215.c`: implements the API functions (ping, read, write and so on) by building communication packages, calculating checksums and handling the communication with its potential errors and timeouts.
* `main.c`: links the device tree to the code (by identifying the node from the overlay file and assigning it to a device struct member), initializes the system and runs the main application loop.

## Implemented Features

The following servo features have been implemented:

* `st3215_ping`: sends the PING (0x01) instruction to verify the presence of the servo on the bus before continuing the following operations.
* `st3215_set_angle`: sends the WRITE DATA (0x03) instruction to send a 12 bit value representing the target angle (practically, we use 16 bits and then split them with simple logical operations in a low byte and high byte to comply the package structure). To transmit data the `uart_poll_out` function is used. It continuously checks (polling) if the transmitter is not full so it can send a new byte to the target register. Until the previous byte is sent, the current calling thread is blocked.
* `st3215_get_angle`: sends the READ DATA (0x02) instruction to query the position register (address 0x38), reading the two bytes needed and reconstructing the actual angle with shift and other logical operations. To receive data, the `uart_poll_in` function is used. It continuously checks if the device register contains a valid byte, and when it does it reads it, stores to the location pointed to by the second pointer argument, and returns 0 to the calling thread (otherwise it returns -1, an error code). This instruction is in a while loop because it's non-blocking, so we can give the servo enough time to respond and move on only when the timeout is reached (or the data has been read correctly).

## Error Handling

To handle errors, the driver checks the checksum of each received packet to ensure data integrity. The checksum is calculated as the bitwise NOT of the sum of the bytes starting from the ID. If the checksum does not match, an error code is returned. Additionally, timeouts are implemented for read operations to prevent the application from hanging indefinitely while waiting for a response from the servo. If a timeout occurs, an appropriate error code is returned to indicate that the operation was unsuccessful. There are also checks for the correct structure of the packets, such as verifying the header, the ID of the device and ensuring that the length of the packet matches the expected value based on the instruction being sent. The status of the servo can also be monitored through the error byte in the response packet, however the manual does not specify what each bit represents, so for now the driver simply checks if it's zero (no error) or non-zero (some error occurred).
