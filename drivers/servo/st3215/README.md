# Waveshare ST3215 Serial Bus Servo Driver

A [servo motor](https://en.wikipedia.org/wiki/Servomotor) is essentially a specialized rotary actuator designed for high-precision control of angular position, velocity, and acceleration. Unlike a standard DC motor, which spins freely as long as power is applied, a servo is built to move to a specific angle and maintain it with high accuracy. For a visual demonstration and a deeper explanation of how these components function, [this video](https://www.youtube.com/watch?v=1WnGv-DPexc) provides an excellent foundation, while the [ST3215 wiki](https://www.waveshare.com/wiki/ST3215_Servo) covers the specific hardware specifications for the ST3215 component.

## PWM vs. Serial Communication

Traditional servos rely on Pulse Width Modulation (PWM), a one-way communication where the pulse width dictates the motor's position. The ST3215 is, instead, a Serial Bus servo that utilizes a digital [UART](https://www.circuitbasics.com/basics-uart-communication/) protocol. The key advantages of this approach include:
* **Daisy-chaining:** multiple servos can be connected to a single serial bus, drastically reducing wiring complexity.
* **Two-way Telemetry:** the microcontroller can query the servo for real-time data such as current position, internal temperature, load, voltage, and speed.

## Communication Protocol

The ST3215 operates on an asynchronous serial protocol with a baud rate of **1 Mbps** (actually no?). 
Communication is structured around command packets as defined in the [Waveshare protocol manual](https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf):

`[Header: 0xFF 0xFF] [ID] [Length] [Instruction] [Parameters] [Checksum]`

* **Header:** Two bytes (`0xFF 0xFF`) to signal the start of a packet.
* **ID:** Unique identifier for the target servo (Default: `0x01`).
* **Length:** The number of remaining bytes in the packet.
* **Instruction:** The command type (e.g., `0x02` for Read, `0x03` for Write).
* **Checksum:** Calculated as the bitwise NOT of the sum of all bytes, starting from the ID to the last parameter.

## Hardware Integration

* TODO

### Auto-Direction Circuit
In this case we don't need a GPIO "Enable" pin to manually toggle between Transmit and Receive modes, as the Waveshare hardware automatically manages signal direction through **TX negation**, allowing the software driver to operate using pure UART logic without the need for manual direction switching in the code.

## Software Architecture (Zephyr RTOS)

The driver is integrated into the Zephyr RTOS device model, implementing the `servo.h` interface. This ensures portability and interoperability with other parts of the ecosystem.

### File Structure

* `st3215.c`: Implementation of the driver logic, packet management, and Zephyr APIs.
* `waveshare,st3215.yaml`: Devicetree binding for UART peripheral configuration.

### Implemented Features

1.  **`st3215_set_position`**: Converts millidegrees to the servo's 12-bit scale (0-4095) and sends the write command. It uses `uart_poll_out` to ensure the packet is fully dispatched: this function continuously checks (polling) if the transmitter is not full so it can send a new byte to the target register. Until the previous byte is sent, the current calling thread is blocked.
2.  **`st3215_get_position`**: sends the READ DATA (0x02) instruction to query the position register (address 0x38), reading the two bytes needed and reconstructing the actual angle with shift and other logical operations. The reception is handled with a non-blocking approach:
    * **Non-blocking Polling** (`uart_poll_in`): the driver utilizes uart_poll_in to retrieve the response. Unlike blocking functions, uart_poll_in returns immediately if no data is available. The driver wraps this in a controlled while loop with a microsecond delay (`k_usleep`) and a maximum attempt counter. This approach provides a software timeout that prevents the entire system from hanging if the servo fails to respond.
    * **UART Flush:** Before every read operation, the driver flushes the RX buffer to discard any residual data resulting from the half-duplex nature of the bus.
    * **Conversion:** reconstructs the 12-bit value from the two response registers and converts it back to millidegrees for the application.

## Error Handling & Reliability

* **Checksum Validation:** every received packet is validated. If the checksum does not match, the driver returns `-EBADMSG` to prevent processing corrupted telemetry.
* **Timeout Management:** since reception is non-blocking, a microsecond-based timeout loop is implemented to prevent system hangs if a servo is disconnected.
* **Status Monitoring:** the driver monitors the status byte returned by the servo. Any anomalies (overheating, voltage fluctuations) are signaled through the Zephyr logging system.