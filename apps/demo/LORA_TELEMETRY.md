# LoRa telemetry for the Idefix signal screen

The demo still exchanges four-byte ASCII PING/PONG with the existing WL55
responder every five seconds. No WL55 update, radio setting change or custom
MAVLink dialect is required. The LED command IDs and startup retry remain intact.

After each cycle ObelICS sends a snapshot over the existing MAVLink/UDP endpoint
using common NAMED_VALUE_INT messages. All fields share one time_boot_ms.

| Field | Meaning |
|---|---|
| LR_BOOT | Random nonnegative 31-bit transport boot identifier |
| LR_SEQ | Observation cycle number |
| LR_TX, LR_RX | Completed PING transmissions, all received radio packets |
| LR_OK | Exact four-byte PONG replies |
| LR_TO, LR_ERR | Receive timeouts, configuration/TX/RX errors |
| LR_RESULT | 1=PONG, 2=timeout, 3=unexpected payload, 4=radio error, 5=unavailable |
| LR_RSSI, LR_SNR | Last valid PONG metrics in dBm/dB; valid only if LR_AGE>=0 |
| LR_RTT | Successful cycle round-trip ms, otherwise -1 |
| LR_AGE | Milliseconds since last valid PONG, or -1 before the first |
| LR_END | Snapshot schema version, currently 1 |

The consumer must require the complete set, handle reordering and reject old or
duplicate cycles. A timeout does not create a fresh signal measurement. RTT
includes PING airtime and the responder's deliberate 200 ms delay. The radio
payload itself remains unnumbered and assumes a single pair of nodes.

Message packing and writes are serialized with a mutex shared by ACK and
telemetry send paths. The consumer in RedPropulsion/idefix-touch-web-app uses a
persistent UDP receiver so telemetry and command acknowledgements coexist.

## Validation

Build the demo with the same Zephyr workspace and the known DHCP-disabled fix
in the separate zephyr-mavlink dependency. This change neither updates the
manifest nor commits that pre-existing dependency fix.

```sh
west build -d build-lora-signal apps/demo
west flash -d build-lora-signal
```

Flash only after a successful build, with only the H723 ST-LINK connected.
Hardware validation: LED commands during radio traffic, WL55 power loss and
recovery, late Idefix startup, H723 reboot, Ethernet disconnect/reconnect,
and a representative 6–8 hour endurance run. These hardware checks must be
performed after flashing; a successful build is not hardware verification.
