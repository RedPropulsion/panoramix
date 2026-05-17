# u-blox M10 GNSS Driver

Driver for u-blox MAX-M10S GNSS module over I2C. Supports UBX-NAV-PVT messages at 25Hz with dual timestamps (GPS nanosecond-precise + CPU microsecond).

## Hardware Connection

| Signal | Pin   | Notes |
|--------|-------|-------|
| SCL     | PF1   | I2C2  |
| SDA     | PF0   | I2C2  |
| VCC     | 3.3V  |       |
| GND     | GND   |       |

- I2C Address: **0x42** (7-bit)
- Protocol: UBX binary + NMEA fallback

## Architecture Overview

```mermaid
flowchart TB
    subgraph Thread["M10 Acquisition Thread (25Hz)"]
        direction LR
        A[Configure M10<br/>once] --> B[Poll for<br/>available bytes]
        B --> C[Read from<br/>I2C]
        C --> D[Parse UBX<br/>or NMEA]
        D --> E[Store in<br/>ring buffer]
    end

    Thread --> F[Consumer API]

    subgraph Buffer["Ring Buffer (2 entries)"]
        direction TB
        G[write_idx] --> H[position data] --> I[read_idx]
    end

    E --> Buffer

    subgraph API["Public API Functions"]
        J[gps_get_latest]
        K[gps_get_latest_if_fresh]
        L[gps_has_fix]
        M[gps_get_satellites]
        N[gps_get_latitude/longitude/altitude]
    end

    F --> API
    Buffer -.-> J
    Buffer -.-> K
    Buffer -.-> L
    Buffer -.-> M
    Buffer -.-> N
```

## Data Flow

### 1. Configuration Phase (Runs Once)

```mermaid
sequenceDiagram
    participant MCU
    participant M10

    Note over MCU: m10_configure()
    MCU->>M10: MON-VER poll (wake-up)
    MCU->>M10: CFG-RATE: meas=40ms, nav=40ms, ref=40ms (25Hz)
    MCU->>M10: CFG-MSG: Enable UBX-NAV-PVT on I2C
    MCU->>M10: CFG-MSG: Disable NMEA GGA/GLL/GSA/RMC on I2C
    Note over MCU: Configuration complete
```

### 2. Data Acquisition Phase (25Hz Loop)

```mermaid
flowchart LR
    subgraph Acquire["m10_acquire_thread (40ms period)"]
        A[Write 0xFD<br/>Register] --> B[Read bytes<br/>available]
        B --> C{avail > 0?}
        C -->|No| D[Skip cycle]
        C -->|Yes| E[Write 0xFF<br/>Register]
        E --> F[Read data<br/>up to 256B]
        F --> G{Parse data}
        G -->|UBX found| H[UBX-NAV-PVT<br/>parser]
        G -->|Fallback| I[NMEA $GGA<br/>parser]
        H --> J[Valid fix?]
        I --> J
        J -->|Yes| K[Store in<br/>ring buffer]
        J -->|No| L[Skip]
    end
```

### 3. Dual Timestamps

```mermaid
flowchart TB
    subgraph GPS["gps_timestamp_ns (uint64_t)"]
        A[iTOW ms<br/>GPS time of week] --> B[× 1,000,000]
        B --> C[nano from<br/>NAV-PVT]
        C --> D[gps_timestamp_ns<br/>= iTOW*1e6 + nano]
    end

    subgraph CPU["cpu_timestamp_us (uint32_t)"]
        E[k_cycle_get_32] --> F[k_ticks_to_us_ceil32]
        F --> G[cpu_timestamp_us<br/>for age calculation]
    end
```

Each position includes two timestamps for different purposes:

| Timestamp | Purpose | Calculation |
|-----------|---------|-------------|
| `gps_timestamp_ns` | Precise sync with external systems | `iTOW * 1,000,000 + nano` |
| `cpu_timestamp_us` | Data freshness check | `k_ticks_to_us_ceil32(k_cycle_get_32())` |

### 4. Consumer API Flow

```mermaid
flowchart TB
    subgraph GL["gps_get_latest"]
        A1["Read ring_buffer[read_idx]"] --> B1["Return position"]
    end

    subgraph GLF["gps_get_latest_if_fresh"]
        C1{"write_idx<br/>!=<br/>read_idx?"}
        C1 -->|"No"| D1["return<br/>-ENODATA"]
        C1 -->|"Yes"| E1["Read position"]
        E1 --> F1["Calculate age"]
        F1 --> G1{"age<br/><<br/>max_age_ms?"}
        G1 -->|"No"| H1["return<br/>-ETIMEDOUT"]
        G1 -->|"Yes"| I1["return<br/>position"]
    end
```

Both functions access the same ring buffer - `gps_get_latest()` is non-blocking (may return stale data), while `gps_get_latest_if_fresh()` validates data age before returning.

## File Structure

```mermaid
graph TD
    A[u_blox_m10/] --> B[gnss_u_blox_m10.c]
    A --> C[gnss_u_blox_m10.h]
    A --> D[gnss_u_blox_m10_i2c.c]
    A --> E[gnss_u_blox_m10_i2c.h]
    A --> F[Kconfig]
    A --> G[CMakeLists.txt]
    A --> H[README.md]

    B --> B1[Acquisition thread<br/>UBX/NMEA parsing<br/>Configuration]
    C --> C1[struct gps_position<br/>Public API functions]
    D --> D1[Modem pipe I2C<br/>transport]
    E --> E1[Register definitions<br/>I2C config struct]
```

## Configuration

### Device Tree (nucleo_h723zg.overlay)

```dts
&i2c2 {
    status = "okay";
    pinctrl-0 = <&i2c2_scl_pf1 &i2c2_sda_pf0>;
    clock-frequency = <I2C_BITRATE_FAST>;  // 400kHz

    gps: gps@42 {
        compatible = "u-blox,m10";
        reg = <0x42>;
    };
};
```

### Kconfig

```
CONFIG_GNSS=y
CONFIG_GNSS_U_BLOX_M10=y
CONFIG_GNSS_LOG_LEVEL_DBG=y  # Optional: debug output
```

### DMA (Currently Disabled)

I2C DMA on STM32H723 has compatibility issues. The driver works with blocking I2C:

```
# In prj.conf - DMA disabled (default)
# CONFIG_I2C_STM32_V2_DMA is not set
```

## UBX Message Format

### NAV-PVT (0x01 0x07) - Position, Velocity, Time

```
Offset  Size  Description
------  ----  -----------
+0      1     Sync1 (0xB5)
+1      1     Sync2 (0x62)
+2      1     Class  (0x01 = NAV)
+3      1     ID     (0x07 = PVT)
+4      2     Length (LE uint16, 92 bytes)
+6     92     Payload (see below)
+98     1     CK_A (checksum A)
+99     1     CK_B (checksum B)
```

### Payload Structure (from Zephyr's `ubx_nav_pvt`)

```c
struct ubx_nav_pvt {
    struct {
        uint32_t itow;      // GPS time of week (ms)
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t valid;
        uint32_t tacc;
        int32_t nano;       // nanoseconds (can be negative)
    } time;

    uint8_t fix_type;       // 0=none, 1=DR, 2=2D, 3=3D
    uint8_t flags;
    uint8_t flags2;

    struct {
        uint8_t num_sv;
        int32_t longitude;  // 1e-7 degrees
        int32_t latitude;   // 1e-7 degrees
        int32_t height;     // mm (ellipsoid)
        int32_t hmsl;       // mm (mean sea level)
        uint32_t horiz_acc; // mm
        uint32_t vert_acc;  // mm
        int32_t ground_speed;   // mm/s
        int32_t head_motion;   // 1e-5 degrees
    } nav;
} __packed;
```

## GPS Fix Types

| Value | Meaning | Description |
|-------|---------|-------------|
| 0 | No fix | No position available |
| 1 | Dead reckoning | Inertial-only, no GPS |
| 2 | 2D fix | Latitude + longitude, no altitude |
| 3 | 3D fix | Full position (valid for time) |

A position is considered **valid** when:

```c
pos.valid = (fix_type >= 3) && (flags & UBX_NAV_PVT_FLAGS_GNSS_FIX_OK);
```

## Error Handling

| Error Type | Behavior |
|------------|----------|
| I2C errors | Logged and skipped, thread continues polling |
| Configuration failures | Retried up to 5 times, then skipped |
| Parse failures | Return -EINVAL, try other parser |
| No data | When bytes available = 0, skip and retry next cycle |

## Timing Considerations

The driver runs at 25Hz (40ms period). Without debug logging, timing may be too tight for the M10 to prepare data. Adding small delays between I2C operations improves reliability:

```c
k_sleep(K_MSEC(1));  // After writing register, before reading
```

## Testing

```bash
# Build
west build -p always panoramix/apps/obelics

# Flash
west flash

# Monitor
west attach
```

Expected output (with fix):
```
GPS: 17/05/2026 14:32:15.123 | lat=437744682, lon=112862637, alt=72000mm | sats=8, fix=3, hdop=15 | speed=0mm/s, heading=0.00000 | gps_ns=..., cpu_us=...
```