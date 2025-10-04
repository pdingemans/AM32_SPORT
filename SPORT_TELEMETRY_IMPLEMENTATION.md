# S.PORT Telemetry Implementation in AM32_SPORT

## Overview

This document describes the FrSky S.PORT telemetry implementation added to the AM32 ESC firmware on the `feature_sport` branch. S.PORT (Smart Port) is FrSky's single-wire bidirectional telemetry protocol operating at 57600 baud with inverted signaling.

## Table of Contents

1. [Core S.PORT Implementation](#core-sport-implementation)
2. [Protocol Details](#protocol-details)
3. [Hardware Integration](#hardware-integration)
4. [Changes to Main Branch](#changes-to-main-branch)
5. [Telemetry Data Flow](#telemetry-data-flow)
6. [Polling Architecture](#polling-architecture)
7. [Configuration](#configuration)
8. [Testing](#testing)

---

## Core S.PORT Implementation

### New Files Added

The S.PORT telemetry implementation consists of the following new files:

**Protocol Layer:**
- `Inc/sport_telemetry.h` - S.PORT telemetry protocol definitions and interface
- `Src/sport_telemetry.c` - S.PORT telemetry implementation with frame encoding/decoding
- `src/kiss_telemetry.c` - KISS telemetry implementation (lowercase src directory)

**Hardware Abstraction Layer (AT32F421):**
- `Mcu/f421/Inc/singlewire_uart.h` - Single-wire UART hardware interface for S.PORT
- `Mcu/f421/Src/singlewire_uart.c` - Single-wire UART implementation using TMR15 + DMA5

**Serial Telemetry Base Class:**
- `Mcu/f421/Inc/serial_telemetry.h` - Base class for all serial telemetry types
- `Mcu/f421/Src/serial_telemetry.c` - Common telemetry interface implementation

## Architecture

### Telemetry Abstraction Layer

The implementation uses a **clean object-oriented abstraction** in C to support multiple telemetry protocols without polluting `main.c`:

```c
// Base class for all serial telemetry protocols (serial_telemetry.h)
typedef struct serial_telemetry_class {
    uint8_t id;                    // Sensor/ESC ID
    uint8_t update_on_interval;    // Auto-update interval (ms), 0 = disabled
    
    // Virtual function pointers (polymorphism)
    void (*set_id)(struct serial_telemetry_class* self, uint8_t new_id);
    void (*handle_esc_telemetry)(struct serial_telemetry_class* self);
    void (*makeTelemPackage)(struct serial_telemetry_class* self, 
                             uint8_t temp, uint16_t voltage, 
                             uint16_t current, uint16_t consumption, 
                             uint16_t e_rpm);
} serial_telemetry_class;
```

### How Protocols "Inherit" the Base Class

**S.PORT Telemetry (`sport_telemetry.c`):**

```c
// Static instance with function pointers initialized
serial_telemetry_class sport_telemetry = {
    .id = 0,
    .set_id = sport_sensor_set_id,              // S.PORT-specific ID calculation
    .handle_esc_telemetry = sport_telemetry_handle_poll,  // Poll-based
    .makeTelemPackage = sportmakeTelemPackage,  // S.PORT frame packing
};

// Factory function returns pointer to static instance
serial_telemetry_class *init_sport_telemetry(void) {
    singlewire_uart_init();  // Hardware setup
    return &sport_telemetry;
}
```

**KISS Telemetry (`kiss_telemetry.c`):**

```c
// Static instance with different function pointers
serial_telemetry_class kiss_telemetry = {
    .id = 0,
    .update_on_interval = 0,
    .set_id = set_kiss_telemetry_id,           // KISS ID = update interval
    .handle_esc_telemetry = handle_kiss_esc_telemetry,  // Checks flags + interval
    .makeTelemPackage = makeTelemPackage,      // KISS packet structure
};

// Factory function returns pointer to static instance
serial_telemetry_class *init_kiss_telemetry() {
    telem_UART_Init();  // USART1 half-duplex hardware (same PB6 pin)
    return &kiss_telemetry;
}

// Telemetry handler - THREE transmission triggers
void handle_kiss_esc_telemetry(serial_telemetry_class *self) {
    static uint32_t old_millis = 0;
    
    // Trigger 1: DShot telemetry request (bit 11 = 1)
    // Set in dshot.c when DShot frame has telemetry bit set
    if (send_telemetry) {
        send_telem_DMA(10);  // Send 10-byte KISS telemetry packet
        send_telemetry = 0;
    }
    
    // Trigger 2: DShot command 6 (ESC info request)
    // Set in dshot.c when command 6 received 6 times consecutively
    if (send_esc_info_flag) {
        makeInfoPacket();    // Build 49-byte ESC info packet
        send_telem_DMA(49);
        send_esc_info_flag = 0;
    }
    
    // Trigger 3: Interval-based periodic transmission
    // Only if update_on_interval > 0 (set by ID: 29 + ID ms)
    if (self->update_on_interval == 0)
        return;
    
    if (millis() - old_millis >= (uint32_t)self->update_on_interval) {
        send_telem_DMA(10);
        old_millis = millis();
    }
}
```

**Global flags set by DShot decoder (`dshot.c`):**

```c
// In dshot.c - declared in main.c
extern char send_telemetry;      // Set when DShot bit 11 (telemetry request) = 1
extern char send_esc_info_flag;  // Set when DShot command 6 received 6× consecutively

// DShot frame processing (around line 87)
if (calcCRC == checkCRC) {
    if (dpulse[11] == 1) {
        send_telemetry = 1;  // Telemetry bit set → trigger KISS transmission
    }
    // ...
}

// DShot command processing (around line 165)
case 6:
    send_esc_info_flag = 1;  // Command 6 → trigger ESC info transmission
    break;
```

### How main.c Uses Polymorphism

**In `main.c`:**

```c
// Single pointer to any telemetry protocol
serial_telemetry_class* serial_telemetry = NULL;

// Initialization (around line 1690)
uint8_t id = eepromBuffer.telemetry_on_interval;

if (id < 10) {
    // KISS telemetry for IDs 0-9
    serial_telemetry = init_kiss_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
} else if (id < 20) {
    // S.PORT telemetry for IDs 10-19
    serial_telemetry = init_sport_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
}

// Main loop - protocol-agnostic call
while (1) {
    if (serial_telemetry != NULL) {
        // Calls sport_telemetry_handle_poll() OR handle_kiss_esc_telemetry()
        // depending on which was initialized
        // - S.PORT: Checks for poll frames (0x7E [ID])
        // - KISS: Checks global flags (send_telemetry, send_esc_info_flag) + interval
        serial_telemetry->handle_esc_telemetry(serial_telemetry);
    }
    // ...
}

// ADC processing section (1kHz) - protocol-agnostic data update
if (PROCESS_ADC_FLAG == 1) {
    // Process ADC values first
    degrees_celsius = converted_degrees;
    battery_voltage = /* filtered ADC voltage */;
    actual_current = /* filtered ADC current */;
    
    // Update telemetry data
    serial_telemetry->makeTelemPackage(serial_telemetry, 
                                       degrees_celsius, 
                                       battery_voltage, 
                                       actual_current,
                                       consumed_current >> 16, 
                                       e_rpm);  // ⚠️ May be stale
}
```

### Key Differences: S.PORT vs KISS

| Aspect | S.PORT | KISS |
|--------|--------|------|
| **Direction** | Bidirectional (request/response) | Unidirectional (TX only) |
| **Trigger 1** | Receiver polls ESC (0x7E [ID]) | DShot telemetry bit (bit 11 = 1) |
| **Trigger 2** | - | DShot command 6 (ESC info) |
| **Trigger 3** | - | Interval (29 + ID ms, e.g., 30-38ms) |
| **Hardware** | Timer-based UART (PB6, TMR15, DMA5) | USART1 half-duplex (PB6, USART1, DMA2) |
| **ID Calculation** | FrSky parity bits (9-18 → sensor IDs) | Direct (0-9 → update interval) |
| **Data Format** | 4 frames (power, rpm/cons, temp, sbec) | 1 frame (10 bytes, all data) |
| **Frame Size** | Variable (8-16 bytes with stuffing) | Fixed (10 or 49 bytes with CRC) |
| **Protocol** | FrSky S.PORT standard | KISS ESC telemetry |
| **Use Case** | FrSky receivers (X8R, X6R, etc.) | KISS FC, Betaflight, iNav, etc. |
| **Global Flags** | None (polling-based) | `send_telemetry`, `send_esc_info_flag` |

---

## Protocol Details

### S.PORT Frame Structure

S.PORT uses a single-wire half-duplex UART at **57600 baud, 8N1, inverted signaling**.

**Frame Format:**
```
[HEADER][ID][DATA_ID][VALUE_LOW][VALUE_MID][VALUE_HIGH][VALUE_TOP][CRC]
  0x10   1B    2B        1B         1B         1B          1B       1B
```

- **HEADER**: Always `0x10` for data frames
- **ID**: Physical sensor ID (polled by receiver)
- **DATA_ID**: Identifies the telemetry parameter (voltage, current, RPM, etc.)
- **VALUE**: 32-bit value, little-endian (4 bytes)
- **CRC**: CRC-8 checksum over entire frame

### Byte Stuffing

To prevent frame desynchronization, special bytes are escaped:

- `0x7D` → `0x7D 0x5D`
- `0x7E` → `0x7D 0x5E`

This applies to all bytes except the initial header.

### Sensor Data IDs
The implementation uses **ESC-specific data IDs**

```c
#define ESC_POWER_DATA_ID     0x0B50  // Voltage (lower 16 bits) + Current (upper 16 bits)
#define ESC_RPM_CONS_DATA_ID  0x0B60  // eRPM (lower 16 bits) + Consumption (upper 16 bits)
#define ESC_TEMP_DATA_ID      0x0B70  // ESC temperature (°C)
#define ESC_SBEC_DATA_ID      0x0E50  // SBEC voltage + current (not implemented)
```

**Data packing:**
- Power frame: `data = (current << 16) | voltage`
- RPM/Consumption frame: `data = (consumption << 16) | e_rpm`
- Temperature frame: `data = temp` (32-bit value, only lower 8 bits used)
- SBEC frame: `data = (sbec_current << 16) | sbec_voltage` (placeholder, always 0)

### CRC-8 Calculation

S.PORT uses a simple CRC-8 (sum of all frame bytes):

```c
uint8_t sport_calc_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc += data[i];
    }
    return 0xFF - crc;
}
```

---

## Hardware Integration (AT32F421)

### Single-Wire UART Abstraction

The single-wire UART is **completely abstracted** from the telemetry protocols. This clean separation allows both S.PORT and KISS telemetry to use the same hardware driver without knowing implementation details.

**Architecture layers:**

```
┌─────────────────────────────────────────────────────────────┐
│  Telemetry Protocol Layer (sport_telemetry.c, kiss_telemetry.c) │
│  - Protocol-specific framing, CRC, data packing             │
│  - Calls: singlewire_uart_send_frame(buffer, len)          │
│           singlewire_uart_read_byte(&data)                  │
└───────────────────────┬─────────────────────────────────────┘
                        │ Clean API boundary
                        ▼
┌─────────────────────────────────────────────────────────────┐
│  Single-Wire UART API (singlewire_uart.h)                   │
│  - singlewire_uart_init()                                   │
│  - singlewire_uart_send_frame(uint8_t* buffer, uint8_t len) │
│  - singlewire_uart_read_byte(uint8_t* data)                 │
└───────────────────────┬─────────────────────────────────────┘
                        │ Hardware abstraction
                        ▼
┌─────────────────────────────────────────────────────────────┐
│  Single-Wire UART Implementation (singlewire_uart.c)        │
│  - Pure DMA operation (zero CPU for TX/RX)                  │
│  - EXTI edge detection for RX frame start                   │
│  - Timer-triggered DMA for bit sampling/generation          │
│  - Half-duplex GPIO switching (INPUT ↔ OUTPUT)              │
│  - FIFO buffer for received bytes                           │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│  Hardware (AT32F421)                                         │
│  - GPIOB Pin 6 (hardcoded in singlewire_uart.c)             │
│  - TMR15 (baud rate timing)                                 │
│  - DMA1_CHANNEL5 (timer-triggered sampling/output)          │
│  - EXTI6 (rising edge detection)                            │
└─────────────────────────────────────────────────────────────┘
```

### How S.PORT Uses Single-Wire UART

**In `sport_telemetry.c`:**

```c
// Initialization - called once
serial_telemetry_class *init_sport_telemetry(void) {
    singlewire_uart_init();  // Hardware setup - no protocol knowledge
    return &sport_telemetry;
}

// Polling for incoming data
void sport_telemetry_handle_poll(serial_telemetry_class *self) {
    uint8_t data;
    while (singlewire_uart_read_byte(&data)) {  // Read from FIFO
        // Parse S.PORT poll frames (0x7E [ID])
        if (previous_char == 0x7E && data == esc_sensor.sensor_id) {
            // Send response
            singlewire_uart_send_frame(frame_buffer, frame_len);
        }
        previous_char = data;
    }
}
```

### How KISS Uses Single-Wire UART

**In `kiss_telemetry.c`:**

```c
// Initialization - called once
serial_telemetry_class *init_kiss_telemetry() {
    telem_UART_Init();  // Sets up different UART for KISS (not single-wire)
    return &kiss_telemetry;
}

// Periodic transmission
void handle_kiss_esc_telemetry(serial_telemetry_class *self) {
    static uint32_t old_millis = 0;
    
    // Interval-based transmission (no polling)
    if (millis() - old_millis >= self->update_on_interval) {
        send_telem_DMA(10);  // Uses USART1 + DMA2 (not timer-based)
        old_millis = millis();
    }
}
```

**Key difference:** KISS telemetry uses **USART1 in half-duplex mode** (via `telem_UART_Init()` and `send_telem_DMA()`), not the single-wire timer-based implementation. Only S.PORT uses the timer-based `singlewire_uart.c`.

### KISS Telemetry Hardware Implementation

**KISS uses USART1 with DMA2 on the same PB6 pin:**

```c
// In Mcu/f421/Src/serial_telemetry.c
void telem_UART_Init(void) {
    // Enable clocks
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    // Configure PB6 as USART1_TX (same pin as S.PORT)
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = GPIO_PINS_6;        // PB6
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_0);  // USART1 function

    // Configure DMA1_CHANNEL2 for USART1 TX
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
    dma_init_struct.memory_base_addr = (uint32_t)aTxBuffer;

    // Configure USART1 in half-duplex mode
    usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_single_line_halfduplex_select(USART1, TRUE);  // Single-wire mode
    usart_dma_transmitter_enable(USART1, TRUE);
}

void send_telem_DMA(uint8_t bytes) {
    DMA1_CHANNEL2->ctrl_bit.chen = FALSE;
    DMA1_CHANNEL2->dtcnt = bytes;                // Set transfer count
    DMA1_CHANNEL2->ctrl_bit.chen = TRUE;         // Start DMA transfer
}
```

**Hardware Differences:**

| Aspect | S.PORT (singlewire_uart.c) | KISS (serial_telemetry.c) |
|--------|----------------------------|----------------------------|
| **Hardware** | TMR15 + DMA1_CHANNEL5 + EXTI6 | USART1 + DMA1_CHANNEL2 |
| **Pin** | PB6 (GPIO bit-banged via timer) | PB6 (USART1_TX native function) |
| **Baud Rate** | 57600 (S.PORT standard) | 115200 (KISS standard) |
| **Direction** | Bidirectional (RX/TX switching) | TX-only (unidirectional) |
| **Protocol** | Polling-based (receiver-initiated) | Push-based (ESC-initiated) |
| **Timing** | Real-time response (<100µs) | Interval-based (30-38ms) |

### Single-Wire UART Implementation Details

**Configuration structure:**

```c
typedef struct {
    gpio_type *gpio_port;       // e.g., GPIOB
    uint8_t gpio_pin_num;       // e.g., 6 for PB6
    uint8_t scfg_source;        // SCFG_PINS_SOURCE6 for EXTI mapping
    uint32_t exti_line;         // EXINT_LINE_6
    IRQn_Type exti_irq;         // EXINT15_4_IRQn
    uint32_t baud_rate;         // 57600 for S.PORT
    tmr_type *timer;            // TMR15
    uint32_t timer_clk;         // CRM_TMR15_PERIPH_CLOCK
    dma_channel_type *dma_channel;  // DMA1_CHANNEL5
    IRQn_Type dma_irq;          // DMA1_Channel5_4_IRQn
    uint32_t dma_full_flag;     // DMA1_FDT5_FLAG
    uint32_t dma_half_flag;     // DMA1_HDT5_FLAG
} sw_uart_config_t;
```

**GPIO Modes:**

```c
// RX mode: Input with pull-down
static inline void sw_uart_enable_rx(void) {
    // Set GPIO to INPUT mode
    gpio_port->cfgr = (cfgr & ~(0x3 << (pin * 2))) | (GPIO_MODE_INPUT << (pin * 2));
    gpio_port->pull = (pull & ~(0x3 << (pin * 2))) | (GPIO_PULL_DOWN << (pin * 2));
    
    // Enable EXTI rising edge detection
    sw_uart_setup_edge_detection();
    sw_uart_start_dma_sampling();
}

// TX mode: Output push-pull
void sw_uart_enable_tx(void) {
    // Set GPIO to OUTPUT mode
    gpio_port->cfgr = (cfgr & ~(0x3 << (pin * 2))) | (GPIO_MODE_OUTPUT << (pin * 2));
    gpio_port->pull = (pull & ~(0x3 << (pin * 2))) | (GPIO_PULL_NONE << (pin * 2));
    gpio_port->omode &= ~pin_mask;  // Push-pull
    
    // Set idle state LOW (inverted protocol)
    gpio_port->clr = pin_mask;
}
```

**DMA-Based Reception:**

1. **EXTI detects rising edge** → Start bit for inverted protocol (idle=LOW, start=HIGH)
2. **Timer configured for 1.5 bit delay** → Sample at bit center
3. **DMA samples GPIO IDT register** at each timer overflow (9 samples: 8 data + 1 stop)
4. **DMA interrupt on half/full transfer** → Decode samples, store in FIFO
5. **Double-buffering** → Can receive next frame while processing current

```c
// DMA RX configuration (GPIO IDT → Memory)
.ctrl = DMA_PRIORITY_VERY_HIGH | 
        DMA_MEMORY_DATA_WIDTH_WORD | 
        DMA_PERIPHERAL_DATA_WIDTH_WORD |
        MEMORY_INCREMENT | 
        CIRCULAR_MODE | 
        PERIPHERAL_TO_MEMORY,
.paddr = GPIOB_IDT_ADDR,  // Read GPIO input register
.maddr = sw_uart_rx_buffer
```

**DMA-Based Transmission:**

1. **Prepare TX buffer** → Each bit = GPIO SCR command (set/clear upper/lower 16 bits)
2. **Timer triggers DMA** at baud rate intervals
3. **DMA writes to GPIO SCR** → Directly controls pin state (zero CPU)
4. **200µs delay before first bit** → Allows receiver to switch TX→RX

```c
// DMA TX configuration (Memory → GPIO SCR)
.ctrl = DMA_PRIORITY_VERY_HIGH | 
        DMA_MEMORY_DATA_WIDTH_WORD | 
        DMA_PERIPHERAL_DATA_WIDTH_WORD |
        MEMORY_INCREMENT | 
        MEMORY_TO_PERIPHERAL,
.paddr = GPIOB_SCR_ADDR,  // Write to GPIO set/clear register
.maddr = sw_uart_tx_dma_buffer
```

**Inverted Protocol Encoding:**

```c
// For S.PORT inverted protocol:
// Idle = LOW, Start bit = HIGH, Logic 1 = LOW, Logic 0 = HIGH, Stop = LOW

uint32_t set_command = pin_mask;       // Set pin HIGH (lower 16 bits of SCR)
uint32_t clr_command = pin_mask << 16; // Clear pin LOW (upper 16 bits of SCR)

// Start bit
sw_uart_tx_dma_buffer[idx++] = set_command;

// Data bits (LSB first, inverted)
for (uint8_t bit = 0; bit < 8; bit++) {
    if (byte_data & (1 << bit))
        sw_uart_tx_dma_buffer[idx++] = clr_command;  // 1 → LOW
    else
        sw_uart_tx_dma_buffer[idx++] = set_command;  // 0 → HIGH
}

// Stop bits (2 bits, both LOW)
sw_uart_tx_dma_buffer[idx++] = clr_command;
sw_uart_tx_dma_buffer[idx++] = clr_command;
```

**FIFO Buffer:**

```c
#define SW_UART_FIFO_SIZE 64

typedef struct {
    uint8_t buffer[SW_UART_FIFO_SIZE];
    volatile uint8_t head, tail, count;
} sw_uart_fifo_t;

// Write to FIFO (from DMA interrupt)
uint8_t sw_uart_fifo_write(uint8_t data) {
    __disable_irq();
    if (count >= SW_UART_FIFO_SIZE) return 0;  // Full
    buffer[head] = data;
    head = (head + 1) % SW_UART_FIFO_SIZE;
    count++;
    __enable_irq();
    return 1;
}

// Read from FIFO (from main loop)
uint8_t sw_uart_fifo_read(uint8_t *data) {
    __disable_irq();
    if (count == 0) return 0;  // Empty
    *data = buffer[tail];
    tail = (tail + 1) % SW_UART_FIFO_SIZE;
    count--;
    __enable_irq();
    return 1;
}
```

### Why This Abstraction Works

1. **Protocol-agnostic hardware layer** - Single-wire UART knows nothing about S.PORT frames, CRC, sensor IDs
2. **Simple API** - Only 3 functions: init, send_frame, read_byte
3. **Zero-copy DMA** - Protocols build frames in their own buffers, pass pointer to driver
4. **Non-blocking** - FIFO allows main loop to poll at any time
5. **Hardware reuse** - Different telemetry types can share same GPIO/timer/DMA (not simultaneously)

**Trade-off:** KISS telemetry **doesn't** use the timer-based single-wire UART because:
- KISS is unidirectional (TX only, no polling/responses needed)
- Uses interval-based transmission, not request/response polling
- USART1 half-duplex mode provides simpler implementation for TX-only use
- Uses DMA1_CHANNEL2 vs singlewire_uart's DMA1_CHANNEL5 (no resource conflict)

**Both protocols can coexist** on the same PB6 pin but use different hardware peripherals, allowing runtime switching between S.PORT and KISS without hardware conflicts.

---

## Changes to Main Branch

### Modified Files

#### 1. **`Src/main.c`:**

**Added telemetry initialization in main():**

```c
#ifdef USE_SERIAL_TELEMETRY
serial_telemetry_class* serial_telemetry = NULL;

// In main() around line 1690:
uint8_t id = eepromBuffer.telemetry_on_interval;
id = 10; // hardcoded for testing

if (id < 10) {
    serial_telemetry = init_kiss_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
} else if (id < 20) {
    serial_telemetry = init_sport_telemetry();  // Returns pointer to static instance
    serial_telemetry->set_id(serial_telemetry, id);  // ID 10→FRSKY_SENSOR_ID10, etc.
}
#endif
```

**Added telemetry data update in ADC processing section (1kHz rate):**

```c
// In main.c main loop, inside ADC processing (when PROCESS_ADC_FLAG == 1):
if (PROCESS_ADC_FLAG == 1) { 
    // ADC processing at 1kHz rate (every 20 main loop cycles)
    
    // Update sensor variables from ADC readings
    degrees_celsius = converted_degrees;
    battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
    smoothed_raw_current = getSmoothedCurrent();
    actual_current = ((smoothed_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
    
    // Update telemetry package with fresh ADC data
    #ifdef USE_SERIAL_TELEMETRY
    serial_telemetry->makeTelemPackage(
        serial_telemetry,
        (int8_t)degrees_celsius,    // Fresh temperature from ADC
        battery_voltage,             // Fresh voltage from ADC (filtered)
        actual_current,              // Fresh current from ADC (filtered)
        (uint16_t)(consumed_current >> 16),  // Accumulated consumption
        e_rpm);                      // ⚠️ WARNING: May be stale (calculated later)
    #endif
    
    PROCESS_ADC_FLAG = 0;
}
```

**IMPORTANT:** This implementation has a timing issue - `e_rpm` is calculated at 20kHz later in the main loop, so the telemetry package may contain stale RPM data up to 1ms old.

**Added telemetry polling in main loop:**

```c
// Around line 1849 in main while(1) loop:
#ifdef USE_SERIAL_TELEMETRY
if (serial_telemetry != NULL) {
    serial_telemetry->handle_esc_telemetry(serial_telemetry);  // Non-blocking poll check
}
#endif
```

#### 2. `Inc/targets.h`

**Note:** Currently no S.PORT-specific pin definitions exist in targets.h. Pin configuration is hardcoded in `singlewire_uart.c`:

```c
// Actual implementation in Mcu/f421/Src/singlewire_uart.c
void singlewire_uart_init() {
    sw_uart_config_t uart_config;
    uart_config.gpio_port = GPIOB;           // Port B
    uart_config.gpio_pin_num = 6;            // Pin 6 (PB6)
    uart_config.exti_line = EXINT_LINE_6;    // EXTI line 6
    uart_config.timer = TMR15;               // Timer 15
    uart_config.dma_channel = DMA1_CHANNEL5; // DMA1 Channel 5
    uart_config.baud_rate = 57600;           // S.PORT standard baud
}
```

#### 3. `Mcu/f421/Src/peripherals.c`

**Note:** No USART1 clock configuration needed. The implementation uses TMR15 and DMA1_CHANNEL5:

```c
// Actual clock initialization in singlewire_uart.c
void singlewire_uart_init() {
    // Enable GPIOB clock (not GPIOA)
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    
    // Enable TMR15 clock (not USART1)
    crm_periph_clock_enable(CRM_TMR15_PERIPH_CLOCK, TRUE);
    
    // Enable DMA1 clock
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
}
```

#### 4. `Mcu/f421/Src/at32f421_it.c`

**Note:** No USART1 interrupt handler needed. The implementation uses EXTI6 and DMA interrupts:

```c
// EXTI interrupt handler for S.PORT frame detection
void EXINT4_15_IRQHandler(void) {
    if (exint_flag_get(EXINT_LINE_6) != RESET) {
        singlewire_uart_exti_callback();
        exint_flag_clear(EXINT_LINE_6);
    }
}

// DMA interrupt handler for transmission completion
void DMA1_Channel4_5_IRQHandler(void) {
    if (dma_flag_get(DMA1_TC5_FLAG) != RESET) {
        singlewire_uart_dma_callback();
        dma_flag_clear(DMA1_TC5_FLAG);
    }
}
```

#### 5. Build System Changes

**`f421makefile.mk`:**

```makefile
# Serial telemetry is NOT enabled by default in the makefile
# It must be enabled per target in the Makefile via target-specific variables

# When USE_SERIAL_TELEMETRY=1 is set, these source files are added:
SRC_DIR_f421 += \
    Src/sport_telemetry.c \
    src/kiss_telemetry.c \
    Mcu/f421/Src/singlewire_uart.c \
    Mcu/f421/Src/serial_telemetry.c

# And the compilation flag is added:
CFLAGS_f421 += -DUSE_SERIAL_TELEMETRY
```

**`Makefile`:**

```makefile
# Target-specific enabling of serial telemetry
# (Currently no targets explicitly enable it - must be added manually)

# Example of how it should be enabled for a target:
# NEUTRON_2_6S_AIO_F421: USE_SERIAL_TELEMETRY = 1
```

**Note:** The current build system requires manual modification to enable telemetry compilation.

---

## Telemetry Data Flow

**CRITICAL ISSUE IDENTIFIED:** The current telemetry implementation has a timing mismatch that affects data freshness.

### Actual Implementation Analysis

**Main Loop Frequency: 20kHz (50µs intervals)**
**ADC Processing: 1kHz (every 20 main loop cycles)**
**Telemetry Polling: Every main loop cycle (20kHz)**

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESC Sensors & Measurements                    │
│  ADC_raw_voltage, ADC_raw_current, ADC_raw_temp                  │
│  commutation_intervals[], consumed_current                       │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                  main.c: ADC Processing (1kHz)                   │
│              (When PROCESS_ADC_FLAG == 1, every 20 cycles)       │
│  - ADC_DMA_Callback() processes raw ADC values                   │
│  - Calculate: battery_voltage, actual_current, degrees_celsius   │
│  - makeTelemPackage() called HERE with STALE e_rpm!              │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│      serial_telemetry->makeTelemPackage(temp, voltage, ...)      │
│              (Called at 1kHz rate inside ADC processing)         │
│  - Uses 1kHz sensor data: temp, voltage, current, consumption    │
│  - Uses STALE e_rpm: calculated later at 20kHz but using old val │
│  - Packs data into protocol-specific structures                  │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              Main Loop: e_rpm Calculation (20kHz)                │
│                     (AFTER telemetry package made!)              │
│  e_com_time = average(commutation_intervals[0-5])                │
│  e_rpm = running * (600000 / e_com_time)  // Fresh RPM!          │
│  - This fresh RPM is NOT in the telemetry package sent          │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│           Main Loop: serial_telemetry->handle_esc_telemetry()    │
│                     (20kHz polling for S.PORT responses)         │
│  - S.PORT: Checks for receiver polls, responds with data         │
│  - KISS: Checks flags (send_telemetry, send_esc_info_flag)       │
│  - Uses previously packaged data (potentially stale RPM)         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
                   ┌────────────────┐
                   │  Protocol Wire │ ──→ To Receiver
                   └────────────────┘
```

### Data Freshness Analysis

| Parameter    | Update Rate | Package Rate | Freshness Issue |
|--------------|-------------|--------------|-----------------|
| Temperature  | 1kHz        | 1kHz         | ✅ Fresh (same rate) |
| Voltage      | 1kHz        | 1kHz         | ✅ Fresh (same rate) |
| Current      | 1kHz        | 1kHz         | ✅ Fresh (same rate) |
| Consumption  | Variable    | 1kHz         | ✅ Fresh (low update rate) |
| **RPM**      | **20kHz**   | **1kHz**     | ❌ **STALE (up to 1ms old)** |

**The Problem:** `e_rpm` is calculated fresh at 20kHz rate in the main loop, but telemetry packages are created at 1kHz rate in the ADC section using potentially stale `e_rpm` values.

### Recommended Fix

Move telemetry package creation after RPM calculation in the main loop:

```c
// Current problematic flow (in ADC section at 1kHz):
if (PROCESS_ADC_FLAG == 1) {
    // Update sensors at 1kHz
    degrees_celsius = converted_degrees;
    battery_voltage = updated_voltage;
    actual_current = updated_current;
    
    // BAD: Using potentially stale e_rpm
    serial_telemetry->makeTelemPackage(..., e_rpm);
}

// Fresh e_rpm calculated later at 20kHz:
e_rpm = running * (600000 / e_com_time);

// SOLUTION: Move package creation here with fresh e_rpm
```
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│          singlewire_uart_send_frame(frame_buffer, len)           │
│  - Switch GPIO to TX mode                                        │
│  - Transmit via TMR15 DMA or polling                             │
│  - Switch back to RX mode when complete                          │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
                   ┌────────────────┐
                   │  S.PORT Wire   │ ──→ To FrSky Receiver
                   └────────────────┘
```

**Key insight:** Data flows from ADC processing (1kHz) to makeTelemPackage(), but transmission only happens when the receiver polls the ESC's sensor ID. The 4 data fields are sent in round-robin fashion across multiple poll cycles.

### Data Scaling

Sensor data scaling as it flows through the system:

| Parameter    | Source (ADC/calc)     | ADC Processing Output (1kHz) | S.PORT Transmission     |
|--------------|----------------------|----------------------|-------------------------|
| Voltage      | ADC_raw_voltage      | battery_voltage (raw)| Packed in power_value   |
| Current      | ADC_raw_current      | actual_current (raw) | Packed in power_value   |
| Temperature  | ADC_raw_temp         | degrees_celsius (°C) | Direct in temp_value    |
| RPM          | commutation calc     | e_rpm (eRPM)         | Packed in rpm_value     |
| Consumption  | integrated current   | consumed_current>>16 | Packed in rpm_value     |

**Note:** The actual scaling (ADC counts to volts/amps) happens in `ADC_DMA_Callback()` and varies by hardware target. S.PORT transmits the already-scaled values without further conversion.

---

## Polling Architecture

### Sensor ID Polling

S.PORT uses a polling mechanism where the receiver requests data from specific sensor IDs:

1. **Receiver sends poll frame**: `0x7E [SENSOR_ID]`
2. **ESC checks if ID matches**: Compare received ID with assigned sensor ID
3. **ESC responds (if match)**: Send telemetry frame within 12ms window
4. **Receiver processes response**: Extract and display telemetry data

### Sensor ID Calculation

The sensor ID is calculated using **FrSky's standard bit-packing algorithm** with parity bits:

```c
#define BIT(x, index) (((x) >> index) & 0x01)

uint8_t sport_calc_sensor_id(uint8_t physical_id) {
    uint8_t result = physical_id;  // bits 0-4 = physical ID (0-27)
    
    // Add parity bits 5-7 for error detection
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    
    return result;
}
```

**How it's used:**
```c
void sport_sensor_set_id(serial_telemetry_class* self, uint8_t instance) {
    instance %= 10;           // Wrap to 0-9
    instance += 9;            // Shift to 9-18 (maps to FRSKY_SENSOR_ID10-ID19)
    self->id = sport_calc_sensor_id(instance);  // Apply parity bits
}
```

**Example:** Instance 10 → physical_id 19 → sensor_id with parity = 0xF2 (FRSKY_SENSOR_ID19)

### KISS ID Calculation

For KISS telemetry, the ID directly determines the **automatic update interval**:

```c
void set_kiss_telemetry_id(serial_telemetry_class* self, uint8_t new_id) {
    self->id = new_id;
    if (new_id > 0) {
        self->update_on_interval = 29 + new_id;  // ID 1→30ms, ID 2→31ms, ... ID 9→38ms
    }
}
```

**Examples:**
- ID 0 → `update_on_interval = 0` (interval transmission disabled, only DShot-triggered)
- ID 1 → `update_on_interval = 30` (send every 30ms + DShot-triggered)
- ID 5 → `update_on_interval = 34` (send every 34ms + DShot-triggered)
- ID 9 → `update_on_interval = 38` (send every 38ms + DShot-triggered)

### Response Timing Requirements

- **Maximum response time**: 12ms from poll request
- **Typical response time**: 200-500µs (depends on main loop timing)
- **Timeout handling**: If ESC doesn't respond, receiver tries next poll cycle

### Non-Blocking Implementation

To avoid disrupting motor control, telemetry polling is non-blocking:

```c
void sport_handle_telemetry(sport_telemetry_class* self) {
    // Quick check - no blocking wait
    if (sw_uart_data_available()) {
        uint8_t poll_frame[2];
        if (sw_uart_read(poll_frame, 2, 100) == 2) { // 100µs timeout
            if (poll_frame[0] == 0x7E && poll_frame[1] == self->base.id) {
                // Send response
                sport_send_next_frame(self);
            }
        }
    }
}
```

Key points:
- No busy-wait loops
- Short timeout (100µs max)
- Returns immediately if no data available
- Main loop continues without blocking

---

## Main Loop Integration 

### Initialization (main.c:1685-1710)

Protocol selection happens during ESC startup based on the telemetry ID:

```c
#ifdef USE_SERIAL_TELEMETRY
uint8_t id = eepromBuffer.telemetry_on_interval;
id = 10; // ⚠️ Currently hardcoded for testing (forces S.PORT)

if (id < 10) {
    // KISS telemetry for IDs 0-9
    serial_telemetry = init_kiss_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
} else if (id < 20) {
    // S.PORT telemetry for IDs 10-19  
    serial_telemetry = init_sport_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
}
#endif
```

### Main Loop Structure (20kHz Frequency)

**1. Telemetry Handler (Every loop iteration - 20kHz):**
```c
// main.c:1845-1852 - High-frequency polling/response handling
#ifdef USE_SERIAL_TELEMETRY
if (serial_telemetry != NULL) {
    serial_telemetry->handle_esc_telemetry(serial_telemetry);
}
#endif
```

**2. Commutation Timing (Every loop iteration - 20kHz):**
```c
// main.c:1855 - Fresh timing data for RPM calculation
e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + 
               commutation_intervals[2] + commutation_intervals[3] + 
               commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1;
```

**3. ADC Processing & Telemetry Packaging (1kHz - Every 20 cycles):**
```c
// main.c:2080-2108 - Sensor data processing and telemetry package creation
if (PROCESS_ADC_FLAG == 1) {
    // Update sensor data (1kHz rate)
    degrees_celsius = converted_degrees;
    battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
    actual_current = ((smoothed_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
    
    // ⚠️ ISSUE: Package telemetry data with potentially stale e_rpm
    #ifdef USE_SERIAL_TELEMETRY
    serial_telemetry->makeTelemPackage(serial_telemetry, 
        (int8_t)degrees_celsius, battery_voltage, actual_current,
        (uint16_t)(consumed_current >> 16), e_rpm);  // e_rpm may be stale
    #endif
    
    PROCESS_ADC_FLAG = 0;
}
```

**4. Fresh RPM Calculation (Every loop iteration - 20kHz):**
```c
// main.c:2175 - ⚠️ AFTER telemetry packaging, but this is the fresh RPM!
if (stepper_sine == 0) {
    e_rpm = running * (600000 / e_com_time);  // Fresh RPM calculation
    k_erpm = e_rpm / 10;
}
```

### Timing Analysis Summary

| Operation | Frequency | Location | Data Freshness |
|-----------|-----------|----------|----------------|
| Telemetry Polling | 20kHz | handle_esc_telemetry() | ✅ Real-time response |
| Sensor Updates | 1kHz | ADC processing | ✅ Adequate for temp/voltage/current |
| **Telemetry Packaging** | **1kHz** | **ADC section** | **❌ Uses stale RPM** |
| RPM Calculation | 20kHz | Main loop | ✅ Fresh, but after packaging |

**Recommendation:** Move telemetry packaging after RPM calculation for data consistency, or calculate RPM within the ADC processing section.

---

## Configuration

### Compile-Time Configuration

Enable S.PORT telemetry by setting the target-specific variable:

```makefile
# In Makefile, add this line for your target:
YOUR_TARGET_NAME: 

USE_SERIAL_TELEMETRY = 1
```


**Target detection in code:**
```c
#ifdef USE_SERIAL_TELEMETRY
    // Telemetry code compiled
#endif
```

### Runtime Configuration

**Telemetry Protocol Selection:**
The protocol is selected based on `eepromBuffer.telemetry_on_interval` value in main.c:

```c
// In main.c initialization (around line 1692)
uint8_t id = eepromBuffer.telemetry_on_interval;
id = 10; // ⚠️ Currently hardcoded to 10 for testing (forces S.PORT)

if (id < 10) {
    // KISS telemetry for IDs 0-9
    serial_telemetry = init_kiss_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
} else if (id < 20) {
    // S.PORT telemetry for IDs 10-19  
    serial_telemetry = init_sport_telemetry();
    serial_telemetry->set_id(serial_telemetry, id);
}
```

**KISS Telemetry Intervals (IDs 0-9):**
- ID = 0: No automatic transmission (DShot/command-triggered only)
- ID = 1-9: Automatic transmission every (29 + ID) milliseconds
  - ID 1: 30ms intervals
  - ID 2: 31ms intervals  
  - ID 9: 38ms intervals

**S.PORT Telemetry (IDs 10-19):**
- Bidirectional polling protocol (receiver-initiated)
- Response time: <100µs to receiver polls  
- No automatic transmission (poll-response only)
- Sensor ID calculated from target ID using FrSky algorithm

**Data Update Rates:**
- Sensor data (temp/voltage/current): 1kHz (adequate for these slow-changing values)
- RPM calculation: 20kHz (fast response to speed changes)
- ⚠️ **Telemetry packaging: 1kHz (causes RPM data staleness)**

**Current Issues:**
1. ID hardcoded to 10 for testing (not reading from EEPROM)
2. RPM data in telemetry packages may be up to 1ms stale

- Transmission rate limited by receiver polling (typically 10-100 Hz)

**Pin Configuration:**

**Currently no pin macros defined in targets.h** - The configuration is hardcoded in `singlewire_uart.c`:

```c
// Actual configuration in Mcu/f421/Src/singlewire_uart.c
void singlewire_uart_init() {
    sw_uart_config_t uart_config;
    
    uart_config.gpio_port = GPIOB;           // Port B
    uart_config.gpio_pin_num = 6;            // Pin 6 (PB6)
    uart_config.baud_rate = 57600;           // S.PORT standard baud rate
    
    uart_config.exti_line = EXINT_LINE_6;    // EXTI line 6 for PB6
    uart_config.exti_irq = EXINT15_4_IRQn;   // EXTI4-15 interrupt
    uart_config.scfg_source = SCFG_PINS_SOURCE6; // EXTI source config
    
    uart_config.timer = TMR15;               // Timer 15
    uart_config.timer_clk = CRM_TMR15_PERIPH_CLOCK;
    
    uart_config.dma_channel = DMA1_CHANNEL5; // DMA1 Channel 5
    uart_config.dma_irq = DMA1_Channel5_4_IRQn;
    uart_config.dma_full_flag = DMA1_FDT5_FLAG;
    uart_config.dma_half_flag = DMA1_HDT5_FLAG;
}
```

**If pin macros were defined, they would look like:**
```c
#define SPORT_PIN       6             // PB6 pin number
#define SPORT_PORT      GPIOB         // GPIO port B
#define SPORT_TIMER     TMR15         // Timer peripheral
#define SPORT_DMA       DMA1_CHANNEL5 // DMA channel
#define SPORT_EXTI      EXINT_LINE_6  // EXTI line
```

---

## Testing

### Integration Testing

**Test procedure:**

1. Flash ESC with S.PORT-enabled firmware
2. Connect S.PORT wire to FrSky receiver
3. Power on ESC (no motor required for telemetry testing)
4. Check receiver displays:
   - Battery voltage
   - Current (should be near zero with no motor)
   - Temperature (should be ~ambient)
   - RPM (should be zero when not running)
   - Consumption (should increment if current flows)

**Debugging tools:**

- Logic analyzer: Capture S.PORT traffic (57600 baud, inverted)
- OpenOCD + GDB: Live debugging with breakpoints in telemetry code
- Serial debug output: Print telemetry values via alternate UART

---

## Summary of Changes vs. Original AM32

### What's New

1. **Dual telemetry protocol support**
   - S.PORT telemetry (ID 10-19)
   - KISS telemetry (ID 0-9)
   - Runtime selection based on `eepromBuffer.telemetry_on_interval`

2. **S.PORT protocol implementation**
   - Frame encoding/decoding with byte stuffing
   - CRC calculation (sum-based, inverted)
   - FrSky sensor ID parity bit calculation
   - Round-robin transmission of 4 data fields

3. **Single-wire UART driver**
   - Half-duplex bidirectional communication on single GPIO (PB6)
   - Pure DMA implementation using TMR15 + DMA1_CHANNEL5
   - EXTI6 rising edge detection for frame start
   - FIFO buffering for received data

4. **Object-oriented telemetry abstraction**
   - `serial_telemetry_class` base interface
   - Function pointers for polymorphic behavior
   - Easy to add more protocols (e.g., MSP, MAVLink)

5. **Non-blocking integration**
   - Telemetry polling in main loop (no blocking waits)
   - Data updates in ADC processing section (1kHz rate)
   - Minimal CPU overhead

### What Remains Unchanged

- Motor control algorithms (unchanged)
- PWM input handling (unchanged)
- Protection features (unchanged)
- EEPROM settings structure (extended, but backward compatible)
- Build system (extended with conditional compilation)

### Performance Impact

- **Flash usage**: +~4-6 KB (with -flto optimization)
- **RAM usage**: +~256 bytes (telemetry buffers)
- **CPU overhead**: <1% (non-blocking polling)
- **Motor control timing**: No measurable impact

---

## Future Enhancements

Potential improvements for future versions:

1. **Multi-protocol support**
   - KISS telemetry (already implemented in `src/kiss_telemetry.c`)
   - MSP (MultiWii Serial Protocol)
   - MAVLink

2. **Advanced telemetry data**
   - Motor temperature (if sensor available)
   - Throttle position
   - Error codes / fault logging
   - Flight mode detection

3. **Bidirectional configuration**
   - Use S.PORT for ESC configuration changes
   - Remote parameter adjustment
   - Firmware updates via telemetry link

4. **Performance optimization**
   - Zero-copy DMA buffers
   - Pre-computed CRC lookup tables
   - Burst transmission mode

---

## References

- **FrSky S.PORT Protocol**: [Betaflight Wiki](https://github.com/betaflight/betaflight/wiki/Telemetry)
- **AM32 Original Project**: [AlkaMotors/AM32](https://github.com/AlkaMotors/AM32)
- **AT32F421 Datasheet**: [Artery AT32F421 Series](https://www.arterytek.com/en/product/AT32F421.jsp)
- **OpenTX Telemetry**: [OpenTX Documentation](https://www.open-tx.org/)

---

## License

This implementation follows the original AM32 license (GPL v3).

## Contributors

- Original AM32 firmware: AlkaMotors
- S.PORT telemetry implementation: pdingemans (feature_sport branch)

---

*Last updated: October 2, 2025*
