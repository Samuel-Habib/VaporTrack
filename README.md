# VaporTrack: Autonomous Gas-Detection Rover

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform](https://img.shields.io/badge/Platform-STM32F446RE-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS%20v10-brightgreen.svg)](https://www.freertos.org/)
[![Toolchain](https://img.shields.io/badge/Toolchain-arm--none--eabi--gcc-informational.svg)](https://developer.arm.com/downloads/-/gnu-rm)

VaporTrack is an autonomous robotic rover powered by an **STM32F446RE (ARM Cortex-M4F @ 180 MHz)** running **FreeRTOS**. The rover characterizes spatial volatile organic compound (VOC) gas distributions in indoor environments, computes 2D spatial gradients ($\nabla G = [\partial G/\partial x, \partial G/\partial y]$) via weighted planar least-squares regression, and tracks plume sources while executing reactive obstacle avoidance using a 3-sensor ultrasonic array.

---

## 1. System Architecture

```text
                         ┌─────────────────────────┐
                         │   NUCLEO-F446RE (ARM)   │
                         │   180 MHz Cortex-M4F    │
                         │                         │
                         │   FreeRTOS (CMSIS-V2)   │
                         └────────────┬────────────┘
                                      │
             ┌────────────────────────┼────────────────────────┐
             │                        │                        │
        I2C1 (PB6/PB7)           TIM4 / GPIO             TIM1 PWM (PA8/PA10)
             │                        │                        │
      ┌──────┼──────┐           ┌─────┴─────┐            ┌─────┴─────┐
      │      │      │           │           │            │           │
    BME688 BNO055 OLED      HC-SR04 x3   Heartbeat     L298N H-Bridge
    (Gas)  (IMU)  (Diag)   (PC0 - PC5)     (PA5)             │
                                                       ┌─────┴─────┐
                                                       │           │
                                                   Left 4WD    Right 4WD
```

---

## 2. Hardware Specification

| Subsystem | Component | Bus / Interface | Operational Parameters | Role in System |
| :--- | :--- | :--- | :--- | :--- |
| **MCU** | STM32F446RET6 (Nucleo-64) | On-chip | 180 MHz, 512 KB Flash, 128 KB SRAM, FPU | Main controller, RTOS scheduler, spatial math |
| **RTOS** | FreeRTOS Kernel v10.3.1 | Software | CMSIS-RTOS v2 API, preemptive scheduling | Task timing, thread synchronization, IPC |
| **Gas / Environment** | Bosch BME688 | I2C1 (`0x77`, 100 kHz) | Forced mode, 320°C heater (150 ms dwell) | MOX gas resistance ($R_{gas}$), T, P, RH |
| **IMU / Heading** | Bosch BNO055 | I2C1 (`0x28`, 100 kHz) | NDOF fusion mode, internal sensor fusion | Absolute Euler yaw ($\theta$), dead-reckoning |
| **Diagnostics Display**| SSD1306 0.96" OLED | I2C1 (`0x3C`, 100 kHz) | 128x64 monochrome framebuffer | Real-time telemetry, state, gradient vector |
| **Distance Ranging** | 3x HC-SR04 Ultrasonic | GPIO + TIM4 (1 MHz) | Sequential 10 µs trigger, 1 µs timing resolution | Front, Left, and Right obstacle distance |
| **Motor Driver** | L298N Dual H-Bridge | TIM1 PWM + GPIO | 20 kHz PWM (ARR=8999), 2A max per channel | 4WD differential steering control |
| **Chassis & Motors** | YIKESHU 4WD Chassis | Mechanical | 4x DC gearmotors (1:48 gear ratio) | Differential mobility platform |
| **Power Regulation** | DFRobot DFR0831 Buck | DC-DC Step-Down | 7.4V input $\rightarrow$ 5.0V regulated (3A max) | Clean 5V logic & ultrasonic sensor supply |
| **Battery Source** | 2x 18650 Li-ion Cells | Series Pack (2S) | 7.4V nominal (8.4V peak), 2600 mAh | Primary rover energy source |

---

## 3. Electrical & Power Architecture

```text
   2S 18650 Li-ion Battery
      (7.4V - 8.4V)
           │
           ├───[ Main Switch ]───┬───────────────────────────────► L298N VMS (Motor Power)
           │                     │
           │                     ▼
           │           DFRobot Buck Converter (DFR0831)
           │                 (7.4V -> 5.0V Regulated)
           │                     │
           │                     ├───► HC-SR04 Ultrasonic Array (VCC = 5.0V)
           │                     │
           │                     ▼
           │           Nucleo-64 5V / E5V Rail
           │                     │
           │                     ▼
           │           On-Board LDO (5.0V -> 3.3V)
           │                     │
           │                     ├───► STM32F446RE MCU Core & Peripherals
           │                     ├───► Bosch BME688 Gas Sensor (VDD = 3.3V)
           │                     ├───► Bosch BNO055 9-DOF IMU (VDD = 3.3V)
           │                     └───► SSD1306 128x64 OLED Display (VDD = 3.3V)
           │
           └─────────────────────────────────────────────────────► Common Ground Bus (GND)
```

> [!NOTE]
> **Ground Isolation & Noise Mitigation:**
> The motor return current path through the L298N is tied to the central battery ground tab to prevent high-current switching noise from inducing ground bounce on the 3.3V analog and I2C lines. Echo pins PC1, PC3, and PC5 on the STM32F446RE are true **5V-tolerant (FT)** inputs, allowing direct interface with 5V HC-SR04 echo signals without resistive voltage dividers.

---

## 4. Hardware Pinout Map

| Signal Name | STM32 Pin | Function / Alternate Function | Electrical Characteristics | Notes |
| :--- | :--- | :--- | :--- | :--- |
| `I2C1_SCL` | **PB6** | `AF4` (I2C1) | Open-Drain, 4.7 kΩ pull-up | Shared I2C bus clock (BME688, BNO055, SSD1306) |
| `I2C1_SDA` | **PB7** | `AF4` (I2C1) | Open-Drain, 4.7 kΩ pull-up | Shared I2C bus data line |
| `ENA` (Left PWM) | **PA8** | `AF1` (`TIM1_CH1`) | Push-Pull, 20 kHz PWM | Left motor bank duty cycle (ARR=8999) |
| `ENB` (Right PWM)| **PA10**| `AF1` (`TIM1_CH3`) | Push-Pull, 20 kHz PWM | Right motor bank duty cycle (ARR=8999) |
| `IN1` | **PB10** | GPIO Output | Push-Pull, Low Speed | Left motor forward control |
| `IN2` | **PB4** | GPIO Output | Push-Pull, Low Speed | Left motor reverse control |
| `IN3` | **PB5** | GPIO Output | Push-Pull, Low Speed | Right motor forward control |
| `IN4` | **PB3** | GPIO Output | Push-Pull, Low Speed | Right motor reverse control |
| `USS_FRONT_TRIG` | **PC0** | GPIO Output | Push-Pull, High Speed | Front ultrasonic 10 µs trigger pulse |
| `USS_FRONT_ECHO` | **PC1** | GPIO Input (`FT` 5V-tolerant) | Floating / Internal Pull-down | Front echo pulse width measurement |
| `USS_LEFT_TRIG` | **PC2** | GPIO Output | Push-Pull, High Speed | Left ultrasonic 10 µs trigger pulse |
| `USS_LEFT_ECHO` | **PC3** | GPIO Input (`FT` 5V-tolerant) | Floating / Internal Pull-down | Left echo pulse width measurement |
| `USS_RIGHT_TRIG`| **PC4** | GPIO Output | Push-Pull, High Speed | Right ultrasonic 10 µs trigger pulse |
| `USS_RIGHT_ECHO`| **PC5** | GPIO Input (`FT` 5V-tolerant) | Floating / Internal Pull-down | Right echo pulse width measurement |
| `STATUS_LED` | **PA5** | `AF1` (`TIM2_CH1`) | Push-Pull PWM | Activity heartbeat LED (duty $\propto$ motor drive) |

---

## 5. Software Architecture & FreeRTOS Tasks

The application runs 6 prioritized, decoupled FreeRTOS tasks. Inter-task communication follows a **producer-consumer snapshot model**: sensors update a shared state struct protected by a FreeRTOS mutex (`g_sensor_mutex`), and consumer tasks copy local snapshots to eliminate torn reads without holding the bus.

In addition, access to the shared physical I2C1 bus (`hi2c1`) between `SensorTask` (reading BME688 and BNO055) and `DisplayTask` (flushing 1024-byte framebuffer to SSD1306) is serialized using a dedicated FreeRTOS mutex (`g_i2c_mutex`). To prevent bus starvation during BME688 gas sensor heating, the driver splits forced-mode triggering and data readout: `SensorTask` initiates the measurement, yields CPU execution via `osDelay(160)` while the heater stabilizes, and acquires `g_i2c_mutex` again only when measurement registers are ready to fetch.

```text
                   SensorTask       UltrasonicTask
                  (BME688/BNO055)     (HC-SR04 x3)
                        │                  │
                        ▼                  ▼
                ┌─────────────────────────────────┐
                │    Thread-Safe Sensor Snapshot  │  (sensors.c: g_sensors)
                └────────────────┬────────────────┘
                                 │
                     ┌───────────┴───────────┐
                     ▼                       ▼
                MappingTask           NavigationTask (State Machine)
             (2D Least-Squares)              │
                     │                       ▼
                     └──────────────►   MotorTask (L298N PWM/GPIO)
                                             │
                                             ▼
                                        DisplayTask (SSD1306 OLED)
```

### Task Scheduling Table

| Task Name | Period | Priority | Stack Size | Role & Responsibilities |
| :--- | :--- | :--- | :--- | :--- |
| **`MotorTask`** | 50 ms | `osPriorityHigh` (40) | 256 words (1 KB) | Translates navigation velocity commands to TIM1 PWM compare values and GPIO directions. |
| **`SensorTask`** | 250 ms | `osPriorityAboveNormal` (32) | 512 words (2 KB) | Triggers BME688 forced-mode gas read and polls BNO055 Euler heading in NDOF mode. |
| **`NavTask`** | 150 ms | `osPriorityAboveNormal` (32) | 512 words (2 KB) | Dead-reckoning position integration, state machine updates, and obstacle avoidance overrides. |
| **`USSTask`** | 100 ms | `osPriorityNormal` (24) | 256 words (1 KB) | Sequentially fires HC-SR04 triggers using TIM4 1 MHz counter with 1 ms inter-ping decay gaps. |
| **`MapTask`** | 500 ms | `osPriorityNormal` (24) | 512 words (2 KB) | Logs spatial samples and solves 2D planar weighted least-squares system for $\nabla G$. |
| **`DispTask`** | 500 ms | `osPriorityBelowNormal` (16) | 512 words (2 KB) | Formats telemetry strings and flushes 1024-byte framebuffer to SSD1306 OLED over I2C. |

---

## 6. Gas Mapping & Gradient Localization Theory

Standard plume-tracking approaches rely on instantaneous concentration thresholds (`if gas > threshold -> move forward`). In indoor environments, turbulent eddies and diffusion create local pockets of high concentration, causing naive rovers to stall or oscillate. VaporTrack uses a **two-pass planar spatial gradient estimation**:

### 1. Spatial Sample Acquisition
As the rover maneuvers, it logs discrete coordinate tuples:
$$S_i = (x_i, y_i, \theta_i, G_i)$$
Where $(x_i, y_i)$ is dead-reckoned position derived from heading $\theta_i$ and differential wheel displacement.

Because the BME688 metal-oxide semiconductor (MOX) sensor's raw resistance $R_{\text{gas}}$ drops when reducing VOC vapors interact with its heated sensing layer, the rover converts raw resistance into relative conductance $G_i$ ($\mu\text{S}$):
$$G_i = \frac{10^6}{R_{\text{gas}, i}}$$
This inversion guarantees that $G_i$ increases proportionally with gas plume concentration, allowing standard gradient ascent to naturally steer toward the gas source without sign inversion.

### 2. Weighted Planar Least-Squares Regression
In the localization phase, the rover models the local gas distribution in a neighborhood of radius $R$ as a first-order planar surface centered at $(x, y)$: $G(x_i, y_i) \approx a + b \cdot \Delta x_i + c \cdot \Delta y_i$, where $\Delta x_i = x_i - x$ and $\Delta y_i = y_i - y$.

Setting up the normal equations: $\begin{bmatrix} \sum \Delta x_i^2 & \sum \Delta x_i \Delta y_i \\ \sum \Delta x_i \Delta y_i & \sum \Delta y_i^2 \end{bmatrix} \begin{bmatrix} b \\ c \end{bmatrix} = \begin{bmatrix} \sum \Delta x_i G_i \\ \sum \Delta y_i G_i \end{bmatrix}$

Solving via Cramer's rule: $D = \left(\sum \Delta x_i^2\right)\left(\sum \Delta y_i^2\right) - \left(\sum \Delta x_i \Delta y_i\right)^2$

$b = \frac{\left(\sum \Delta x_i G_i\right)\left(\sum \Delta y_i^2\right) - \left(\sum \Delta y_i G_i\right)\left(\sum \Delta x_i \Delta y_i\right)}{D}$

$c = \frac{\left(\sum \Delta x_i^2\right)\left(\sum \Delta y_i G_i\right) - \left(\sum \Delta x_i \Delta y_i\right)\left(\sum \Delta x_i G_i\right)}{D}$

### 3. Gradient Ascent Steering
The estimated gradient vector $\nabla G$ gives the direction of steepest concentration change:
$$\nabla G = \left[ \frac{\partial G}{\partial x}, \frac{\partial G}{\partial y} \right] = [b, c]$$
$$\theta_{\text{target}} = \text{atan2}(c, b), \quad \|\nabla G\| = \sqrt{b^2 + c^2}$$

The rover computes heading error $e_\theta = \theta_{\text{target}} - \theta_{\text{current}}$ and applies differential steering proportional to $e_\theta$. When $\|\nabla G\|$ drops below the planar detection threshold and $G$ exceeds the plume peak threshold, the rover transitions to `NAV_STATE_SOURCE_FOUND`.

---

## 7. Navigation State Machine & Safety Overrides

```text
    ┌──────────────┐
    │  STATE_BOOT  │
    └──────┬───────┘
           │ Hardware peripherals ready
           ▼
    ┌──────────────┐
    │ INIT_SENSORS │
    └──────┬───────┘
           │ I2C devices acknowledged
           ▼
    ┌──────────────┐
    │ CALIBRATION  │◄────────┐ BNO055 NDOF cal dance
    └──────┬───────┘         │ (sys >= 1, gyro+mag >= 2)
           │ Calibration OK  │
           ▼                 │
    ┌──────────────┐         │
    │   MAPPING    ├─────────┘
    └──────┬───────┘
           │ Waypoint quota reached (>= 64 samples)
           ▼
    ┌──────────────┐
    │ LOCALIZATION │
    └──────┬───────┘
           │ Valid gradient computed (|grad| > threshold)
           ▼
    ┌──────────────┐
    │   TRACKING   ├───────────────────────────────┐
    └──────┬───────┘                               │
           │ Gas peak reached & grad flat          │
           ▼                                       ▼
    ┌──────────────┐                      ┌─────────────────┐
    │ SOURCE_FOUND │                      │ OBSTACLE_AVOID  │
    └──────────────┘                      └─────────────────┘
                                           ▲ (Preempts any state
                                           │  if d_front < 20 cm)
```

* **Obstacle Preemption:** If `dist_front_cm < 20.0 cm` or either side sensor detects an obstacle `< 15.0 cm`, the rover immediately halts forward movement and transitions into `NAV_STATE_OBSTACLE_AVOID`.
* **Escape Maneuver:** The rover pivots away from the closer side until clearance exceeds `40.0 cm`, then restores the previous state to resume gradient navigation.

---

## 8. Repository Layout

```text
VaporTrack/
├── Core/
│   ├── Inc/
│   │   ├── bme688.h           # Bosch BME688 MOX gas & environmental driver header
│   │   ├── bno055.h           # Bosch BNO055 9-DOF IMU & Euler orientation header
│   │   ├── hcsr04.h           # 3-channel HC-SR04 ultrasonic driver header
│   │   ├── ssd1306.h          # 128x64 OLED framebuffer driver & font header
│   │   ├── motor.h            # L298N differential drive & PWM interface header
│   │   ├── sensors.h          # Mutex-protected thread-safe sensor snapshot header
│   │   ├── mapping.h          # 2D spatial sample grid & least-squares gradient header
│   │   ├── navigation.h       # Autonomous finite state machine & obstacle avoidance header
│   │   ├── main.h             # Pin assignments, peripheral exports, and clock macros
│   │   └── FreeRTOSConfig.h   # FreeRTOS kernel configuration (32 KB heap, FPU enabled)
│   └── Src/
│       ├── bme688.c           # BME688 register access & datasheet compensation formulas
│       ├── bno055.c           # BNO055 NDOF mode setup & Euler heading acquisition
│       ├── hcsr04.c           # TIM4 microsecond pulse width timing & distance calculation
│       ├── ssd1306.c          # Framebuffer manipulation & 6x8 ASCII font renderer
│       ├── motor.c            # TIM1 PWM duty update & GPIO direction control
│       ├── sensors.c          # Global sensor snapshot & FreeRTOS mutex synchronization
│       ├── mapping.c          # Planar regression solver using Cramer's rule
│       ├── navigation.c       # Navigation state machine, steering laws & obstacle overrides
│       ├── main.c             # System clock setup, peripheral init & FreeRTOS task threads
│       └── stm32f4xx_hal_msp.c# Low-level MSP peripheral clock and GPIO initialization
├── Drivers/                   # STMicroelectronics STM32F4xx HAL & ARM CMSIS sources
├── Middlewares/               # FreeRTOS Kernel v10.3.1 sources and CMSIS-RTOS v2 wrapper
├── cmake/                     # CMake toolchain definitions for arm-none-eabi-gcc
├── vapor_track.ioc            # STM32CubeMX graphical configuration file
├── CMakeLists.txt             # Root CMake build configuration
├── commands.sh                # Portable build and test automation script
├── openocd.cfg                # OpenOCD target configuration for ST-Link V2/V2-1
└── LICENSE                    # MIT License
```

---

## 9. Building and Flashing

### Toolchain Prerequisites
* **CMake** $\ge$ 3.22
* **Ninja** or **Make**
* **GNU Arm Embedded Toolchain** (`arm-none-eabi-gcc` $\ge$ 10.3)
* **stlink-tools** or **OpenOCD**

### Compilation
```bash
# Clone repository
git clone https://github.com/Samuel-Habib/VaporTrack.git
cd VaporTrack

# Configure build directory
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -B build

# Build ELF and generate binary
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)
```

### Flashing to STM32 Target
```bash
# Flash using CMake target (via st-flash)
cmake --build build --target flash

# Or flash binary directly with st-flash
st-flash --connect-under-reset --reset write build/vapor_track.bin 0x08000000

# Or using OpenOCD
openocd -f openocd.cfg -c "program build/vapor_track.elf verify reset exit"
```

---

## 10. Bench Calibration & Verification

1. **IMU Heading Calibration:** On power-up, the rover remains stationary for 3 seconds to let the gyroscope calibrate, then executes a slow rotational pivot. The OLED displays `CAL S:<sys> G:<gyro> M:<mag>`. Once `mag >= 2` and `gyro == 3`, calibration locks.
2. **Ultrasonic Ranging Verification:** Each channel can be probed on PC1, PC3, and PC5 with a logic analyzer. A 10 µs pulse on trigger generates an echo high pulse between 116 µs (2 cm) and 23.2 ms (400 cm).
3. **VOC Sensor Response Verification:** Exposing the BME688 to an isopropyl alcohol vapor source causes the MOX sensor resistance $R_{gas}$ to drop sharply from $\sim 150\text{ k}\Omega$ to $< 10\text{ k}\Omega$. The OLED reflects real-time $R_{gas}$ drops and computes gradient vector arrows on the display.
4. **Motor Differential Trim:** If the chassis drifts slightly under equal PWM duty cycles due to motor gearbox variance, adjust `NAV_SPEED_CRUISE` offsets in `Core/Inc/navigation.h`.

---

## 11. License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
