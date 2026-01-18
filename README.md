# Motor Control Joycon - ESP32

A wireless motor control system using Nintendo Joycon controllers to drive two NEMA17 stepper motors through a ESP32 microcontroller.

## Overview

This project enables intuitive control of dual stepper motors using a Nintendo Joycon controller in wireless mode. The ESP32 automatically detects and connects to a Joycon when placed in pairing mode, and the joystick axes directly control motor speed and direction.

## Features

- **Wireless Control**: Automatic Joycon pairing and connection via Bluetooth
- **Dual Motor Control**: Independent speed control for two NEMA17 stepper motors
- **Adjustable Speed**: Two speed modes (normal and fine adjustment) selectable via X button
- **Real-time Feedback**: Serial output for debugging and monitoring

## Hardware Requirements

### Components
- **ESP32 Development Board** - Main controller
- **2x NEMA17 Stepper Motors** - Load motors
- **2x TB6600 Microstep Motor Driver** - Motor drivers with microstepping capability
- **Nintendo Joycon Controller** - Wireless input device
- **USB Power Supply** - 5V for ESP32 and drivers

### Power Specifications
- ESP32: 5V USB
- TB6600 Drivers: Logic 3.3V (from ESP32), Power 12-24V (depends on motor specifications)

## Hardware Wiring

### Pin Configuration

| Component | ESP32 Pin | Function |
|-----------|-----------|----------|
| Motor 1 - Step | GPIO 23 | Pulse signal |
| Motor 1 - Direction | GPIO 22 | Direction control |
| Motor 2 - Step | GPIO 21 | Pulse signal |
| Motor 2 - Direction | GPIO 19 | Direction control |
| Enable (Motors) | GPIO 0 | Active high enable |
| GND | GND | Common ground |

### Wiring Diagram

```
                                    ┌──────────────────┐
                                    │     ESP32        │
                                    │                  │
                                    │  USB Power       │
                                    └──────────────────┘
                                            ▲
                                            │
                                    USB Power Supply
                                       (5V, 1A+)
                                            │
                    ┌───────────────────────┼───────────────────────┐
                    │                       │                       │
              ┌─────▼────┐           ┌─────▼────┐           ┌─────▼────┐
              │  TB6600   │           │  TB6600  │           │  GND     │
              │  Motor 1  │           │  Motor 2 │           │  Rail    │
              │ Driver    │           │ Driver   │           │          │
              └─────┬────┘           └─────┬────┘           └──────────┘
                    │                       │
          (Motor Power 12-24V)     (Motor Power 12-24V)
          
ESP32 GPIO Connections:
────────────────────────
GPIO 23 ──► TB6600-1 PUL (Step)
GPIO 22 ──► TB6600-1 DIR (Direction)
GPIO 21 ──► TB6600-2 PUL (Step)
GPIO 19 ──► TB6600-2 DIR (Direction)
GPIO 0  ──► TB6600 ENA (Enable) - Common to both drivers
GND     ──► TB6600 GND (Logic Ground)

NEMA17 Motor Connections:
─────────────────────────
TB6600-1 A+ ──► Motor 1 Coil A+
TB6600-1 A- ──► Motor 1 Coil A-
TB6600-1 A+ ──► Motor 1 Coil B+
TB6600-1 A- ──► Motor 1 Coil B-

TB6600-2 B+ ──► Motor 2 Coil A+
TB6600-2 B- ──► Motor 2 Coil A-
TB6600-2 B+ ──► Motor 2 Coil B+
TB6600-2 B- ──► Motor 2 Coil B-
```

## Control Mapping

### Joystick Input
- **Left Joystick X-Axis**: Controls Motor 1 speed and direction
  - Left: Negative direction, increasing speed
  - Right: Positive direction, increasing speed
  - Center: Stop
  
- **Left Joystick Y-Axis**: Controls Motor 2 speed and direction
  - Up: Negative direction, increasing speed
  - Down: Positive direction, increasing speed
  - Center: Stop

### Button Controls
- **X Button**: Toggle fine speed adjustment mode
  - When pressed: Speed reduces to 1/10th of normal speed for precise control
  - When released: Returns to normal speed

## Software Setup

### Prerequisites
- Arduino IDE with ESP32 board support installed
- Libraries:
  - [Bluepad32](https://github.com/ricardoquesada/bluepad32) - Joycon controller library
  - joycon_esp32 by gjlp25 (alternative/complementary)

### Configuration

Open `MotorControlJoycon.ino` and adjust these parameters:

```cpp
// Motor speeds (revolutions per second)
constexpr float NORMAL_RPS = 0.5;    // Normal speed
constexpr float FINE_RPS = 0.05;     // Fine adjustment speed (10% of normal)

// Motor directions (invert to reverse motor direction)
constexpr bool MOTOR1DIRECTION = true;
constexpr bool MOTOR2DIRECTION = true;

// Joycon calibration (adjust based on your controller's idle values)
const int JOYCON_IDLE_X = -55;
const int JOYCON_IDLE_Y = 13;
const int DEADZONE_THRESHOLD = 25;
```

### Installation Steps

1. Clone or download this repository
2. Open `MotorControlJoycon.ino` in Arduino IDE
3. Select **Tools > Board > ESP32-WROOM-32**
4. Select the correct **COM port** for your ESP32
5. Install required libraries via **Sketch > Include Library > Manage Libraries**
6. Upload the sketch to your ESP32
7. Open Serial Monitor (115200 baud) to verify operation

## Operating Instructions

1. **Power Up**: Connect ESP32 via USB
2. **Put Joycon in Pairing Mode**: Press pairing button on Joycon (usually takes 2-5 seconds)
3. **Auto Connection**: ESP32 will automatically detect and connect to the Joycon
4. **Control Motors**: Use left joystick to control motor speeds
5. **Fine Adjustment**: Press X button for precise speed control
6. **Stop**: Center the joystick to stop both motors

## Troubleshooting

### Joycon Not Connecting
- Check that Joycon is in pairing mode
- Verify Bluepad32 library is properly installed
- Check Serial Monitor for connection messages

### Motors Not Responding
- Verify GPIO pin assignments match your ESP32
- Check TB6600 driver power supply (12-24V)
- Ensure motor coil connections are correct
- Verify Enable pin (GPIO 0) is set HIGH

### Unusual Motor Behavior
- Check motor direction settings (MOTOR1DIRECTION, MOTOR2DIRECTION)
- Verify Joycon calibration values in code
- Test individual motors with Serial Monitor output

### Speed Too Fast/Slow
- Adjust NORMAL_RPS and FINE_RPS values
- Check motor microstep settings on TB6600 driver switches
- Verify step timer period calculations

## Project Structure

```
MotorControlRepo/
├── MotorControlJoycon.ino    # Main firmware
├── README.md                  # This file
└── (Compiled binaries)
```

## Technical Details

### Motor Control Implementation
- **Timer-based Pulse Generation**: Uses ESP32 hardware timer for precise step pulse generation
- **Microstepping**: TB6600 drivers configured for microstepping via DIP switches
- **1600 Steps/Revolution**: Default configuration assumes 1/8 microstepping
- **Frequency Range**: ~0.05 to 0.5 RPS (revolutions per second)

### Bluetooth Connection
- Bluepad32 library handles Joycon Bluetooth protocol
- Automatic reconnection after power loss
- Support for up to 4 simultaneous controllers (only 1 used in this project)

## Safety Considerations

⚠️ **Important**:
- Ensure motors are mechanically loaded before testing (prevent overspeeding)
- Use appropriate power supply with overcurrent protection
- Add emergency stop mechanism if used in critical applications
- Keep fingers away from rotating motor shafts
- Verify all connections before power-up

## Future Improvements

- [ ] Add multiple controller support for coordinated control
- [ ] Implement acceleration/deceleration ramping
- [ ] Add telemetry feedback (current sensing, encoder feedback)
- [ ] Create mobile app alternative to Joycon control
- [ ] Add preset position/speed profiles

## References

- [Bluepad32 Documentation](https://github.com/ricardoquesada/bluepad32)
- [ESP32 Technical Reference](https://www.espressif.com/en/products/microcontrollers/esp32/overview)
- [TB6600 Stepper Driver Datasheet](https://www.omc-stepperonline.com/download/TB6600_Stepper_MotorDriver_datasheet.pdf)
- [NEMA17 Specifications](https://github.com/gjlp25/joycon_esp32)

## License

This project is provided as-is for educational and personal use.

---

**Last Updated**: January 2026
**Tested On**: ESP32-WROOM-32, Bluetooth 4.2
