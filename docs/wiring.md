# Wiring

## Pin Connections

| Component | Arduino Pin | Notes |
|---|---|---|
| Joystick X axis | A0 | Analogue |
| Joystick Y axis | A1 | Analogue |
| Joystick Button | A2 | Uses internal pull-up (`INPUT_PULLUP`) |
| Ultrasonic TRIG | D4 | Digital output |
| Ultrasonic ECHO | D3 | Digital input |
| Pitch Servo (signal) | D6 | PWM |
| Yaw Servo (signal) | D5 | PWM |

## Power Notes

> **Important:** Power both servos from an **external 5 V supply**, not the
> Arduino's 5 V pin. Two SG90 servos under load can draw 500+ mA, which will
> cause brownout resets if powered from the Arduino's onboard regulator.

- Connect the external 5 V supply **GND** to the Arduino **GND** (common ground).
- The ultrasonic sensor and joystick can be powered from the Arduino's 5 V pin.

## Schematic

```
                  ┌──────────────┐
                  │  Arduino Uno │
                  │              │
  Joystick X ────┤ A0           │
  Joystick Y ────┤ A1           │
  Joystick Btn ──┤ A2           │
                  │              │
  Echo ──────────┤ D3           │
  Trig ──────────┤ D4           │
  Yaw Servo ─────┤ D5 (PWM)     │
  Pitch Servo ───┤ D6 (PWM)     │
                  │              │
                  │          GND ├─── Common GND
                  └──────────────┘
```
