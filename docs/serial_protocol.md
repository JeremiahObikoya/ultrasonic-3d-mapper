# Serial Protocol

**Baud rate:** 115200

All messages are single-line, comma-separated.

## Messages (Arduino → PC)

| Message | Meaning |
|---|---|
| `D,<yaw>,<pitch>,<distance_cm>,<mode>` | Data point. `mode`: `A` = auto-scan, `J` = joystick. `distance = -1.0` means no valid echo. |
| `S,START` | Automatic scan started |
| `S,END` | Automatic scan complete |
| `M,AUTO` | Mode changed to automatic scan |
| `M,JOY` | Mode changed to joystick |
| `# <text>` | Info / debug message (ignored by the mapper) |

## Example Session

```
# Scanner ready
M,JOY
M,AUTO
S,START
D,20.0,60.0,142.3,A
D,25.0,60.0,138.7,A
D,30.0,60.0,135.1,A
...
S,END
S,START
...
```

## Coordinate Convention

The mapper converts `(yaw, pitch, distance)` to Cartesian `(X, Y, Z)` in centimetres:

- **X** = right
- **Y** = forward (away from the sensor)
- **Z** = up

The sensor sits at the origin `(0, 0, 0)`.
