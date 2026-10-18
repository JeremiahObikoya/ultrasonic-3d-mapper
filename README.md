# 3D Ultrasonic Room Scanner

Pan/tilt servo rig with an HC-SR04 ultrasonic sensor, controlled by an Arduino,
with a live 3D point cloud visualisation in Python.

<!-- Add a photo or GIF of the project here:
![demo](docs/demo.gif)
-->

## Project Structure

```
3d_mapping/
├── firmware/              Arduino sketch (runs on the microcontroller)
│   └── pan_tilt_scanner/
├── mapper/                Python 3D visualiser (runs on your PC)
│   ├── mapper.py
│   └── requirements.txt
├── tests/                 Hardware test sketches
│   └── test_all_components/
└── docs/                  Wiring, protocol, and reference
```

## Hardware

- Arduino Uno (or Nano / compatible)
- 2× SG90 micro servo (pan + tilt)
- HC-SR04 ultrasonic distance sensor
- Analogue joystick module (with button)
- External 5 V power supply for servos

See [docs/wiring.md](docs/wiring.md) for the full pin-out and schematic.

## Quick Start

### 1. Flash the Firmware

Open `firmware/pan_tilt_scanner/pan_tilt_scanner.ino` in the
[Arduino IDE](https://www.arduino.cc/en/software) and upload to your board.

### 2. Run the Mapper

```bash
cd mapper
pip install -r requirements.txt

python mapper.py --port COM3            # Windows
python mapper.py --port /dev/ttyUSB0    # Linux
python mapper.py --demo                 # no hardware — simulated room
```

### 3. Controls

| Input | Action |
|---|---|
| Joystick | Aim the sensor (velocity control) |
| Joystick button | Toggle joystick ↔ auto-scan mode |
| `d` key (plot window) | Clear the map |
| `p` key (plot window) | Flip Pitch inversion (Up ↔ Down) |
| `y` key (plot window) | Flip Yaw inversion (Left ↔ Right) |

## How It Works

1. The Arduino sweeps two servos in a serpentine raster pattern (or follows the joystick).
2. At each angle it fires the ultrasonic sensor and averages 3 readings.
3. `(yaw, pitch, distance)` tuples are streamed over serial at 115200 baud.
4. The Python mapper converts spherical → Cartesian coordinates and renders a
   real-time 3D point cloud coloured by echo distance.

See [docs/serial_protocol.md](docs/serial_protocol.md) for the full message format.

## License

[MIT](LICENSE)

