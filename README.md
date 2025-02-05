# PicoMEML

Example applications using the meMLP library.

## Build

Use the Arduino IDE with Earl Philhower's `arduino-pico` extension: [Official documentation](https://arduino-pico.readthedocs.io)

Build and run within the Arduino IDE by activating one of the three macros in `src/PicoDefs.hpp`:
- `FM_SYNTH` - Joystick-controlled FM synth (audio: mono output only)
- `FX_PROCESSOR` - Joystick-controlled multi FX processor (audio: mono input, mono output)
- `EUCLIDEAN` - Joystick controlled Euclidean sequencers (GPIO output only)

## Documentation

Further documentation in [this presentation](doc/lecturenotes.odp).
