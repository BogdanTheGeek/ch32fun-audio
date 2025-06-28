# Audio Playback on a CH32V00X
Example code for playing audio on a CH32V006 using PWM and DMA.

## Hardware
Small speaker connected to PC3 though a large capacitor(in series), at least 10uF.

The example PCB is designed for experimentation, therefore it also includes USB, SPI-FLASH and a can use the internal OPAMP as a second order low-pass filter. All of these features are optional.
<img width="794" alt="image" src="https://github.com/user-attachments/assets/7d49fe85-f6db-43ad-9c5c-9b3c44e674cc" />

## Getting Started
1. Get everything:
```sh
git clone --recurse-submodules
```
2. Build tools:
```sh
make tools
```
3. Convert audio file to code:
```sh
./tools/wav2code beep-boop.wav > samples.h
```
4. Build and flash:
```sh
make
```
5. Run and interract with the program:
```sh
make monitor
```

> [!WARNING]
> Make sure the audio file is encoded as 16-bit signed PCM, mono, and 32kHz sample rate(this can be adjusted).

