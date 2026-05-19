# Galvo Controller

A high-performance, web-based G-code controller for laser galvanometer scanners.

This project uses a Raspberry Pi and a custom C extension leveraging `pigpio` to achieve 15kpps+ hardware-timed SPI streaming. By utilizing Direct Memory Access (DMA) and multi-processing, it guarantees deterministic hardware timing, bypassing standard Linux kernel jitter while providing a zero-latency WebSockets UI.

## Hardware Requirements


- **Raspberry Pi**: Raspberry Pi 3 B+, 4, or Zero 2 W recommended. (Must use a Broadcom CPU for `pigpio` memory mapping; Pi 5 or Allwinner-based boards are not supported).


- **DAC**: Dual-channel SPI DAC (e.g., MCP4922) for X/Y analog control.


- **Laser Control**: Standard 5V PWM logic.



## Schematic

Refer to the included schematic for wiring the Raspberry Pi to the DAC, laser: `Schematic/schematic.png`



## Installation (Docker - Recommended)

Running via Docker Compose isolates the dependencies while passing through the necessary `/dev/mem` privileges for hardware control.


1. Ensure the default OS-level `pigpiod` daemon is stopped and disabled:

```
`sudo systemctl stop pigpiod sudo systemctl disable pigpiod sudo killall pigpiod   
`
```


1. Build and start the container:

```
`docker compose up -d --build   
`
```



The application will be available at `http://<YOUR_PI_IP>:5000`.

## Installation (Bare Metal)

If you prefer to run the application directly on the host OS, you must compile `pigpio` from source and run the application as root.


1. Install system dependencies:

```
`sudo apt-get update sudo apt-get install -y build-essential wget unzip python3-pip   
`
```


1. Download and compile `pigpio` from source:

```
`cd /tmp wget [https://github.com/joan2937/pigpio/archive/master.zip](https://github.com/joan2937/pigpio/archive/master.zip) unzip master.zip cd pigpio-master make sudo make install   
`
```


1. Install Python requirements:

```
`cd /path/to/GalvoCon pip3 install -r requirements.txt   
`
```


1. Compile the native C extension:

```
`gcc -shared -o galvo_core.so -fPIC galvo_core.c -lpigpio -lpthread   
`
```


1. Run the server (Requires `sudo` for DMA memory access):

```
`sudo python3 app.py   
`
```



## Usage


1. Navigate to the web interface via a browser on the same network.


1. Upload standard `.gcode` or `.nc` files.


1. Use the **Frame Box** feature to physically trace the bounding box using the red dot laser.


1. Adjust the **Speed Multiplier** and **PWM Max** sliders in real-time during burns.



## Safety Warning

This software overrides standard OS scheduling to stream laser commands at extremely high speeds. Always wire a physical hardware cutoff switch to the interlock pin and ensure appropriate laser safety enclosures and eyewear are used during operation.
