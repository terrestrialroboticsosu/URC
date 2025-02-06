# HAL
Hardware Abstraction Layer (HAL) is the code for the RP2040 feather board that interacts with the sensors and actuators on the robot. 

## Hardware
The HAL runs on the [Adafruit RP2040 CAN Bus Feather](https://www.adafruit.com/product/5724?gad_source=1&gclid=CjwKCAjw6JS3BhBAEiwAO9waF6-QUOsvE4WANgtbXqKQ9XEZZYqk7GmbhO3cfyLxJRCH3CGZkDhGFxoCfXMQAvD_BwE).

## Setup
Install Arduino-CLI: 

Install the RP2040 Board files: 
```bash
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core install rp2040:rp2040
```

Install the libraries: 
```bash
arduino-cli lib install "Adafruit MCP2515"
```

## Compilation
To compile: 
```bash
arduino-cli compile -b rp2040:rp2040:adafruit_feather_can
```

## Upload
To upload, run the compile command with the `--upload` flag: 
```bash
arduino-cli compile -b rp2040:rp2040:adafruit_feather_can --upload
```
