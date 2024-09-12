# NIP-91 POC

https://github.com/nostr-protocol/nips/blob/iot/91.md

Get sensor data and push to a relay

## Notes

### Configuration

Copy `config.example.h` to `config.h` and add hex values for your nostr sk and pk

### Connections

For this to work connect the BME280 to the ESP32 as follows:

- VCC -> 3.3V (of ESP32)
- GND -> GND
- SDA -> GPIO21 (Default SDA for ESP32)
- SCL -> GPIO22 (Default SCL for ESP32)

If you're using a different I2C address for your BME280 (some modules may use 0x77), you'll need to modify the bme.begin(0x76) line accordingly.