# MS5849_30BA Arduino Library

Arduino IDE library for the TE Connectivity MS5849-30BA 30 bar absolute pressure sensor using I2C.

## Wiring

The MS5849-30BA is a 3.3 V device. If you use a 5 V Arduino board, use a proper I2C level shifter.

| Sensor | Arduino |
| --- | --- |
| VDD | 3.3 V |
| GND | GND |
| SDA | SDA |
| SCL | SCL |

Default I2C address in the example is `0x77` (`ADDRESS_GND`). Some boards/jumpers use `0x76` (`ADDRESS_VCC`).

## Install

1. Download the ZIP.
2. Arduino IDE: **Sketch > Include Library > Add .ZIP Library...**
3. Open **File > Examples > MS5849_30BA > MS5849_30BA_Basic**.

## Basic use

```cpp
#include <Wire.h>
#include <MS5849_30BA.h>

MS5849_30BA sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz I2C — sensor supports it
  sensor.begin(MS5849_30BA::ADDRESS_GND, MS5849_30BA::OSR_6, MS5849_30BA::OSR_1);
}

void loop() {
  float pressureMbar, temperatureC;
  if (sensor.read(pressureMbar, temperatureC)) {
    Serial.println(pressureMbar);
    Serial.println(temperatureC);
  }
  delay(100);
}
```

## Independent oversampling

Pressure and temperature OSR can be set independently. Temperature changes slowly, so a low temperature OSR saves conversion time without hurting accuracy:

```cpp
// High resolution pressure, fast temperature
sensor.begin(MS5849_30BA::ADDRESS_GND, MS5849_30BA::OSR_6, MS5849_30BA::OSR_1);

// Or change later:
sensor.setOversampling(MS5849_30BA::OSR_5, MS5849_30BA::OSR_1);
```

## Hardware IIR filter

The MS5849-30BA has a built-in digital IIR low-pass filter that smooths ADC noise at the hardware level. Enable or disable it and set the strength:

```cpp
// Enable with strength 2 (FILTER_1 .. FILTER_7, higher = smoother)
sensor.setFilter(true, MS5849_30BA::FILTER_2);

// Or set directly
sensor.setFilter(MS5849_30BA::FILTER_3);

// Disable
sensor.setFilter(false);

// Check current setting
MS5849_30BA::Filter f = sensor.pressureFilter();
```

## Software EMA filter

An optional exponential moving-average filter smooths pressure readings in software on top of any hardware filtering:

```cpp
sensor.setEmaAlpha(0.1);  // 0.0–1.0. Smaller = smoother. 1.0 = off (default).

// After read():
float smoothed = sensor.emaPressure();  // latest EMA value

sensor.resetEma();  // reset filter state
```

## Temperature skip

When polling rapidly, skip temperature reads to save time. The cached temperature is reused for pressure compensation between reads:

```cpp
sensor.setTemperatureSkip(10);  // read temp once every 10 calls
```

## Non-blocking (async) reads

For time-critical applications, the non-blocking API frees the MCU during ADC conversion:

```cpp
void loop() {
  static bool running = false;
  static uint32_t lastMs = 0;

  if (!running && (millis() - lastMs >= 50)) {
    sensor.startConversionAsync();
    running = true;
  }

  if (running) {
    float pressure, temperature;
    if (sensor.readAsync(pressure, temperature)) {
      running = false;
      lastMs = millis();
      Serial.println(pressure, 3);
    }
  }

  // MCU is free to do other work here
}
```

## Reading internal calibration

`begin()` automatically reads and caches the internal factory calibration PROM/NVRAM. You can print or copy those values like this:

```cpp
MS5849_30BA::CalibrationData cal;
sensor.getCalibration(cal);

Serial.println(cal.serialNumber, HEX);
Serial.println(cal.productId, HEX);
Serial.println(cal.storedCrc, HEX);
Serial.println(cal.calculatedCrc, HEX);
Serial.println(cal.crcOk ? "CRC OK" : "CRC mismatch");

for (uint8_t i = 1; i <= 10; i++) {
  Serial.print("C");
  Serial.print(i);
  Serial.print(" = ");
  Serial.println(cal.c[i]);
}

for (uint8_t addr = 0; addr < 16; addr++) {
  Serial.println(cal.rawProm[addr], HEX);
}
```

You can also read one calibration/PROM word directly from the device:

```cpp
uint16_t word;
if (sensor.readCalibrationWord(0x04, word)) {
  Serial.println(word, HEX); // C1
}
```

## Accuracy notes

### Double-precision compensation

The compensation math uses `double` internally. On 32-bit boards (ARM, ESP32, RP2040, Teensy) this gives true 64-bit precision (~15 significant digits), eliminating the rounding errors that 32-bit `float` causes with the large intermediate products.

On 8-bit AVR boards `double` is the same as `float` (32 bits). For maximum AVR accuracy, rewrite the `compensate()` method using `int64_t` fixed-point arithmetic — do all multiplications and divisions in integer, and convert to `float` only at the final output step.

### Hardware tips

- Decouple VDD with a 100 nF ceramic cap right at the sensor pins.
- Keep I2C traces short and away from noisy signals.
- If sub-mbar accuracy is critical, reduce read rate to minimise die self-heating.

## Notes

- Returns compensated pressure in mbar and temperature in degrees Celsius.
- Reads all 16 internal PROM/NVRAM words and exposes C1..C10, serial number, product ID, stored CRC, and calculated CRC.
- Uses MS5849-30BA commands and compensation constants from the public reference material for the Pressure 23 Click/MS5849-30BA.
- The library is I2C-only; SPI is not included.
