// MS5849-30BA full-feature example
// Demonstrates: independent OSR, hardware IIR filter, software EMA,
// temperature skip, and non-blocking (async) reads.

#include <Wire.h>
#include <MS5849_30BA.h>

MS5849_30BA sensor;

void printHex16(uint16_t value) {
  if (value < 0x1000) Serial.print('0');
  if (value < 0x0100) Serial.print('0');
  if (value < 0x0010) Serial.print('0');
  Serial.print(value, HEX);
}

void printCalibration() {
  MS5849_30BA::CalibrationData cal;
  sensor.getCalibration(cal);

  Serial.println();
  Serial.println("Internal factory calibration PROM/NVRAM:");

  Serial.print("Serial number: 0x");
  Serial.println(cal.serialNumber, HEX);

  Serial.print("Product ID: 0x");
  Serial.println(cal.productId, HEX);

  Serial.print("Stored CRC: 0x");
  Serial.println(cal.storedCrc, HEX);

  Serial.print("Calculated CRC: 0x");
  Serial.print(cal.calculatedCrc, HEX);
  Serial.print("  ");
  Serial.println(cal.crcOk ? "OK" : "MISMATCH");

  Serial.println("Raw PROM words:");
  for (uint8_t addr = 0; addr < 16; addr++) {
    Serial.print("  [0x");
    Serial.print(addr, HEX);
    Serial.print("] = 0x");
    printHex16(cal.rawProm[addr]);
    Serial.println();
  }

  Serial.println("Factory coefficients:");
  for (uint8_t i = 1; i <= 10; i++) {
    Serial.print("  C");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(cal.c[i]);
    Serial.print(" (0x");
    printHex16(cal.c[i]);
    Serial.println(")");
  }
  Serial.println();
}

// ---- Choose one demo mode by uncommenting ----
#define DEMO_BLOCKING       // Simple blocking reads with all improvements
// #define DEMO_ASYNC       // Non-blocking reads (MCU is free during conversion)

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Use 400 kHz I2C for faster bus transactions. The MS5849-30BA supports it.
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("MS5849-30BA I2C example (improved)");

  // Independent OSR: high resolution for pressure, low for temperature (saves time).
  if (!sensor.begin(MS5849_30BA::ADDRESS_GND, MS5849_30BA::OSR_6, MS5849_30BA::OSR_1)) {
    Serial.println("Sensor not found. Check wiring, address, and 3.3 V power.");
    while (1) { delay(1000); }
  }

  // Enable the sensor's hardware IIR low-pass filter (strength 2).
  // Use FILTER_OFF to disable, or FILTER_1..FILTER_7 for increasing smoothing.
  sensor.setFilter(true, MS5849_30BA::FILTER_2);
  // Or equivalently: sensor.setFilter(MS5849_30BA::FILTER_2);
  // To disable:       sensor.setFilter(false);

  // Software exponential moving-average filter.
  // alpha 0.1 = heavy smoothing; 1.0 = no filtering (default).
  sensor.setEmaAlpha(0.1);

  // Read temperature only once every 10 pressure reads.
  // Cached value is reused in between — fine for slowly-changing environments.
  sensor.setTemperatureSkip(10);

  printCalibration();

  Serial.print("Pressure OSR: ");
  Serial.println(sensor.pressureOsr());
  Serial.print("Temperature OSR: ");
  Serial.println(sensor.temperatureOsr());
  Serial.print("IIR filter: ");
  Serial.println(sensor.pressureFilter());
  Serial.print("EMA alpha: ");
  Serial.println(sensor.emaAlpha(), 2);
  Serial.print("Temperature skip: ");
  Serial.println(sensor.temperatureSkip());
  Serial.println();
}

// ---------------------------------------------------------------------------
#ifdef DEMO_BLOCKING
// ---------------------------------------------------------------------------
void loop() {
  float pressureMbar;
  float temperatureC;

  if (sensor.read(pressureMbar, temperatureC)) {
    Serial.print("Pressure: ");
    Serial.print(pressureMbar, 3);
    Serial.print(" mbar  (EMA: ");
    Serial.print(sensor.emaPressure(), 3);
    Serial.print(" mbar)\tTemperature: ");
    Serial.print(temperatureC, 2);
    Serial.println(" C");
  } else {
    Serial.println("Read failed");
  }

  delay(100);
}
#endif

// ---------------------------------------------------------------------------
#ifdef DEMO_ASYNC
// ---------------------------------------------------------------------------
// Non-blocking demo.  The MCU can do other work while the ADC converts.
void loop() {
  static bool conversionRunning = false;
  static uint32_t lastReadMs = 0;

  // Start a new conversion every 100 ms.
  if (!conversionRunning && (millis() - lastReadMs >= 100)) {
    if (sensor.startConversionAsync()) {
      conversionRunning = true;
    }
  }

  // Poll — returns true only when the full result is ready.
  if (conversionRunning) {
    float pressureMbar, temperatureC;
    if (sensor.readAsync(pressureMbar, temperatureC)) {
      conversionRunning = false;
      lastReadMs = millis();

      Serial.print("Pressure: ");
      Serial.print(pressureMbar, 3);
      Serial.print(" mbar\tTemperature: ");
      Serial.print(temperatureC, 2);
      Serial.println(" C");
    }
  }

  // ... do other work here while waiting ...
}
#endif
