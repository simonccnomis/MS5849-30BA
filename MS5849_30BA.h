#pragma once

#include <Arduino.h>
#include <Wire.h>

class MS5849_30BA {
public:
  enum Oversampling : uint8_t {
    OSR_0 = 0,   // ~0.33 ms
    OSR_1 = 1,   // ~0.59 ms
    OSR_2 = 2,   // ~1.10 ms
    OSR_3 = 3,   // ~2.12 ms
    OSR_4 = 4,   // ~4.16 ms  (good default)
    OSR_5 = 5,   // ~8.24 ms
    OSR_6 = 6    // ~16.5 ms  (highest resolution)
  };

  // Hardware IIR low-pass filter strength (bits 7:5 of config byte 1).
  // Higher values = heavier smoothing = less noise but slower step response.
  enum Filter : uint8_t {
    FILTER_OFF = 0,
    FILTER_1   = 1,
    FILTER_2   = 2,
    FILTER_3   = 3,
    FILTER_4   = 4,
    FILTER_5   = 5,
    FILTER_6   = 6,
    FILTER_7   = 7
  };

  struct CalibrationData {
    uint16_t rawProm[16];      // Raw 16 x 16-bit NVRAM/PROM words, addresses 0x00..0x0F
    uint32_t serialNumber;     // PROM words 0x02 and 0x03
    uint8_t productId;         // High byte of PROM word 0x0F
    uint8_t storedCrc;         // Low byte of PROM word 0x0F
    uint8_t calculatedCrc;     // CRC-8 calculated from rawProm with stored CRC byte cleared
    bool crcOk;                // true when calculatedCrc == storedCrc
    uint16_t c[11];            // Factory calibration coefficients C1..C10; c[0] unused
  };

  static constexpr uint8_t ADDRESS_GND = 0x77;
  static constexpr uint8_t ADDRESS_VCC = 0x76;

  explicit MS5849_30BA(TwoWire &wire = Wire);

  // Initialise sensor. Pressure and temperature OSR can be set independently.
  // If osrTemperature is not supplied it defaults to the same as osrPressure.
  bool begin(uint8_t address = ADDRESS_GND,
             Oversampling osrPressure = OSR_4,
             Oversampling osrTemperature = OSR_4);

  bool reset();

  // Reads and caches the device's internal factory calibration PROM/NVRAM.
  // This is called automatically by begin(), but can be called again any time.
  bool readCalibration();

  // Set both pressure and temperature OSR to the same value.
  bool setOversampling(Oversampling osr);
  // Set pressure and temperature OSR independently.
  // Tip: temperature changes slowly — a low temperature OSR saves time.
  bool setOversampling(Oversampling osrPressure, Oversampling osrTemperature);

  // Enable/disable the sensor's hardware IIR low-pass filter and set strength.
  // Applies to the pressure channel. Call after begin().
  bool setFilter(Filter filter);
  bool setFilter(bool enabled, Filter strength = FILTER_2);

  // Low-level sensor configuration (pressure or temperature channel).
  bool configureSensor(uint8_t sensorSelect, Oversampling osr,
                       uint8_t ratio = 0, uint8_t filter = 0,
                       uint8_t resolution = 0);

  // ---- Blocking reads ----
  // Reads both pressure (mbar) and temperature (°C) in one call.
  bool read(float &pressureMbar, float &temperatureC);
  float readPressureMbar();
  float readTemperatureC();

  bool readRawPressure(uint32_t &rawPressure);
  bool readRawTemperature(uint32_t &rawTemperature);

  // ---- Non-blocking (async) reads ----
  // Call startConversionAsync() to kick off both ADC conversions.
  // Poll isConversionComplete() or wait conversionTimeUs() microseconds.
  // Then call readAsync() to retrieve the compensated result.
  bool startConversionAsync();
  bool isConversionComplete() const;
  uint32_t conversionTimeUs() const;      // worst-case total for both channels
  bool readAsync(float &pressureMbar, float &temperatureC);

  // ---- Software EMA (exponential moving-average) filter ----
  // Smooths pressure readings in software on top of any hardware filtering.
  // alpha = 0.0..1.0. Smaller = smoother/slower. 1.0 = no filtering (default).
  void setEmaAlpha(float alpha);
  float emaAlpha() const { return _emaAlpha; }
  float emaPressure() const { return _emaPressure; }
  void resetEma();

  // ---- Temperature skip ----
  // Read temperature only once every N calls to read()/readAsync().
  // Reuses the cached temperature value for pressure compensation in between.
  // Default = 1 (every call). Set to e.g. 10 for fast pressure-only polling.
  void setTemperatureSkip(uint8_t n);
  uint8_t temperatureSkip() const { return _tempSkip; }

  // ---- Accessors ----
  uint8_t address() const { return _address; }
  uint32_t serialNumber() const { return _serialNumber; }
  uint8_t productId() const { return _productId; }
  uint8_t promCrc() const { return _promCrc; }
  uint8_t calculatedPromCrc() const { return _calculatedPromCrc; }
  bool calibrationCrcOk() const { return _calibrationCrcOk; }
  Oversampling pressureOsr() const { return _osrPressure; }
  Oversampling temperatureOsr() const { return _osrTemperature; }
  Filter pressureFilter() const { return _pressureFilter; }

  // Coefficient index is 1..10. Returns 0 for an invalid index.
  uint16_t coefficient(uint8_t index) const;

  // Raw PROM/NVRAM word address is 0x00..0x0F. Returns 0 for an invalid address.
  uint16_t rawPromWord(uint8_t address) const;

  // Copies all cached calibration values into a user supplied struct.
  void getCalibration(CalibrationData &calibration) const;

  // Reads one PROM/NVRAM word directly from the device over I2C.
  bool readCalibrationWord(uint8_t address, uint16_t &value);

private:
  TwoWire *_wire;
  uint8_t _address;
  Oversampling _osrPressure;
  Oversampling _osrTemperature;
  Filter _pressureFilter;

  uint32_t _serialNumber;
  uint8_t _productId;
  uint8_t _promCrc;
  uint8_t _calculatedPromCrc;
  bool _calibrationCrcOk;
  uint16_t _prom[16];
  uint16_t _c[11]; // c[1]..c[10] used

  // Async state
  enum AsyncState : uint8_t { ASYNC_IDLE, ASYNC_PRESS_STARTED, ASYNC_PRESS_DONE, ASYNC_TEMP_STARTED, ASYNC_DONE };
  AsyncState _asyncState;
  uint32_t _asyncStartUs;
  uint32_t _asyncRawPressure;
  uint32_t _asyncRawTemperature;

  // EMA state
  float _emaAlpha;
  float _emaPressure;
  bool _emaInitialised;

  // Temperature skip state
  uint8_t _tempSkip;
  uint8_t _tempSkipCount;
  uint32_t _cachedRawTemp;
  bool _cachedRawTempValid;

  static constexpr uint8_t CMD_RESET = 0x10;
  static constexpr uint8_t CMD_WRITE_CONFIG_PRESS = 0x20;
  static constexpr uint8_t CMD_READ_MEMORY = 0xE0;
  static constexpr uint8_t CMD_START_CONVERSION = 0x40;
  static constexpr uint8_t CMD_READ_ADC_REG = 0x50;

  static constexpr uint8_t REG_SERIAL_NUMBER_MSB = 0x02;
  static constexpr uint8_t REG_SERIAL_NUMBER_LSB = 0x03;
  static constexpr uint8_t REG_COEF_C1 = 0x04;
  static constexpr uint8_t REG_PROM_ID_CRC = 0x0F;

  static constexpr uint8_t SENSOR_PRESSURE = 0x00;
  static constexpr uint8_t SENSOR_TEMPERATURE = 0x01;

  void compensate(uint32_t d1, uint32_t d2, float &pressureMbar, float &temperatureC);
  void applyEma(float &pressureMbar);

  bool writeCommand(uint8_t command);
  bool writeCommandData(uint8_t command, const uint8_t *data, size_t len);
  bool readBytes(uint8_t command, uint8_t *data, size_t len);
  bool readMemoryWord(uint8_t reg, uint16_t &value);
  bool startConversion(uint8_t sensorSelect);
  bool readAdc(uint8_t sensorSelect, uint32_t &value);
  uint16_t conversionDelayUs(Oversampling osr) const;
  uint8_t calculatePromCrc8() const;
  static uint8_t crc8Update(uint8_t data, uint8_t crc);
};
