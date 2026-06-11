#include "MS5849_30BA.h"

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MS5849_30BA::MS5849_30BA(TwoWire &wire)
  : _wire(&wire), _address(ADDRESS_GND),
    _osrPressure(OSR_4), _osrTemperature(OSR_4), _pressureFilter(FILTER_OFF),
    _serialNumber(0), _productId(0), _promCrc(0), _calculatedPromCrc(0),
    _calibrationCrcOk(false), _prom{0}, _c{0},
    _asyncState(ASYNC_IDLE), _asyncStartUs(0),
    _asyncRawPressure(0), _asyncRawTemperature(0),
    _emaAlpha(1.0f), _emaPressure(0.0f), _emaInitialised(false),
    _tempSkip(1), _tempSkipCount(0), _cachedRawTemp(0), _cachedRawTempValid(false) {}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

bool MS5849_30BA::begin(uint8_t address, Oversampling osrPressure, Oversampling osrTemperature) {
  _address = address;
  _osrPressure = osrPressure;
  _osrTemperature = osrTemperature;

  if (!reset()) return false;
  delay(10);

  if (!readCalibration()) return false;
  if (!setOversampling(osrPressure, osrTemperature)) return false;

  return true;
}

bool MS5849_30BA::reset() {
  _asyncState = ASYNC_IDLE;
  resetEma();
  _tempSkipCount = 0;
  _cachedRawTempValid = false;
  return writeCommand(CMD_RESET);
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

bool MS5849_30BA::readCalibration() {
  for (uint8_t reg = 0; reg <= REG_PROM_ID_CRC; reg++) {
    if (!readMemoryWord(reg, _prom[reg])) return false;
  }

  _serialNumber = (static_cast<uint32_t>(_prom[REG_SERIAL_NUMBER_MSB]) << 16) |
                  static_cast<uint32_t>(_prom[REG_SERIAL_NUMBER_LSB]);

  for (uint8_t i = 1; i <= 10; i++) {
    _c[i] = _prom[REG_COEF_C1 + i - 1];
  }

  uint16_t idCrcWord = _prom[REG_PROM_ID_CRC];
  _productId = static_cast<uint8_t>(idCrcWord >> 8);
  _promCrc = static_cast<uint8_t>(idCrcWord & 0xFF);

  _calculatedPromCrc = calculatePromCrc8();
  _calibrationCrcOk = (_calculatedPromCrc == _promCrc);

  return true;
}

// ---------------------------------------------------------------------------
// Oversampling configuration
// ---------------------------------------------------------------------------

bool MS5849_30BA::setOversampling(Oversampling osr) {
  return setOversampling(osr, osr);
}

bool MS5849_30BA::setOversampling(Oversampling osrPressure, Oversampling osrTemperature) {
  _osrPressure = osrPressure;
  _osrTemperature = osrTemperature;

  if (!configureSensor(SENSOR_PRESSURE, osrPressure, 0, static_cast<uint8_t>(_pressureFilter))) return false;
  if (!configureSensor(SENSOR_TEMPERATURE, osrTemperature)) return false;

  return true;
}

// ---------------------------------------------------------------------------
// IIR filter configuration
// ---------------------------------------------------------------------------

bool MS5849_30BA::setFilter(Filter filter) {
  _pressureFilter = filter;
  return configureSensor(SENSOR_PRESSURE, _osrPressure, 0, static_cast<uint8_t>(filter));
}

bool MS5849_30BA::setFilter(bool enabled, Filter strength) {
  return setFilter(enabled ? strength : FILTER_OFF);
}

// ---------------------------------------------------------------------------
// Low-level sensor configuration
// ---------------------------------------------------------------------------

bool MS5849_30BA::configureSensor(uint8_t sensorSelect, Oversampling osr,
                                   uint8_t ratio, uint8_t filter, uint8_t resolution) {
  sensorSelect &= 0x01;
  ratio &= 0x07;
  filter &= 0x07;
  resolution &= 0x03;
  osr = static_cast<Oversampling>(static_cast<uint8_t>(osr) & 0x07);

  uint8_t data[2];
  data[0] = ratio;
  data[1] = static_cast<uint8_t>((filter << 5) | (resolution << 3) | osr);

  bool ok = writeCommandData(CMD_WRITE_CONFIG_PRESS | (sensorSelect << 1), data, 2);
  if (ok) {
    if (sensorSelect == SENSOR_PRESSURE) {
      _osrPressure = osr;
      _pressureFilter = static_cast<Filter>(filter);
    } else {
      _osrTemperature = osr;
    }
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Blocking reads
// ---------------------------------------------------------------------------

bool MS5849_30BA::read(float &pressureMbar, float &temperatureC) {
  uint32_t d1 = 0;
  uint32_t d2 = 0;

  if (!readRawPressure(d1)) return false;

  // Temperature skip: reuse the cached raw temperature value when possible.
  bool needTemp = !_cachedRawTempValid || (++_tempSkipCount >= _tempSkip);
  if (needTemp) {
    if (!readRawTemperature(d2)) return false;
    _cachedRawTemp = d2;
    _cachedRawTempValid = true;
    _tempSkipCount = 0;
  } else {
    d2 = _cachedRawTemp;
  }

  compensate(d1, d2, pressureMbar, temperatureC);
  applyEma(pressureMbar);

  return true;
}

float MS5849_30BA::readPressureMbar() {
  float pressure = NAN;
  float temperature = NAN;
  if (!read(pressure, temperature)) return NAN;
  return pressure;
}

float MS5849_30BA::readTemperatureC() {
  float pressure = NAN;
  float temperature = NAN;
  if (!read(pressure, temperature)) return NAN;
  return temperature;
}

bool MS5849_30BA::readRawPressure(uint32_t &rawPressure) {
  if (!startConversion(SENSOR_PRESSURE)) return false;
  delayMicroseconds(conversionDelayUs(_osrPressure));
  return readAdc(SENSOR_PRESSURE, rawPressure);
}

bool MS5849_30BA::readRawTemperature(uint32_t &rawTemperature) {
  if (!startConversion(SENSOR_TEMPERATURE)) return false;
  delayMicroseconds(conversionDelayUs(_osrTemperature));
  return readAdc(SENSOR_TEMPERATURE, rawTemperature);
}

// ---------------------------------------------------------------------------
// Non-blocking (async) reads
// ---------------------------------------------------------------------------

bool MS5849_30BA::startConversionAsync() {
  if (!startConversion(SENSOR_PRESSURE)) return false;
  _asyncStartUs = micros();
  _asyncState = ASYNC_PRESS_STARTED;
  return true;
}

bool MS5849_30BA::isConversionComplete() const {
  if (_asyncState == ASYNC_IDLE) return false;
  if (_asyncState == ASYNC_DONE) return true;

  uint32_t elapsed = micros() - _asyncStartUs;

  if (_asyncState == ASYNC_PRESS_STARTED) {
    return elapsed >= conversionDelayUs(_osrPressure);
  }
  if (_asyncState == ASYNC_TEMP_STARTED) {
    return elapsed >= conversionDelayUs(_osrTemperature);
  }
  return false;
}

uint32_t MS5849_30BA::conversionTimeUs() const {
  uint32_t t = conversionDelayUs(_osrPressure);
  bool needTemp = !_cachedRawTempValid || (_tempSkipCount + 1 >= _tempSkip);
  if (needTemp) t += conversionDelayUs(_osrTemperature);
  return t;
}

bool MS5849_30BA::readAsync(float &pressureMbar, float &temperatureC) {
  // State machine: advance through each phase.
  // This can be called repeatedly until it returns true.

  if (_asyncState == ASYNC_IDLE) return false;

  // Phase 1: wait for pressure conversion, then read pressure ADC.
  if (_asyncState == ASYNC_PRESS_STARTED) {
    if ((micros() - _asyncStartUs) < conversionDelayUs(_osrPressure)) return false;
    if (!readAdc(SENSOR_PRESSURE, _asyncRawPressure)) { _asyncState = ASYNC_IDLE; return false; }
    _asyncState = ASYNC_PRESS_DONE;
  }

  // Phase 2: decide if we need a fresh temperature reading.
  if (_asyncState == ASYNC_PRESS_DONE) {
    bool needTemp = !_cachedRawTempValid || (++_tempSkipCount >= _tempSkip);
    if (needTemp) {
      if (!startConversion(SENSOR_TEMPERATURE)) { _asyncState = ASYNC_IDLE; return false; }
      _asyncStartUs = micros();
      _asyncState = ASYNC_TEMP_STARTED;
    } else {
      _asyncRawTemperature = _cachedRawTemp;
      _asyncState = ASYNC_DONE;
    }
  }

  // Phase 3: wait for temperature conversion, then read temperature ADC.
  if (_asyncState == ASYNC_TEMP_STARTED) {
    if ((micros() - _asyncStartUs) < conversionDelayUs(_osrTemperature)) return false;
    if (!readAdc(SENSOR_TEMPERATURE, _asyncRawTemperature)) { _asyncState = ASYNC_IDLE; return false; }
    _cachedRawTemp = _asyncRawTemperature;
    _cachedRawTempValid = true;
    _tempSkipCount = 0;
    _asyncState = ASYNC_DONE;
  }

  // Phase 4: compensate and return.
  if (_asyncState == ASYNC_DONE) {
    compensate(_asyncRawPressure, _asyncRawTemperature, pressureMbar, temperatureC);
    applyEma(pressureMbar);
    _asyncState = ASYNC_IDLE;
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// Software EMA filter
// ---------------------------------------------------------------------------

void MS5849_30BA::setEmaAlpha(float alpha) {
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  _emaAlpha = alpha;
}

void MS5849_30BA::resetEma() {
  _emaInitialised = false;
  _emaPressure = 0.0f;
}

void MS5849_30BA::applyEma(float &pressureMbar) {
  if (_emaAlpha >= 1.0f) {
    _emaPressure = pressureMbar;
    _emaInitialised = true;
    return;
  }
  if (!_emaInitialised) {
    _emaPressure = pressureMbar;
    _emaInitialised = true;
  } else {
    _emaPressure = _emaAlpha * pressureMbar + (1.0f - _emaAlpha) * _emaPressure;
  }
  pressureMbar = _emaPressure;
}

// ---------------------------------------------------------------------------
// Temperature skip
// ---------------------------------------------------------------------------

void MS5849_30BA::setTemperatureSkip(uint8_t n) {
  if (n < 1) n = 1;
  _tempSkip = n;
  _tempSkipCount = 0;
}

// ---------------------------------------------------------------------------
// Compensation (double precision for accuracy)
// ---------------------------------------------------------------------------

void MS5849_30BA::compensate(uint32_t d1, uint32_t d2,
                              float &pressureMbar, float &temperatureC) {
  // Use double for all intermediate math.  On ARM/ESP32/RP2040 this is true
  // 64-bit (≈15 significant digits).  On 8-bit AVR, double == float (32-bit)
  // so precision is unchanged there — see README for an int64_t alternative.

  // --- First-order temperature (C1..C3) ---
  double temp = (static_cast<double>(_c[1]) * static_cast<double>(d2)) / 536870912.0;
  temp -= (static_cast<double>(_c[3]) * static_cast<double>(d1)) / 34359738368.0;
  temp -= static_cast<double>(_c[2]) / 64.0;

  // --- Second-order temperature compensation (C4, C5, C10) ---
  double tempSq = temp * temp;
  double T2    = (static_cast<double>(_c[10]) * tempSq) / 1048576.0;
  double OFF2  = (static_cast<double>(_c[4])  * tempSq) / 1048576.0;
  double SENS2 = (static_cast<double>(_c[5])  * tempSq) / 1048576.0;

  double tempComp = temp;

  // --- Pressure compensation (first + second order) ---
  double off  = static_cast<double>(_c[6]) + (static_cast<double>(_c[7]) * temp) / 512.0 - OFF2;
  double sens = static_cast<double>(_c[8]) + (static_cast<double>(_c[9]) * temp) / 512.0 - SENS2;

  double press = (static_cast<double>(d1) * sens) / 4194304.0 - off;

  // Convert back to float for the output.
  pressureMbar  = static_cast<float>(press);
  temperatureC  = static_cast<float>(tempComp);
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

uint16_t MS5849_30BA::coefficient(uint8_t index) const {
  if (index < 1 || index > 10) return 0;
  return _c[index];
}

uint16_t MS5849_30BA::rawPromWord(uint8_t address) const {
  if (address > REG_PROM_ID_CRC) return 0;
  return _prom[address];
}

void MS5849_30BA::getCalibration(CalibrationData &calibration) const {
  for (uint8_t i = 0; i < 16; i++) calibration.rawProm[i] = _prom[i];
  for (uint8_t i = 0; i < 11; i++) calibration.c[i] = _c[i];
  calibration.serialNumber = _serialNumber;
  calibration.productId = _productId;
  calibration.storedCrc = _promCrc;
  calibration.calculatedCrc = _calculatedPromCrc;
  calibration.crcOk = _calibrationCrcOk;
}

bool MS5849_30BA::readCalibrationWord(uint8_t address, uint16_t &value) {
  return readMemoryWord(address, value);
}

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------

bool MS5849_30BA::writeCommand(uint8_t command) {
  _wire->beginTransmission(_address);
  _wire->write(command);
  return _wire->endTransmission() == 0;
}

bool MS5849_30BA::writeCommandData(uint8_t command, const uint8_t *data, size_t len) {
  _wire->beginTransmission(_address);
  _wire->write(command);
  for (size_t i = 0; i < len; i++) _wire->write(data[i]);
  return _wire->endTransmission() == 0;
}

bool MS5849_30BA::readBytes(uint8_t command, uint8_t *data, size_t len) {
  _wire->beginTransmission(_address);
  _wire->write(command);

  // Use a normal STOP condition before the read.
  // This is more reliable at the default 100 kHz I2C clock on ESP32.
  if (_wire->endTransmission(true) != 0) return false;

  // Give the sensor/bus a small settling gap before requesting bytes.
  delayMicroseconds(50);

  size_t received = _wire->requestFrom(static_cast<int>(_address), static_cast<int>(len));
  if (received != len) return false;

  for (size_t i = 0; i < len; i++) data[i] = _wire->read();
  return true;
}

bool MS5849_30BA::readMemoryWord(uint8_t reg, uint16_t &value) {
  if (reg > REG_PROM_ID_CRC) return false;

  uint8_t data[2] = {0, 0};
  uint8_t command = CMD_READ_MEMORY | (reg << 1);
  if (!readBytes(command, data, 2)) return false;

  value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return true;
}

bool MS5849_30BA::startConversion(uint8_t sensorSelect) {
  sensorSelect &= 0x01;
  return writeCommand(CMD_START_CONVERSION | (0x04 << sensorSelect));
}

bool MS5849_30BA::readAdc(uint8_t sensorSelect, uint32_t &value) {
  sensorSelect &= 0x01;
  uint8_t data[3] = {0, 0, 0};
  uint8_t command = CMD_READ_ADC_REG | (0x04 << sensorSelect);
  if (!readBytes(command, data, 3)) return false;

  value = (static_cast<uint32_t>(data[0]) << 16) |
          (static_cast<uint32_t>(data[1]) << 8) |
          static_cast<uint32_t>(data[2]);
  return true;
}

uint16_t MS5849_30BA::conversionDelayUs(Oversampling osr) const {
  switch (osr) {
    // Extra margin improves reliability at the default 100 kHz I2C clock.
    case OSR_0: return 600;
    case OSR_1: return 900;
    case OSR_2: return 1500;
    case OSR_3: return 2600;
    case OSR_4: return 5000;
    case OSR_5: return 9500;
    case OSR_6: return 18000;
    default: return 5000;
  }
}

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

uint8_t MS5849_30BA::calculatePromCrc8() const {
  uint8_t crc = 0;
  for (uint8_t word = 0; word < 16; word++) {
    uint8_t msb = static_cast<uint8_t>(_prom[word] >> 8);
    uint8_t lsb = static_cast<uint8_t>(_prom[word] & 0xFF);

    crc = crc8Update(msb, crc);
    if (word == REG_PROM_ID_CRC) lsb = 0;
    crc = crc8Update(lsb, crc);
  }
  return crc;
}

uint8_t MS5849_30BA::crc8Update(uint8_t data, uint8_t crc) {
  for (uint8_t bit = 0; bit < 8; bit++) {
    uint8_t dataMsb = data >> 7;
    uint8_t crcMsb = crc >> 7;
    data <<= 1;
    crc <<= 1;
    if (dataMsb != crcMsb) crc ^= 0x31;
  }
  return crc;
}
