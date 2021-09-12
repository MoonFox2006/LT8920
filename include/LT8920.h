#pragma once

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif
#include <Arduino.h>
#include <SPI.h>

template<const int8_t CS_PIN, const int8_t RST_PIN = -1, SPIClass &_SPI = SPI>
class LT8920 {
public:
  enum datarate_t : uint8_t { DATARATE_1MBS = 0x01, DATARATE_250KBS = 0x04, DATARATE_125KBS = 0x08, DATARATE_62KBS = 0x10 };

  LT8920();

  bool begin();
  uint8_t getChannel();
  void setChannel(uint8_t channel);
  datarate_t getDataRate();
  void setDataRate(datarate_t rate);
  void setPowerControl(uint8_t power, uint8_t gain);
  void setSyncWord(const uint8_t *words, uint8_t length);
  void setSyncWord_P(const uint8_t *words, uint8_t length);
  bool getCRC();
  void setCRC(bool enable);
  bool getAutoAck();
  void setAutoAck(bool enable);
  uint8_t getRetry();
  void setRetry(uint8_t retry);
  bool getScramble();
  void setScramble(bool enable);
  uint8_t getScrambleData();
  void setScrambleData(uint8_t data);

  void listen();
  void flushRx();
  void flushTx();
  void flush();
  bool available();
  int16_t read(uint8_t *buffer, uint8_t size);
  bool send(const uint8_t *buffer, uint8_t size);
  bool send_P(const uint8_t *buffer, uint8_t size);

  uint8_t getRSSI();
  bool scanRSSI(uint8_t fromChannel, uint8_t channels, uint8_t *out);

  void sleep(bool on = true);
  void powerOff(bool off = true);

  bool waitPkt(uint32_t timeout = RF_TIMEOUT);

protected:
  static const uint32_t RF_TIMEOUT = 5; // 5 ms.

  uint8_t readRegister8(uint8_t reg);
  uint16_t readRegister(uint8_t reg);
  uint8_t readRegisters8(uint8_t reg, uint8_t *values, uint8_t count);
  uint8_t writeRegister(uint8_t addr, uint16_t value);
  uint8_t writeRegister8(uint8_t addr, uint8_t value);
  uint8_t modifyRegister(uint8_t addr, uint16_t andMask, uint16_t orMask);
  uint8_t writeRegisters(uint8_t addr, const uint16_t *values, uint8_t count);
  uint8_t writeRegisters8(uint8_t addr, const uint8_t *values, uint8_t count);
  uint8_t writeRegisters_P(uint8_t addr, const uint16_t *values, uint8_t count);
  uint8_t writeRegisters8_P(uint8_t addr, const uint8_t *values, uint8_t count);

  uint8_t _channel : 7;
  bool _autoAck : 1;
};

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
LT8920<CS_PIN, RST_PIN, _SPI>::LT8920() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  if (RST_PIN >= 0) {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
  }
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
bool LT8920<CS_PIN, RST_PIN, _SPI>::begin() {
  static const uint16_t REGVALUES[] PROGMEM = {
    0x6FE0, 0x5681, 0x6617, 0x9CC9, 0x6637, 0x0030, 0x6C90, 0x4780,
    0x7FFD, 0x0008, 0x0000, 0x48BD, 0x00FF, 0x8005, 0x0067, 0x1659,
    0x19E0, 0x1300, 0x1800, 0x5000, 0x3FC7, 0x2000, 0x0300, 0xDEAD,
    0x0000, 0xBEEF, 0xFEED, 0x2101, 0xB000, 0xFDB0, 0x000F, 0x0100,
    0x0080 };

  _SPI.begin();
  if (RST_PIN >= 0) {
    digitalWrite(RST_PIN, LOW);
    delay(10);
    digitalWrite(RST_PIN, HIGH);
  }
  delay(10);
  writeRegisters_P(0, REGVALUES, 3);
  writeRegisters_P(4, &REGVALUES[3], 2);
  writeRegisters_P(7, &REGVALUES[5], 7);
  writeRegisters_P(22, &REGVALUES[12], 7);
  writeRegisters_P(32, &REGVALUES[19], 14);
  flush();
  _channel = 0x30;
  _autoAck = false;
  return true;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::getChannel() {
  return _channel;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
void LT8920<CS_PIN, RST_PIN, _SPI>::setChannel(uint8_t channel) {
  _channel = channel;
  writeRegister(7, _channel);
//  delayMicroseconds(150);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline typename LT8920<CS_PIN, RST_PIN, _SPI>::datarate_t LT8920<CS_PIN, RST_PIN, _SPI>::getDataRate() {
  return (datarate_t)(readRegister(44) >> 8);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
void LT8920<CS_PIN, RST_PIN, _SPI>::setDataRate(datarate_t rate) {
  writeRegister(44, rate << 8);
  writeRegister(45, rate == DATARATE_1MBS ? 0x0080 : 0x0552);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::setPowerControl(uint8_t power, uint8_t gain) {
  writeRegister(9, ((power & 0x0F) << 12) | ((gain & 0x0F) << 7));
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
void LT8920<CS_PIN, RST_PIN, _SPI>::setSyncWord(const uint8_t *words, uint8_t length) {
  uint8_t len;

  if (length <= 2)
    len = 0;
  else if (length <= 4)
    len = 1;
  else if (length <= 6)
    len = 2;
  else
    len = 3;
  modifyRegister(32, 0xE7FF, len << 11);
  writeRegister(36, (*words++ << 8) | *words++);
  if (len == 3)
    writeRegister(37, (*words++ << 8) | *words++);
  if (len >= 2)
    writeRegister(38, (*words++ << 8) | *words++);
  if (len > 0)
    writeRegister(39, (*words++ << 8) | *words++);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
void LT8920<CS_PIN, RST_PIN, _SPI>::setSyncWord_P(const uint8_t *words, uint8_t length) {
  uint8_t len;

  if (length <= 2)
    len = 0;
  else if (length <= 4)
    len = 1;
  else if (length <= 6)
    len = 2;
  else
    len = 3;
  modifyRegister(32, 0xE7FF, len << 11);
  writeRegister(36, (pgm_read_byte(words++) << 8) | pgm_read_byte(words++));
  if (len == 3)
    writeRegister(37, (pgm_read_byte(words++) << 8) | pgm_read_byte(words++));
  if (len >= 2)
    writeRegister(38, (pgm_read_byte(words++) << 8) | pgm_read_byte(words++));
  if (len > 0)
    writeRegister(39, (pgm_read_byte(words++) << 8) | pgm_read_byte(words++));
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline bool LT8920<CS_PIN, RST_PIN, _SPI>::getCRC() {
  return (readRegister(41) >> 15) & 0x01;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::setCRC(bool on) {
  modifyRegister(41, 0x7FFF, on << 15);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline bool LT8920<CS_PIN, RST_PIN, _SPI>::getAutoAck() {
  return _autoAck;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
void LT8920<CS_PIN, RST_PIN, _SPI>::setAutoAck(bool on) {
  _autoAck = on;
  modifyRegister(41, 0xF7FF, _autoAck << 11);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::getRetry() {
  return (readRegister(35) >> 8) & 0x0F;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::setRetry(uint8_t retry) {
  modifyRegister(35, 0xF0FF, (retry & 0x0F) << 8);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline bool LT8920<CS_PIN, RST_PIN, _SPI>::getScramble() {
  return (readRegister(41) >> 14) & 0x01;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::setScramble(bool on) {
  modifyRegister(41, 0xBFFF, on << 14);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::getScrambleData() {
  return readRegister(35) & 0x7F;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::setScrambleData(uint8_t data) {
  modifyRegister(35, 0xFF80, data & 0x7F);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
void LT8920<CS_PIN, RST_PIN, _SPI>::listen() {
//  modifyRegister(7, 0xFE7F, 0); // Idle
  writeRegister(7, 0); // Idle
  delayMicroseconds(150);
  flushRx();
//  modifyRegister(7, 0xFE7F, 0x0080); // RX_EN
  writeRegister(7, 0x0080 | _channel); // RX_EN
  delayMicroseconds(150);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::flushRx() {
  writeRegister(52, 0x0080);
  delayMicroseconds(1);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::flushTx() {
  writeRegister(52, 0x8000);
  delayMicroseconds(1);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::flush() {
  writeRegister(52, 0x8080);
  delayMicroseconds(1);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline bool LT8920<CS_PIN, RST_PIN, _SPI>::available() {
  return (readRegister(48) & 0x40) != 0; // PKT_FLAG
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
int16_t LT8920<CS_PIN, RST_PIN, _SPI>::read(uint8_t *buffer, uint8_t size) {
  if (! (readRegister(48) & 0x8000)) { // No CRC error
    uint8_t len;

    len = readRegister8(50);
    if (len > size)
      return -2; // Not enough memory
    if (size > len)
      size = len;
    return readRegisters8(50, buffer, size);
  }
  return -1;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
bool LT8920<CS_PIN, RST_PIN, _SPI>::send(const uint8_t *buffer, uint8_t size) {
  if (size) {
//    modifyRegister(7, 0xFE7F, 0); // Idle
    writeRegister(7, 0); // Idle
    delayMicroseconds(150);
    if (_autoAck)
      flush();
    else
      flushTx();
    writeRegister8(50, size);
    writeRegisters8(50, buffer, size);
//    modifyRegister(7, 0xFE7F, 0x0100); // TX_EN
    writeRegister(7, 0x0100 | _channel); // TX_EN
//    delayMicroseconds(150);
    if (! waitPkt())
      return false;
    return (! _autoAck) || ((readRegister(52) & 0x003F) == 0);
  }
  return false;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
bool LT8920<CS_PIN, RST_PIN, _SPI>::send_P(const uint8_t *buffer, uint8_t size) {
  if (size) {
//    modifyRegister(7, 0xFE7F, 0); // Idle
    writeRegister(7, 0); // Idle
    delayMicroseconds(150);
    if (_autoAck)
      flush();
    else
      flushTx();
    writeRegister8(50, size);
    writeRegisters8_P(50, buffer, size);
//    modifyRegister(7, 0xFE7F, 0x0100); // TX_EN
    writeRegister(7, 0x0100 | _channel); // TX_EN
//    delayMicroseconds(150);
    if (! waitPkt())
      return false;
    return (! _autoAck) || ((readRegister(52) & 0x003F) == 0);
  }
  return false;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::getRSSI() {
  return readRegister(6) >> 10;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
bool LT8920<CS_PIN, RST_PIN, _SPI>::scanRSSI(uint8_t fromChannel, uint8_t channels, uint8_t *out) {
  flushRx();
  modifyRegister(42, 0x03FF, ((channels - 1) & 0x3F) << 10);
  modifyRegister(43, 0x00FF, 0x8000 | ((fromChannel & 0x7F) << 14));
  if (! waitPkt(100)) // 100 ms.
    return false;
  return readRegisters8(50, out, channels) == channels;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::sleep(bool on) {
  modifyRegister(35, 0xBFFF, on << 14);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
inline void LT8920<CS_PIN, RST_PIN, _SPI>::powerOff(bool off) {
  modifyRegister(35, 0x7FFF, off << 15);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
bool LT8920<CS_PIN, RST_PIN, _SPI>::waitPkt(uint32_t timeout) {
  do {
    if (readRegister(48) & 0x40)
      return true;
    if (timeout)
      delay(1);
  } while (timeout--);
  return false;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::readRegister8(uint8_t reg) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  _SPI.transfer(reg | 0x80);
  result = _SPI.transfer(0x00);
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint16_t LT8920<CS_PIN, RST_PIN, _SPI>::readRegister(uint8_t reg) {
  uint16_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  _SPI.transfer(reg | 0x80);
  result = _SPI.transfer(0x00) << 8;
  result |= _SPI.transfer(0x00);
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::readRegisters8(uint8_t reg, uint8_t *values, uint8_t count) {
  uint8_t result = 0;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  _SPI.transfer(reg | 0x80);
  while (count--) {
    *values++ = _SPI.transfer(0x00);
    ++result;
  }
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::writeRegister8(uint8_t reg, uint8_t value) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  result = _SPI.transfer(reg & 0x7F);
  _SPI.transfer(value);
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::writeRegister(uint8_t reg, uint16_t value) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  result = _SPI.transfer(reg & 0x7F);
  _SPI.transfer(value >> 8);
  _SPI.transfer(value);
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::modifyRegister(uint8_t reg, uint16_t andMask, uint16_t orMask) {
  uint16_t value;

  value = readRegister(reg);
  value &= andMask;
  value |= orMask;
  return writeRegister(reg, value);
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::writeRegisters(uint8_t reg, const uint16_t *values, uint8_t count) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  result = _SPI.transfer(reg & 0x7F);
  while (count--) {
    _SPI.transfer(*values >> 8);
    _SPI.transfer(*values);
    ++values;
  }
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::writeRegisters8(uint8_t reg, const uint8_t *values, uint8_t count) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  result = _SPI.transfer(reg & 0x7F);
  while (count--) {
    _SPI.transfer(*values++);
  }
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::writeRegisters_P(uint8_t reg, const uint16_t *values, uint8_t count) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  result = _SPI.transfer(reg & 0x7F);
  while (count--) {
    _SPI.transfer(pgm_read_word(values) >> 8);
    _SPI.transfer(pgm_read_word(values));
    ++values;
  }
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}

template<const int8_t CS_PIN, const int8_t RST_PIN, SPIClass &_SPI>
uint8_t LT8920<CS_PIN, RST_PIN, _SPI>::writeRegisters8_P(uint8_t reg, const uint8_t *values, uint8_t count) {
  uint8_t result;

  digitalWrite(CS_PIN, LOW);
  _SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  result = _SPI.transfer(reg & 0x7F);
  while (count--) {
    _SPI.transfer(pgm_read_word(values++));
  }
  _SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  return result;
}
