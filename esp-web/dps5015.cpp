#include "dps5015.h"

// DPS5015 Modbus Register Addresses
#define REG_INPUT_VOLTAGE 0x0000   // Input voltage (V * 100)
#define REG_INPUT_CURRENT 0x0001   // Input current (A * 100)
#define REG_OUTPUT_VOLTAGE 0x0002  // Output voltage (V * 100)
#define REG_OUTPUT_CURRENT 0x0003  // Output current (A * 100)
#define REG_SET_VOLTAGE 0x0000     // Set voltage (V * 100) - holding register
#define REG_SET_CURRENT 0x0001     // Set current (A * 100) - holding register
#define REG_OUTPUT_ENABLE 0x0009   // Output enable (0=OFF, 1=ON) - holding register

DPS5015::DPS5015(uint8_t txPin, uint8_t rxPin, uint8_t address) {
  _serial = new SoftwareSerial(rxPin, txPin);
  _modbus = new ModbusMaster();
  _address = address;
  _connected = false;
}

bool DPS5015::begin() {
  _serial->begin(9600);
  _modbus->begin(_address, *_serial);
  
  // Test connection by reading a register
  delay(100);
  uint16_t result = readRegister(REG_INPUT_VOLTAGE);
  _connected = (result != 0xFFFF);
  
  return _connected;
}

bool DPS5015::isConnected() {
  return _connected;
}

float DPS5015::readInputVoltage() {
  uint16_t value = readRegister(REG_INPUT_VOLTAGE);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readInputCurrent() {
  uint16_t value = readRegister(REG_INPUT_CURRENT);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readOutputVoltage() {
  uint16_t value = readRegister(REG_OUTPUT_VOLTAGE);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readOutputCurrent() {
  uint16_t value = readRegister(REG_OUTPUT_CURRENT);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readSetVoltage() {
  _modbus->readHoldingRegisters(REG_SET_VOLTAGE, 1);
  if (_modbus->ku8MBSuccess == _modbus->getResponseBuffer(0)) {
    uint16_t value = _modbus->getResponseBuffer(0);
    return value / 100.0;
  }
  return 0.0;
}

float DPS5015::readSetCurrent() {
  _modbus->readHoldingRegisters(REG_SET_CURRENT, 1);
  if (_modbus->ku8MBSuccess == _modbus->getResponseBuffer(0)) {
    uint16_t value = _modbus->getResponseBuffer(0);
    return value / 100.0;
  }
  return 0.0;
}

bool DPS5015::setVoltage(float voltage) {
  uint16_t value = (uint16_t)(voltage * 100.0);
  return writeRegister(REG_SET_VOLTAGE, value);
}

bool DPS5015::setCurrent(float current) {
  uint16_t value = (uint16_t)(current * 100.0);
  return writeRegister(REG_SET_CURRENT, value);
}

bool DPS5015::setOutput(bool enabled) {
  return writeRegister(REG_OUTPUT_ENABLE, enabled ? 1 : 0);
}

uint16_t DPS5015::readRegister(uint16_t address) {
  uint8_t result = _modbus->readInputRegisters(address, 1);
  if (result == _modbus->ku8MBSuccess) {
    return _modbus->getResponseBuffer(0);
  }
  _connected = false;
  return 0xFFFF;
}

bool DPS5015::writeRegister(uint16_t address, uint16_t value) {
  uint8_t result = _modbus->writeSingleRegister(address, value);
  if (result == _modbus->ku8MBSuccess) {
    return true;
  }
  _connected = false;
  return false;
}
