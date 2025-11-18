#include "dps5015.h"

// Modbus registers (DPS5015)
static const uint16_t REG_VOLTAGE_SET = 0x00;
static const uint16_t REG_CURRENT_SET = 0x01;
static const uint16_t REG_VOLTAGE_OUT = 0x02;
static const uint16_t REG_CURRENT_OUT = 0x03;
static const uint16_t REG_POWER_OUT = 0x04;
static const uint16_t REG_VOLTAGE_IN = 0x05;
static const uint16_t REG_MODE = 0x08;
static const uint16_t REG_OUTPUT = 0x09;

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
  uint16_t result = readRegister(REG_VOLTAGE_IN);
  _connected = (result != 0xFFFF);
  
  return _connected;
}

bool DPS5015::isConnected() {
  return _connected;
}

float DPS5015::readInputVoltage() {
  uint16_t value = readRegister(REG_VOLTAGE_IN);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readInputCurrent() {
  // Input current is typically at the next register after input voltage
  uint16_t value = readRegister(REG_VOLTAGE_IN + 1);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readOutputVoltage() {
  uint16_t value = readRegister(REG_VOLTAGE_OUT);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readOutputCurrent() {
  uint16_t value = readRegister(REG_CURRENT_OUT);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readOutputPower() {
  uint16_t value = readRegister(REG_POWER_OUT);
  return (value == 0xFFFF) ? 0.0 : value / 100.0;
}

float DPS5015::readSetVoltage() {
  _modbus->readHoldingRegisters(REG_VOLTAGE_SET, 1);
  if (_modbus->ku8MBSuccess == _modbus->getResponseBuffer(0)) {
    uint16_t value = _modbus->getResponseBuffer(0);
    return value / 100.0;
  }
  return 0.0;
}

float DPS5015::readSetCurrent() {
  _modbus->readHoldingRegisters(REG_CURRENT_SET, 1);
  if (_modbus->ku8MBSuccess == _modbus->getResponseBuffer(0)) {
    uint16_t value = _modbus->getResponseBuffer(0);
    return value / 100.0;
  }
  return 0.0;
}

bool DPS5015::setVoltage(float voltage) {
  uint16_t value = (uint16_t)(voltage * 100.0);
  return writeRegister(REG_VOLTAGE_SET, value);
}

bool DPS5015::setCurrent(float current) {
  uint16_t value = (uint16_t)(current * 100.0);
  return writeRegister(REG_CURRENT_SET, value);
}

bool DPS5015::setOutput(bool enabled) {
  return writeRegister(REG_OUTPUT, enabled ? 1 : 0);
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
