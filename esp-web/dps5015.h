#ifndef DPS5015_H
#define DPS5015_H

#include <Arduino.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

class DPS5015 {
public:
  DPS5015(uint8_t txPin, uint8_t rxPin, uint8_t address);
  
  bool begin();
  bool isConnected();
  
  // Read functions
  float readInputVoltage();
  float readInputCurrent();
  float readOutputVoltage();
  float readOutputCurrent();
  float readSetVoltage();
  float readSetCurrent();
  
  // Write functions
  bool setVoltage(float voltage);
  bool setCurrent(float current);
  bool setOutput(bool enabled);
  
private:
  SoftwareSerial* _serial;
  ModbusMaster* _modbus;
  uint8_t _address;
  bool _connected;
  
  uint16_t readRegister(uint16_t address);
  bool writeRegister(uint16_t address, uint16_t value);
};

#endif // DPS5015_H
