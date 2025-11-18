#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>

enum ChargerState {
  STATE_STANDBY,
  STATE_CHARGING,
  STATE_DISCHARGING,
  STATE_DISCONNECTED,
  STATE_ERROR
};

class Autopilot {
public:
  Autopilot();
  
  void begin();
  void update(float batteryVoltage, float batteryCurrent, float avgTemperature);
  
  ChargerState getState();
  const char* getStateString();
  
  float getChargeVoltage();
  float getChargeCurrent();
  float getStandbyVoltage();
  float getStandbyCurrent();
  
  void setChargeVoltage(float voltage);
  void setChargeCurrent(float current);
  void setStandbyVoltage(float voltage);
  void setStandbyCurrent(float current);
  
  bool isManualMode();
  
private:
  ChargerState _state;
  
  // Base voltages (can be overridden manually)
  float _chargeVoltageBase;
  float _standbyVoltageBase;
  float _chargeCurrent;
  float _standbyCurrent;
  
  // Temperature compensation
  float _lastTemperature;
  float _compensatedChargeVoltage;
  float _compensatedStandbyVoltage;
  
  // State transition timers
  bool _lowCurrentDetected;
  unsigned long _lowCurrentStartTime;
  bool _disconnectedDetected;
  unsigned long _disconnectedStartTime;
  
  // Manual mode
  bool _manualMode;
  
  void updateTemperatureCompensation(float temperature);
  float calculateCompensatedVoltage(float baseVoltage, float temperature);
  void checkStateTransitions(float voltage, float current);
};

#endif // AUTOPILOT_H
