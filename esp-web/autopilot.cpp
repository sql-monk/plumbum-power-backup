#include "autopilot.h"
#include "config.h"

Autopilot::Autopilot() {
  _state = STATE_DISCONNECTED;
  _chargeVoltageBase = BULK_VOLTAGE_BASE;
  _standbyVoltageBase = STANDBY_VOLTAGE_BASE;
  _chargeCurrent = MAX_CURRENT;
  _standbyCurrent = MAX_CURRENT;
  _lastTemperature = REFERENCE_TEMP;
  _compensatedChargeVoltage = BULK_VOLTAGE_BASE;
  _compensatedStandbyVoltage = STANDBY_VOLTAGE_BASE;
  _lowCurrentDetected = false;
  _lowCurrentStartTime = 0;
  _disconnectedDetected = false;
  _disconnectedStartTime = 0;
  _manualMode = false;
}

void Autopilot::begin() {
  updateTemperatureCompensation(REFERENCE_TEMP);
}

void Autopilot::update(float batteryVoltage, float batteryCurrent, float avgTemperature) {
  // Update temperature compensation if needed
  updateTemperatureCompensation(avgTemperature);
  
  // Check state transitions
  checkStateTransitions(batteryVoltage, batteryCurrent);
}

ChargerState Autopilot::getState() {
  return _state;
}

const char* Autopilot::getStateString() {
  switch (_state) {
    case STATE_STANDBY: return "STANDBY";
    case STATE_CHARGING: return "CHARGING";
    case STATE_DISCHARGING: return "DISCHARGING";
    case STATE_DISCONNECTED: return "DISCONNECTED";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

float Autopilot::getChargeVoltage() {
  return _compensatedChargeVoltage;
}

float Autopilot::getChargeCurrent() {
  return _chargeCurrent;
}

float Autopilot::getStandbyVoltage() {
  return _compensatedStandbyVoltage;
}

float Autopilot::getStandbyCurrent() {
  return _standbyCurrent;
}

void Autopilot::setChargeVoltage(float voltage) {
  _chargeVoltageBase = voltage;
  _manualMode = true;
  updateTemperatureCompensation(_lastTemperature);
}

void Autopilot::setChargeCurrent(float current) {
  _chargeCurrent = current;
  _manualMode = true;
}

void Autopilot::setStandbyVoltage(float voltage) {
  _standbyVoltageBase = voltage;
  updateTemperatureCompensation(_lastTemperature);
}

void Autopilot::setStandbyCurrent(float current) {
  _standbyCurrent = current;
}

bool Autopilot::isManualMode() {
  return _manualMode;
}

void Autopilot::updateTemperatureCompensation(float temperature) {
  // Only recalculate if temperature changed significantly
  if (abs(temperature - _lastTemperature) < TEMP_CHANGE_THRESHOLD) {
    return;
  }
  
  _lastTemperature = temperature;
  _compensatedChargeVoltage = calculateCompensatedVoltage(_chargeVoltageBase, temperature);
  _compensatedStandbyVoltage = calculateCompensatedVoltage(_standbyVoltageBase, temperature);
}

float Autopilot::calculateCompensatedVoltage(float baseVoltage, float temperature) {
  float compensated = baseVoltage + (temperature - REFERENCE_TEMP) * TEMP_COEFFICIENT;
  
  // Limit to MAX_VOLTAGE
  if (compensated > MAX_VOLTAGE) {
    compensated = MAX_VOLTAGE;
  }
  
  return compensated;
}

void Autopilot::checkStateTransitions(float voltage, float current) {
  // Check for DISCONNECTED state
  if (voltage < 3.0 || abs(current) < CURRENT_DEADBAND) {
    if (!_disconnectedDetected) {
      _disconnectedDetected = true;
      _disconnectedStartTime = millis();
    } else if (millis() - _disconnectedStartTime >= DISCONNECTED_TIME) {
      if (_state != STATE_DISCONNECTED) {
        Serial.println("State: DISCONNECTED");
        _state = STATE_DISCONNECTED;
        _lowCurrentDetected = false;
      }
      return;
    }
  } else {
    _disconnectedDetected = false;
  }
  
  // Check for DISCHARGING state
  if (current < -0.02) {
    if (_state != STATE_DISCHARGING) {
      Serial.println("State: DISCHARGING");
      _state = STATE_DISCHARGING;
    }
    _lowCurrentDetected = false;
    return;
  }
  
  // Check for low current condition (for CHARGING -> STANDBY transition)
  if (current < ABSORPTION_CURRENT) {
    if (!_lowCurrentDetected) {
      _lowCurrentDetected = true;
      _lowCurrentStartTime = millis();
    }
  } else {
    _lowCurrentDetected = false;
  }
  
  // State transitions based on voltage and current
  switch (_state) {
    case STATE_DISCONNECTED:
      // Transition to CHARGING or STANDBY when battery is connected
      if (voltage >= 3.0) {
        if (voltage < LOW_VOLTAGE_THRESHOLD || current >= ABSORPTION_CURRENT) {
          Serial.println("State: CHARGING");
          _state = STATE_CHARGING;
        } else {
          Serial.println("State: STANDBY");
          _state = STATE_STANDBY;
        }
      }
      break;
      
    case STATE_CHARGING:
      // Transition to STANDBY when current drops below threshold for confirmation time
      if (_lowCurrentDetected && (millis() - _lowCurrentStartTime >= CONFIRMATION_TIME)) {
        if (voltage >= LOW_VOLTAGE_THRESHOLD) {
          Serial.println("State: CHARGING -> STANDBY");
          _state = STATE_STANDBY;
          _lowCurrentDetected = false;
        }
      }
      break;
      
    case STATE_STANDBY:
      // Transition to CHARGING if voltage drops or current increases
      if (voltage < LOW_VOLTAGE_THRESHOLD || current >= ABSORPTION_CURRENT) {
        Serial.println("State: STANDBY -> CHARGING");
        _state = STATE_CHARGING;
        _lowCurrentDetected = false;
      }
      break;
      
    case STATE_DISCHARGING:
      // Already handled above, will exit discharging when current becomes positive
      if (current >= -0.02) {
        if (voltage < LOW_VOLTAGE_THRESHOLD || current >= ABSORPTION_CURRENT) {
          Serial.println("State: DISCHARGING -> CHARGING");
          _state = STATE_CHARGING;
        } else {
          Serial.println("State: DISCHARGING -> STANDBY");
          _state = STATE_STANDBY;
        }
        _lowCurrentDetected = false;
      }
      break;
      
    case STATE_ERROR:
      // Can recover from error state
      if (voltage >= 3.0) {
        Serial.println("State: ERROR -> STANDBY");
        _state = STATE_STANDBY;
      }
      break;
  }
}
