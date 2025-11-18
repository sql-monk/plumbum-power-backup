#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPassword"
#define STATIC_IP IPAddress(192, 168, 50, 82)
#define GATEWAY IPAddress(192, 168, 50, 1)
#define SUBNET IPAddress(255, 255, 255, 0)

// Pin Definitions
#define DPS_TX_PIN D7        // GPIO13
#define DPS_RX_PIN D8        // GPIO15
#define DS18B20_PIN D6       // GPIO12
#define INA219_SDA D2        // GPIO4 (I2C)
#define INA219_SCL D1        // GPIO5 (I2C)
#define ACS711_PIN A0        // Analog input
#define GPIO_D3 D3           // GPIO0
#define GPIO_D4 D4           // GPIO2
#define GPIO_D5 D5           // GPIO14

// DPS5015 Configuration
#define DPS_MODBUS_ADDRESS 1
#define DPS_BAUD_RATE 9600

// INA219 Configuration
#define INA219_ADDRESS 0x40

// Charging Parameters (24V system)
#define BULK_VOLTAGE_BASE 28.4     // Base bulk charging voltage
#define STANDBY_VOLTAGE_BASE 27.2  // Base standby voltage
#define LOW_VOLTAGE_THRESHOLD 25.0 // Threshold for charging activation
#define ABSORPTION_CURRENT 0.2     // Current threshold for absorption phase
#define MAX_VOLTAGE 29.0           // Maximum allowed voltage
#define MAX_CURRENT 7.0            // Maximum charging current
#define TEMP_COEFFICIENT -0.036    // Voltage/°C for 24V system
#define REFERENCE_TEMP 25.0        // Reference temperature in °C

// Current Measurement
#define ACS711_SENSITIVITY 0.185   // V/A
#define ACS711_ZERO_OFFSET 2.5     // Zero current voltage
#define CURRENT_DEADBAND 0.05      // Deadband for noise filtering

// Temperature Validation
#define TEMP_MIN 0.0
#define TEMP_MAX 50.0
#define TEMP_CHANGE_THRESHOLD 1.0  // Recalculate compensation if temp changes > 1°C

// Timing
#define UPDATE_INTERVAL 500        // Sensor reading interval (ms)
#define CONFIRMATION_TIME 15000    // State transition confirmation time (ms)
#define DISCONNECTED_TIME 15000    // Time to detect disconnection (ms)

// NTP Configuration
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.nist.gov"
#define NTP_SERVER3 "ua.pool.ntp.org"
#define NTP_TIMEZONE_OFFSET 0      // UTC offset in seconds

#endif // CONFIG_H
