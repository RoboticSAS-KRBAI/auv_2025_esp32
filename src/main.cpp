/*
 * Multi-Sensor ESP32-S3 - Fixed HIDS Reading (2s Interval)
 * SDA: GPIO 18 | SCL: GPIO 17
 * - WSEN-PADS (Pressure) - 0x5D
 * - WSEN-HIDS (Temp/Humidity) - 0x44 [FIXED]
 * - INA226 #1 (Power) - 0x45
 * - INA226 #2 (Power) - 0x41
 */

#include <Wire.h>
#include <INA226.h>

// Pin Configuration
#define SDA_PIN 18
#define SCL_PIN 17

// WSEN-PADS Configuration
#define PADS_ADDRESS 0x5D
#define PADS_DEVICE_ID_REG 0x0F
#define PADS_CTRL_1_REG 0x10
#define PADS_CTRL_2_REG 0x11
#define PADS_STATUS_REG 0x27
#define PADS_DATA_P_XL_REG 0x28
#define PADS_DATA_T_L_REG 0x2B
#define PADS_DEVICE_ID_VALUE 0xB3

// WSEN-HIDS Configuration
#define HIDS_ADDRESS 0x44
#define HIDS_MEASURE_HPM 0xFD
#define HIDS_SOFT_RESET 0x94
#define HIDS_MEASURE_SERIAL_NUMBER 0x89
#define HIDS_MEASURE_DELAY_HPM 20  // Increased from 15 to 20ms for reliability
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

// INA226 Configuration
#define INA1_ADDR 0x45
#define INA2_ADDR 0x41

// Display Configuration
#define DISPLAY_WITH_LABELS true
#define UPDATE_INTERVAL 2000
#define I2C_TIMEOUT 100
#define SERIAL_BUFFER_SIZE 512

INA226 ina1(INA1_ADDR, &Wire);
INA226 ina2(INA2_ADDR, &Wire);

// Timing variables for HIDS
unsigned long hidsStartTime = 0;
bool hidsWaiting = false;
bool hidsDataValid = false;

unsigned long lastDisplayTime = 0;

// Sensor data structure with mutex protection
struct SensorData {
  float pressure;
  float tempHIDS;
  float tempPADS;
  float tempFusion;
  float humidity;
  float voltage1;
  float current1;
  float power1;
  float voltage2;
  float current2;
  float power2;
  unsigned long timestamp;
  bool dataReady;
  bool hidsValid;  // Flag untuk validasi HIDS
} sensorData;

// Mutex for thread-safe data access
SemaphoreHandle_t dataMutex;
TaskHandle_t sensorTaskHandle = NULL;

// Function Prototypes
uint8_t PADS_readRegister(uint8_t reg);
void PADS_writeRegister(uint8_t reg, uint8_t value);
int32_t PADS_readPressure();
int16_t PADS_readTemperature();
bool PADS_init();

uint8_t calculateCRC8(uint8_t *data, uint8_t len);
bool verifyCRC8(uint8_t *data, uint8_t len, uint8_t crc);
bool HIDS_sendCommand(uint8_t command);
bool HIDS_readData(uint8_t *data, uint8_t len);
bool HIDS_init();
bool HIDS_softReset();
bool HIDS_readSerialNumber(uint32_t *serialNumber);
bool HIDS_measureBlocking(float *temperature, float *humidity);  // Changed to blocking

// Task functions
void sensorReadTask(void *parameter);
void displaySensorData();

void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  delay(1000);
  
  Serial.println(F("\n╔════════════════════════════════════════╗"));
  Serial.println(F("║  Multi-Sensor ESP32-S3 (HIDS Fixed)   ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));
  
  // Initialize I2C with timeout
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // Reduced to 100kHz for better stability
  Wire.setTimeOut(I2C_TIMEOUT);
  delay(100);
  
  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  
  // Initialize WSEN-PADS
  Serial.print(F("[PADS] "));
  Serial.println(PADS_init() ? F("✓ OK") : F("✗ FAIL"));
  
  // Initialize WSEN-HIDS
  Serial.print(F("[HIDS] "));
  if (HIDS_init()) {
    Serial.print(F("✓ OK | SN: 0x"));
    uint32_t serialNo = 0;
    if (HIDS_readSerialNumber(&serialNo)) {
      Serial.println(serialNo, HEX);
    } else {
      Serial.println();
    }
  } else {
    Serial.println(F("✗ FAIL"));
  }
  
  // Initialize INA226 #1
  Serial.print(F("[INA1] "));
  if (ina1.begin()) {
    ina1.setAverage(16);
    ina1.setMaxCurrentShunt(20.0, 0.001);
    Serial.println(F("✓ OK"));
  } else {
    Serial.println(F("✗ FAIL"));
  }
  
  // Initialize INA226 #2
  Serial.print(F("[INA2] "));
  if (ina2.begin()) {
    ina2.setAverage(16);
    ina2.setMaxCurrentShunt(20.0, 0.001);
    Serial.println(F("✓ OK"));
  } else {
    Serial.println(F("✗ FAIL"));
  }
  
  Serial.println(F("\n════════════ SYSTEM READY ════════════"));
  Serial.println(F("Transmitting every 2 seconds...\n"));
  
  // Initialize sensor data
  sensorData.dataReady = false;
  sensorData.hidsValid = false;
  
  // Create sensor reading task on Core 0
  xTaskCreatePinnedToCore(
    sensorReadTask,
    "SensorRead",
    8192,
    NULL,
    2,
    &sensorTaskHandle,
    0  // Core 0
  );
  
  delay(500);
}

void loop() {
  // This runs on Core 1 - dedicated to Serial output
  if (millis() - lastDisplayTime >= UPDATE_INTERVAL) {
    lastDisplayTime = millis();
    
    // Get data with mutex protection
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (sensorData.dataReady) {
        displaySensorData();
      }
      xSemaphoreGive(dataMutex);
    }
  }
  
  vTaskDelay(pdMS_TO_TICKS(100));
}

// Task running on Core 0 - dedicated to I2C sensor reading
void sensorReadTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500);  // Read every 500ms (slower but more reliable)
  
  while (true) {
    // Read WSEN-PADS
    uint8_t pads_status = PADS_readRegister(PADS_STATUS_REG);
    
    float pressure = 0.0;
    float tempPADS = 0.0;
    
    if (pads_status & 0x01) {
      int32_t pressure_raw = PADS_readPressure();
      if (pressure_raw != 0) {
        pressure = (pressure_raw / 40960.0) * 10.0;
      }
    }
    
    if (pads_status & 0x02) {
      int16_t temp_raw = PADS_readTemperature();
      tempPADS = temp_raw / 100.0;
    }
    
    // Read WSEN-HIDS (blocking mode for reliability)
    float tempHIDS = 0.0;
    float humidity = 0.0;
    bool hidsValid = HIDS_measureBlocking(&tempHIDS, &humidity);
    
    // Only update if HIDS reading is valid
    if (!hidsValid) {
      // Retry once if failed
      vTaskDelay(pdMS_TO_TICKS(50));
      hidsValid = HIDS_measureBlocking(&tempHIDS, &humidity);
    }
    
    // Read INA226 sensors
    float v1 = ina1.getBusVoltage();  // Calibration offset
    float i1 = ina1.getCurrent();
    float p1 = ina1.getPower();
    
    float v2 = ina2.getBusVoltage();
    float i2 = ina2.getCurrent();
    float p2 = ina2.getPower();
    
    // Calculate fusion temperature only if both sensors are valid
    float tempFusion = 0.0;
    if (hidsValid && tempPADS != 0.0) {
      tempFusion = (tempHIDS + tempPADS) / 2.0;
    } else if (hidsValid) {
      tempFusion = tempHIDS;
    } else if (tempPADS != 0.0) {
      tempFusion = tempPADS;
    }
    
    // Update shared data with mutex protection
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      sensorData.pressure = pressure;
      sensorData.tempPADS = tempPADS;
      sensorData.tempHIDS = tempHIDS;
      sensorData.humidity = humidity;
      sensorData.tempFusion = tempFusion;
      sensorData.voltage1 = v1;
      sensorData.current1 = i1;
      sensorData.power1 = p1;
      sensorData.voltage2 = v2;
      sensorData.current2 = i2;
      sensorData.power2 = p2;
      sensorData.timestamp = millis();
      sensorData.dataReady = true;
      sensorData.hidsValid = hidsValid;
      
      xSemaphoreGive(dataMutex);
    }
    
    // Wait for next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void displaySensorData() {
  #if DISPLAY_WITH_LABELS
    Serial.println(F("┌─────────────────────────────────────┐"));
    Serial.printf("│ Time: %.1f s", sensorData.timestamp / 1000.0);
    if (!sensorData.hidsValid) {
      Serial.print(F(" [HIDS: ✗]"));
    }
    Serial.println();
    
    Serial.println(F("├─────────────────────────────────────┤"));
    Serial.printf("│ Pressure:      %.2f hPa\n", sensorData.pressure);
    
    Serial.println(F("├─────────────────────────────────────┤"));
    
    if (sensorData.hidsValid) {
      Serial.printf("│ Temp (HIDS):   %.2f °C ✓\n", sensorData.tempHIDS);
    } else {
      Serial.println(F("│ Temp (HIDS):   -- °C (No Data)"));
    }
    
    Serial.printf("│ Temp (PADS):   %.2f °C\n", sensorData.tempPADS);
    Serial.printf("│ Temp (Fusion): %.2f °C\n", sensorData.tempFusion);
    
    if (sensorData.hidsValid) {
      Serial.printf("│ Humidity:      %.2f %%RH ✓\n", sensorData.humidity);
    } else {
      Serial.println(F("│ Humidity:      -- %RH (No Data)"));
    }
    
    Serial.println(F("├─────────────────────────────────────┤"));
    Serial.println(F("│ INA226 #1 (0x45)                   │"));
    Serial.printf("│   Voltage:     %.3f V\n", sensorData.voltage1);  // Calibration offset
    Serial.printf("│   Current:     %.2f mA\n", sensorData.current1 * 1000);
    Serial.printf("│   Power:       %.3f W\n", sensorData.power1);
    
    Serial.println(F("├─────────────────────────────────────┤"));
    Serial.println(F("│ INA226 #2 (0x41)                   │"));
    Serial.printf("│   Voltage:     %.3f V\n", sensorData.voltage2);
    Serial.printf("│   Current:     %.2f mA\n", sensorData.current2 * 1000);
    Serial.printf("│   Power:       %.3f W\n", sensorData.power2);
    Serial.println(F("└─────────────────────────────────────┘\n"));
    
  #else
    // CSV format
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                  sensorData.timestamp,
                  sensorData.pressure,
                  sensorData.tempHIDS,
                  sensorData.tempPADS,
                  sensorData.tempFusion,
                  sensorData.humidity,
                  sensorData.voltage1,
                  sensorData.current1,
                  sensorData.power1,
                  sensorData.voltage2,
                  sensorData.current2,
                  sensorData.power2,
                  sensorData.hidsValid ? 1 : 0);  // HIDS status flag
  #endif
}

// ========== WSEN-PADS Functions ==========
bool PADS_init() {
  uint8_t deviceID = PADS_readRegister(PADS_DEVICE_ID_REG);
  if (deviceID != PADS_DEVICE_ID_VALUE) return false;
  
  PADS_writeRegister(PADS_CTRL_1_REG, 0x30);
  PADS_writeRegister(PADS_CTRL_2_REG, 0x12);
  delay(50);
  
  return true;
}

uint8_t PADS_readRegister(uint8_t reg) {
  Wire.beginTransmission(PADS_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  
  Wire.requestFrom((uint8_t)PADS_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void PADS_writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PADS_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int32_t PADS_readPressure() {
  Wire.beginTransmission(PADS_ADDRESS);
  Wire.write(PADS_DATA_P_XL_REG);
  if (Wire.endTransmission(false) != 0) return 0;
  
  Wire.requestFrom((uint8_t)PADS_ADDRESS, (uint8_t)3);
  
  if (Wire.available() >= 3) {
    uint8_t xl = Wire.read();
    uint8_t l = Wire.read();
    uint8_t h = Wire.read();
    
    int32_t pressure = ((int32_t)h << 24) | ((int32_t)l << 16) | ((int32_t)xl << 8);
    pressure /= 256;
    return pressure;
  }
  return 0;
}

int16_t PADS_readTemperature() {
  Wire.beginTransmission(PADS_ADDRESS);
  Wire.write(PADS_DATA_T_L_REG);
  if (Wire.endTransmission(false) != 0) return 0;
  
  Wire.requestFrom((uint8_t)PADS_ADDRESS, (uint8_t)2);
  
  if (Wire.available() >= 2) {
    uint8_t l = Wire.read();
    uint8_t h = Wire.read();
    return ((int16_t)h << 8) | l;
  }
  return 0;
}

// ========== WSEN-HIDS Functions ==========
uint8_t calculateCRC8(uint8_t *data, uint8_t len) {
  uint8_t crc = CRC8_INIT;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ CRC8_POLYNOMIAL) : (crc << 1);
    }
  }
  return crc;
}

bool verifyCRC8(uint8_t *data, uint8_t len, uint8_t crc) {
  return (calculateCRC8(data, len) == crc);
}

bool HIDS_sendCommand(uint8_t command) {
  Wire.beginTransmission(HIDS_ADDRESS);
  Wire.write(command);
  return (Wire.endTransmission() == 0);
}

bool HIDS_readData(uint8_t *data, uint8_t len) {
  uint8_t received = Wire.requestFrom((uint8_t)HIDS_ADDRESS, len);
  if (received != len) return false;
  
  for (uint8_t i = 0; i < len; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      return false;
    }
  }
  return true;
}

bool HIDS_init() {
  return HIDS_softReset();
}

bool HIDS_softReset() {
  if (!HIDS_sendCommand(HIDS_SOFT_RESET)) return false;
  delay(2);
  return true;
}

bool HIDS_readSerialNumber(uint32_t *serialNumber) {
  uint8_t data[6];
  
  if (!HIDS_sendCommand(HIDS_MEASURE_SERIAL_NUMBER)) return false;
  delay(1);
  
  if (!HIDS_readData(data, 6)) return false;
  
  if (!verifyCRC8(&data[0], 2, data[2])) return false;
  if (!verifyCRC8(&data[3], 2, data[5])) return false;
  
  *serialNumber = ((uint32_t)data[0] << 24) | 
                  ((uint32_t)data[1] << 16) | 
                  ((uint32_t)data[3] << 8) | 
                  ((uint32_t)data[4]);
  
  return true;
}

// Changed to BLOCKING for reliability
bool HIDS_measureBlocking(float *temperature, float *humidity) {
  uint8_t data[6];
  
  // Send measurement command
  if (!HIDS_sendCommand(HIDS_MEASURE_HPM)) return false;
  
  // Wait for conversion to complete
  delay(HIDS_MEASURE_DELAY_HPM);
  
  // Read data
  if (!HIDS_readData(data, 6)) return false;
  
  // Verify CRC
  if (!verifyCRC8(&data[0], 2, data[2])) return false;
  if (!verifyCRC8(&data[3], 2, data[5])) return false;
  
  // Parse temperature
  uint16_t tempRaw = (data[0] << 8) | data[1];
  *temperature = -45.0 + 175.0 * ((float)tempRaw / 65535.0);
  
  // Parse humidity
  uint16_t humRaw = (data[3] << 8) | data[4];
  *humidity = 100.0 * ((float)humRaw / 65535.0);
  
  // Constrain humidity
  if (*humidity < 0) *humidity = 0;
  if (*humidity > 100) *humidity = 100;
  
  // Validate readings (sanity check)
  if (*temperature < -40.0 || *temperature > 125.0) return false;
  
  return true;
}