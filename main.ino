#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define MPU6500_ADDR     0x68
#define WHO_AM_I         0x75
#define PWR_MGMT_1       0x6B
#define ACCEL_XOUT_H     0x3B

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 1);
  return Wire.read();
}

void readAccelGyro(int16_t *accel, int16_t *gyro, int16_t *temp) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if(Wire.endTransmission(false) != 0) {
    return;
  }
  
  Wire.requestFrom(MPU6500_ADDR, 14);

  if (Wire.available() != 14) {
    return;
  }

  uint8_t raw[14];
  for (int i = 0; i < 14; i++) {
    raw[i] = Wire.read();
  }

  // Accelerometer
  accel[0] = (raw[0] << 8) | raw[1];
  accel[1] = (raw[2] << 8) | raw[3];
  accel[2] = (raw[4] << 8) | raw[5];

  // Temperature
  *temp = (raw[6] << 8) | raw[7];

  // Gyroscope
  gyro[0] = (raw[8] << 8) | raw[9];
  gyro[1] = (raw[10] << 8) | raw[11];
  gyro[2] = (raw[12] << 8) | raw[13];
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Wake up MPU6500
  writeRegister(PWR_MGMT_1, 0x00);
  delay(100);

  // Check WHO_AM_I
  uint8_t who = readRegister(WHO_AM_I);
  Serial.println(who);

  BLEDevice::init("CALI_MPU");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  pService->start();

  pServer->getAdvertising()->start();
  Serial.println("Waiting for BLE client...");
  pinMode(A0, INPUT);         // Configure A0 as ADC input
  
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(100);                          

  int16_t accel[3], gyro[3], temp;
  readAccelGyro(accel, gyro, &temp);
  
  Serial.print("Accel X: "); Serial.print(accel[0]);
  Serial.print(" Y: "); Serial.print(accel[1]);
  Serial.print(" Z: "); Serial.print(accel[2]);

  Serial.print("| Temp: "); Serial.print(temp);

  Serial.print(" | Gyro X: "); Serial.print(gyro[0]);
  Serial.print(" Y: "); Serial.print(gyro[1]);
  Serial.print(" Z: "); Serial.println(gyro[2]);

  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(A0); // Read and accumulate ADC voltage
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;     // Adjust for 1:2 divider and convert to volts
  Serial.println(Vbattf, 3); 

  char buffer[256];
  snprintf(buffer, sizeof(buffer), "[%d,%d,%d,%d,%d,%d]", accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);

  if (deviceConnected) {
      pCharacteristic->setValue(buffer);
      pCharacteristic->notify();
  }

  digitalWrite(LED_BUILTIN, LOW);    
  delay(100);


}
