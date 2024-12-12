#include <ArduinoBLE.h>
#include <SPI.h>
#include <SdFat.h>
#include "LSM6DS3.h" // LSM6DS3 library

// BLE Service and Characteristics
BLEService imuService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEStringCharacteristic commandCharacteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLEWrite, 20); // Command input
BLEStringCharacteristic dataCharacteristic("19b10002-e8f2-537e-4f6c-d104768a1214", BLERead, 512);   // Data output

// IMU and SdFat
LSM6DS3 imu(SPI_MODE, 10);      // Use SPI mode and specify the chip select pin for IMU
SdFat sd;                       // SdFat object
SdFile logFile;                 // SdFat file object
const uint8_t SD_CS_PIN = 4;    // Chip select pin for SD card (adjust if necessary)
bool recording = false;

// Sampling interval in milliseconds
const unsigned long samplingInterval = 100; // Adjust this value for desired frequency (100 ms = 10 Hz)
unsigned long lastSampleTime = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  // Initialize IMU
  if (imu.begin() != 0) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized successfully.");

  // Initialize SD card
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(25))) { // Adjust SPI clock speed if needed
    Serial.println("Failed to initialize SD card with SdFat!");
    while (1);
  }
  Serial.println("SD card initialized successfully with SdFat.");

  // Start BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Configure BLE settings
  BLE.setLocalName("XIAO-IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(commandCharacteristic);
  imuService.addCharacteristic(dataCharacteristic);
  BLE.addService(imuService);

  // Start Advertising
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // While Central is Connected
    while (central.connected()) {
      if (commandCharacteristic.written()) {
        String command = commandCharacteristic.value();
        if (command == "start") {
          startRecording();
        } else if (command == "stop") {
          stopRecording();
        }
      }

      if (recording) {
        recordIMUData();
      }
    }

    // Central Disconnected
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void startRecording() {
  if (!recording) {
    if (logFile.open("imu_data.csv", O_RDWR | O_CREAT | O_TRUNC)) {
      logFile.println("Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z");
      logFile.flush();
      recording = true;
      lastSampleTime = millis(); // Reset sampling timer
      Serial.println("Recording started...");
    } else {
      Serial.println("Failed to open log file!");
    }
  }
}

void stopRecording() {
  if (recording) {
    recording = false;
    logFile.close();
    Serial.println("Recording stopped. Sending data...");

    // Reopen the file for reading
    if (logFile.open("imu_data.csv", O_READ)) {
      char buffer[512];
      while (logFile.available()) {
        size_t bytesRead = logFile.read(buffer, sizeof(buffer));
        String chunkData = String(buffer).substring(0, bytesRead);
        dataCharacteristic.writeValue(chunkData.c_str());
        delay(100); // Allow BLE time to transfer data
      }
      logFile.close();
      Serial.println("Data sent successfully.");
    } else {
      Serial.println("Failed to reopen log file for reading!");
    }

    // Optionally, delete the file after transfer
    sd.remove("imu_data.csv");
  }
}

void recordIMUData() {
  // Check if enough time has passed since the last sample
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime >= samplingInterval) {
    lastSampleTime = currentTime; // Update the last sample time

    // Read IMU data
    float accelX = imu.readFloatAccelX();
    float accelY = imu.readFloatAccelY();
    float accelZ = imu.readFloatAccelZ();
    float gyroX = imu.readFloatGyroX();
    float gyroY = imu.readFloatGyroY();
    float gyroZ = imu.readFloatGyroZ();

    // Prepare data as CSV line
    String data = String(currentTime) + "," +
                  String(accelX) + "," +
                  String(accelY) + "," +
                  String(accelZ) + "," +
                  String(gyroX) + "," +
                  String(gyroY) + "," +
                  String(gyroZ);

    // Log data to SD card
    if (logFile) {
      logFile.println(data);
      logFile.flush(); // Ensure data is written to the SD card
    } else {
      Serial.println("Failed to write data to log file!");
    }
  }
}
