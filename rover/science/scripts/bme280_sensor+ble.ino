#include <ArduinoBLE.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1011.3)

Adafruit_BME280 bme;

// BLE Service
BLEService envService("181A"); // Environmental Sensing Service

// 4 readable characteristics
BLEStringCharacteristic tempChar("2A6E", BLERead | BLENotify, 20);
BLEStringCharacteristic humChar ("2A6F", BLERead | BLENotify, 20);
BLEStringCharacteristic presChar("2A6D", BLERead | BLENotify, 20);
BLEStringCharacteristic altChar ("2AB3", BLERead | BLENotify, 20);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // ---- BME280 ----
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found");
    while (1);
  }

  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::FILTER_OFF
  );

  // ---- BLE ----
  if (!BLE.begin()) {
    Serial.println("BLE failed");
    while (1);
  }

  BLE.setLocalName("Nano-Environemtal-Sensor");
  BLE.setAdvertisedService(envService);

  envService.addCharacteristic(tempChar);
  envService.addCharacteristic(humChar);
  envService.addCharacteristic(presChar);
  envService.addCharacteristic(altChar);

  BLE.addService(envService);

  BLE.advertise();
  Serial.println("BLE advertising");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to ");
    Serial.println(central.address());

    while (central.connected()) {
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      float p = bme.readPressure() / 100.0f;
      float a = bme.readAltitude(SEALEVELPRESSURE_HPA);

      char buf[20];

      snprintf(buf, sizeof(buf), "%.2f C", t);
      tempChar.writeValue(buf);

      snprintf(buf, sizeof(buf), "%.1f %%", h);
      humChar.writeValue(buf);

      snprintf(buf, sizeof(buf), "%.1f hPa", p);
      presChar.writeValue(buf);

      snprintf(buf, sizeof(buf), "%.1f m", a);
      altChar.writeValue(buf);

      Serial.println("BLE values updated");
      delay(2000);
    }

    Serial.println("Disconnected");
  }
}
