#include "BLETypedCharacteristics.h"
#include "ble.h"

#include <ArduinoBLE.h>
#include "main.h"

BLEService tofViz("180F");
BLEService stateViz("9b7796b6-f098-4f51-84ce-a11f7219300c");

// statically allocate all characteristics x_x
BLEFloatCharacteristic tofCharacteristics[] = {
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
  BLEFloatCharacteristic("ABCD", BLERead|BLENotify),
};

BLEIntCharacteristic bleIMUOrientation("0000", BLERead|BLENotify);
BLEIntCharacteristic bleThrowbotState("0001", BLERead|BLENotify);

void initBLE() {
  if (!BLE.begin()) {
    Serial.println("couldn't start BLE :(");
  }
  BLE.setLocalName("G21 Artemis");
  BLE.setAdvertisedService(tofViz);
  for (uint8_t i = 0; i < 64; i++) {
    tofViz.addCharacteristic(tofCharacteristics[i]);
  }
  BLE.addService(tofViz);
  for (int i = 0; i < 64; i++) {
    tofCharacteristics[i].writeValue(0);
  }
  stateViz.addCharacteristic(bleIMUOrientation);
  stateViz.addCharacteristic(bleThrowbotState);
  BLE.addService(stateViz);
  BLE.advertise();
  Serial.println("waiting for a BLE connection");
}

void BLEComm() {
  while (1) {
    BLEDevice connectedComputer = BLE.central();
    if (connectedComputer) {

      while (connectedComputer.connected()) {
        tofDataLock.lock();
        for (int i = 0; i < 64; i++) {
          tofCharacteristics[i].writeValue(tofNormalized[i]);
        }
        tofDataLock.unlock();
        bleIMUOrientation.writeValue(orientation);
        bleThrowbotState.writeValue(throwbotState);
        rtos::ThisThread::sleep_for(100ms);
      }

    }
    rtos::ThisThread::sleep_for(100ms);
  }

}
