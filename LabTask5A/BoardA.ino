
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>

const char *DEVICE_NAME = "RSSI_BEACON";

constexpr int TX_POWER_DBM = 0;

void setup()
{
    Serial.begin(115200); // Start UART
    delay(1000);

    Serial.println("Starting BLE advertiser...");

    BLEDevice::init(DEVICE_NAME); // Set device name

    // Set advertising transmit power.
    esp_power_level_t powerLevel;

    switch (TX_POWER_DBM)
    {
        case -12: powerLevel = ESP_PWR_LVL_N12; break;
        case -9:  powerLevel = ESP_PWR_LVL_N9;  break;
        case -6:  powerLevel = ESP_PWR_LVL_N6;  break;
        case -3:  powerLevel = ESP_PWR_LVL_N3;  break;
        case 0:   powerLevel = ESP_PWR_LVL_N0;  break;
        case 3:   powerLevel = ESP_PWR_LVL_P3;  break;
        case 6:   powerLevel = ESP_PWR_LVL_P6;  break;
        case 9:   powerLevel = ESP_PWR_LVL_P9;  break;
        default:
            Serial.println("Unsupported TX_POWER_DBM!");
            while (true) delay(1000);
    }

    BLEDevice::setPower(powerLevel, ESP_BLE_PWR_TYPE_ADV);

    Serial.print("Configured BLE advertising power: ");
    Serial.print(TX_POWER_DBM);
    Serial.println(" dBm");

    BLEServer *server = BLEDevice::createServer(); // Create a BLE server on the ESP32.

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising(); // Get the BLE advertising object.

    advertising->addServiceUUID(
        "12345678-1234-1234-1234-123456789ABC"
    ); // Add a service UUID to identify the advertised BLE service.

    advertising->start(); // Start BLE advertising so nearby devices can discover the board.

    Serial.println("BLE advertising started!");
}

void loop()
{
    delay(1000);
}
