#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---------------- BLE configuration ----------------

const char *TARGET_NAME = "RSSI_BEACON";

constexpr int RSSI_SAMPLE_COUNT = 20;

BLEScan *scanner;

// ---------------- UART configuration ----------------

constexpr int UART_TX_PIN = 17;
constexpr int UART_RX_PIN = 18;
constexpr uint32_t UART_BAUD = 115200;

constexpr uint8_t START_BYTE = 0xA5;
constexpr uint8_t RSSI_READING = 0x01;
constexpr uint8_t END_FRAME = 0x02;

HardwareSerial STM32Serial(1);

uint8_t sequenceNumber = 0;

// ---------------- RSSI batch ----------------

int8_t rssiValues[RSSI_SAMPLE_COUNT];
int rssiCount = 0;

// ---------------- CRC16/MODBUS ----------------

uint16_t calculateCRC(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}

// ---------------- UART frame transmission ----------------

void sendFrame(uint8_t type,
               const uint8_t *payload,
               uint8_t payloadLength)
{
    // LEN = TYPE + SEQ + payload
    uint8_t length = 2 + payloadLength;
    uint8_t frame[8];

    frame[0] = START_BYTE;
    frame[1] = length;
    frame[2] = type;
    frame[3] = sequenceNumber++;

    for (uint8_t i = 0; i < payloadLength; i++)
        frame[4 + i] = payload[i];

    // CRC covers LEN, TYPE, SEQ and payload.
    uint16_t crc = calculateCRC(&frame[1], length + 1);

    frame[4 + payloadLength] = crc & 0xFF;
    frame[5 + payloadLength] = (crc >> 8) & 0xFF;

    STM32Serial.write(frame, 4 + payloadLength + 2);
}

void sendRSSI(int8_t rssi)
{
    uint8_t payload = (uint8_t)rssi;
    sendFrame(RSSI_READING, &payload, 1);
}

void sendEnd()
{
    sendFrame(END_FRAME, nullptr, 0);
}

// ---------------- BLE scan callback ----------------

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice device) override
    {
        if (rssiCount >= RSSI_SAMPLE_COUNT)
            return;

        if (device.haveName() &&
            device.getName() == TARGET_NAME)
        {
            int8_t rssi = (int8_t)device.getRSSI();

            rssiValues[rssiCount] = rssi;
            rssiCount++;

            Serial.print("Collected ");
            Serial.print(rssiCount);
            Serial.print("/");
            Serial.print(RSSI_SAMPLE_COUNT);
            Serial.print(" | RSSI: ");
            Serial.print((int)rssi);
            Serial.println(" dBm");
        }
    }
};

// ---------------- Setup ----------------

void setup()
{
    Serial.begin(115200);
    delay(1000);

    STM32Serial.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN
    );

    BLEDevice::init("");

    scanner = BLEDevice::getScan();

    scanner->setAdvertisedDeviceCallbacks(
        new ScanCallbacks()
    );

    scanner->setActiveScan(true);
    scanner->setInterval(100);
    scanner->setWindow(99);
}

// Scan, collect 20 readings, and transmit the batch
void loop()
{
    if (rssiCount < RSSI_SAMPLE_COUNT)
    {
        // Collect readings over one or more scan periods.
        scanner->start(3, false);
        scanner->clearResults();
    }

    if (rssiCount == RSSI_SAMPLE_COUNT)
    {
        for (int i = 0; i < RSSI_SAMPLE_COUNT; i++)
        {
            sendRSSI(rssiValues[i]); // Send each RSSI READING
        }

        sendEnd(); // Sending END Frame
        rssiCount = 0;
        delay(500);
    }
}
