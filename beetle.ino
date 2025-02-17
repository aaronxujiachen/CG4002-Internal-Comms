#include <Arduino.h>
#include <Wire.h>
#include <FastCRC.h>

FastCRC8 crc8;  // CRC-8 instance for checksum calculation

/**
 * Packet class: Represents a 20-byte packet structure.
 * Format:
 * - 1 byte  → Packet Type
 * - 1 byte  → Device ID
 * - 17 bytes → Sensor Data (6 values: 3 accelerometer + 3 gyroscope + 5 padding)
 * - 1 byte  → CRC Checksum
 */
class Packet {
    public:
        char packetType;
        char deviceId;
        uint8_t data[17] = {0};  // 17-byte sensor data + padding
        uint8_t crc;

        /**
         * Constructor: Initializes packet fields.
         * @param packetType Type of packet (e.g., 'M' for IMU data)
         * @param deviceId Device identifier
         * @param data Pointer to the 17-byte data array
         */
        Packet(char packetType, char deviceId, uint8_t* data) {
            this->packetType = packetType;
            this->deviceId = deviceId;
            memcpy(this->data, data, 17);  // Copy provided data into packet
            crc = 0;  // Initialize CRC (computed later)
        }

        /**
         * Formats the packet into a 20-byte buffer.
         * Computes CRC-8 and adds it to the packet.
         */
        void formatPacket(uint8_t* buffer) {
            buffer[0] = packetType;
            buffer[1] = deviceId;
            memcpy(&buffer[2], data, sizeof(data));  // Copy 17-byte data
            crc = crc8.smbus(buffer, 19);  // Compute CRC-8 checksum for first 19 bytes
            buffer[19] = crc;  // Append checksum at last byte
        }

        /**
         * Sends the formatted packet over Serial.
         */
        void sendPacket() {
            uint8_t buffer[20];  // Buffer for formatted packet
            formatPacket(buffer);  // Prepare packet
            Serial.write(buffer, 20);  // Send packet via Serial
            Serial.flush();  // Ensure transmission completion
        }
};

/**
 * **Dummy IMU Data (8 Sets)**
 * Each row contains 6 values:
 * - **Accelerometer**: `ax, ay, az` (in mg or m/s²)
 * - **Gyroscope**: `gx, gy, gz` (in degrees per second, dps)
 * - **Stored as int16_t (2 bytes per value)**
 */
const int16_t imuDummyData[8][6] = {
    {100, -200, 300, 500, -600, 700},  // Sample 1
    {150, -250, 350, 550, -650, 750},  // Sample 2
    {200, -300, 400, 600, -700, 800},  // Sample 3
    {250, -350, 450, 650, -750, 850},  // Sample 4
    {300, -400, 500, 700, -800, 900},  // Sample 5
    {350, -450, 550, 750, -850, 950},  // Sample 6
    {400, -500, 600, 800, -900, 1000}, // Sample 7
    {450, -550, 650, 850, -950, 1050}  // Sample 8
};

/**
 * Retrieves a random IMU sample and formats it into a 17-byte buffer.
 * The IMU values are stored in **Little Endian** format.
 * 
 * @param buffer Pointer to a 17-byte buffer where the formatted IMU data is stored.
 */
void getImuData(uint8_t* buffer) {
    int index = random(0, 8);  // Choose a random sample from imuDummyData
    Serial.print("Selected Sample Index: ");
    Serial.println(index);
    
    for (int i = 0; i < 6; i++) {
        buffer[i * 2] = imuDummyData[index][i] & 0xFF;            // Lower byte
        buffer[i * 2 + 1] = (imuDummyData[index][i] >> 8) & 0xFF; // Upper byte
    }
    
    // Fill the remaining 5 bytes with padding (zeroes)
    memset(&buffer[12], 0, 5);
}

void setup() {
    Serial.begin(115200);  // Start Serial communication at 115200 baud rate
    randomSeed(analogRead(0));  // Initialize random seed from an unconnected analog pin
}

void loop() {
    uint8_t imuData[17] = {0};  // Buffer for IMU data
    getImuData(imuData);  // Fill buffer with a random IMU sample

    // Create and send a packet containing IMU data
    Packet imuPacket('M', '1', imuData);
    imuPacket.sendPacket();

    delay(1000);  // Send a packet every second
}
