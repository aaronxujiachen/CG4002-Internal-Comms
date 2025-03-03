#include <Arduino.h>
#include <Wire.h>
#include <FastCRC.h>

FastCRC8 crc8;  // CRC-8 instance for checksum calculation

#define PACKET_SIZE 20  // Each packet should be exactly 20 bytes

/**
 * Packet class: Represents a 20-byte packet structure.
 */
class Packet {
    public:
        char packetType;
        char deviceId;
        uint8_t data[17] = {0};  // Sensor data + padding
        uint8_t crc;

        Packet(char packetType, char deviceId, uint8_t* data) {
            this->packetType = packetType;
            this->deviceId = deviceId;
            memcpy(this->data, data, 17);  // Copy provided data
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
            crc = crc8.smbus(buffer, 19);  // Compute CRC-8 checksum
            buffer[19] = crc;  // Append checksum
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
 * **Reads and validates an incoming packet from Serial.**
 * 
 * @param receivedPacket Reference to store the received packet.
 * @return True if a valid packet was received, false otherwise.
 */
bool receivePacket(Packet &receivedPacket) {
    uint8_t buffer[PACKET_SIZE];

    // Wait until a full packet is available
    if (Serial.available() >= PACKET_SIZE) {
        Serial.readBytes(buffer, PACKET_SIZE);  // Read 20 bytes

        // Extract packet fields
        char packetType = buffer[0];
        char deviceId = buffer[1];
        uint8_t data[17];
        memcpy(data, &buffer[2], 17);  // Copy sensor data
        uint8_t receivedCRC = buffer[19];

        // Compute expected CRC
        uint8_t expectedCRC = crc8.smbus(buffer, 19);

        // Validate checksum
        if (receivedCRC != expectedCRC) {
            // Serial.println("[ERROR] Checksum mismatch! Dropping packet.");
            return false;
        }

        // Create the received packet
        receivedPacket = Packet(packetType, deviceId, data);
        return true;
    }
    return false;
}

/**
 * **Handles handshake by responding to SYN ('S') packets with ACK ('A').**
 */
bool handshaked = false;
void handleHandshake() {
    Packet receivedPacket(' ', ' ', nullptr);

    if (receivePacket(receivedPacket)) {
        char packetType = receivedPacket.packetType;
        char deviceId = receivedPacket.deviceId;

        // // Serial.print("[INFO] Received Handshake Packet: ");
        // Serial.print(packetType);
        // // Serial.print(" from Device: ");
        // Serial.println(deviceId);

        if (packetType == 'S') {  // If received SYN, respond with ACK
            // Serial.println("[INFO] Sending ACK...");
            uint8_t dummyData[17] = {0};  // No sensor data needed
            Packet ackPacket('A', deviceId, dummyData);
            ackPacket.sendPacket();
            handshaked=true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    // Serial.println("[INFO] Bluno Beetle Ready.");
}

void loop() {
    // Check for handshake initiation
    handleHandshake();

    // Normal data transmission after handshake
    if(handshaked) {
    uint8_t imuData[17] = {0};
    Packet imuPacket('M', '1', imuData);
    imuPacket.sendPacket();
    }
    delay(1000);  // Send IMU data every second
}
