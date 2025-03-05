#include <Arduino.h>
#include <Wire.h>
#include <FastCRC.h>
#include <MPU6050.h>
#include <U8g2lib.h>
#include <IRremote.hpp>

// --- Define Macros ---
#define BUZZER_PIN 2          // Buzzer on D2
#define IR_SEND_PIN 3         // IR LED on D3
#define BUTTON_PIN 4          // Push button on D4
#define SHOOTING_DURATION 100 // Buzzer on duration in ms
#define IR_ADDRESS_ID 0xB3
#define IR_PLAYER_ID 0x01

#define MOTION_THRESHOLD 10

#define PACKET_SIZE 20  // Each packet should be exactly 20 bytes

// --- Initialise OLED and MPU instance ---
MPU6050 mpu;
FastCRC8 crc8;  // CRC-8 instance for checksum calculation
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// --- Global Variables ---
float alpha = 0.1;
int8_t smooth_gX = 0;
int8_t smooth_gY = 0;
int8_t smooth_gZ = 0;
int8_t smooth_aX = 0;
int8_t smooth_aY = 0;
int8_t smooth_aZ = 0;

int ammo_count = 6;
const int max_ammo = 6;

unsigned long shootStartTime = 0;
bool isShooting = false;
bool hasShot = false;

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

// --- HELPER FUNCTIONS ---
void initIR()
{
  IrSender.begin(IR_SEND_PIN);
//  Serial.println("IR Transmitter initialized.");
}

void sendIRSignal()
{
  uint8_t address = IR_ADDRESS_ID;
  uint8_t command = IR_PLAYER_ID;

//  Serial.println("Sending IR signal...");
  IrSender.sendNEC(address, command, 0);
//  Serial.print("Address: ");
//  Serial.print(address, HEX);
//  Serial.print(", Command: ");
//  Serial.println(command, HEX);
}

void initIMU()
{
//  Serial.println(F("Initializing MPU6050..."));
  while (!mpu.begin(MPU6050_SCALE_1000DPS, MPU6050_RANGE_2G))
  {
//    Serial.println(F("MPU6050 NOT found!"));
    delay(500);
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);

  mpu.setDLPFMode(MPU6050_DLPF_2);

//  Serial.println(F("MPU6050 initialized."));
}

bool isMotionDetected(int8_t smooth_gX, int8_t smooth_gY, int8_t smooth_gZ, int8_t smooth_aX, int8_t smooth_aY, int8_t smooth_aZ)
{
  int16_t accel_magnitude = sqrt(smooth_aX * smooth_aX + smooth_aY * smooth_aY + smooth_aZ * smooth_aZ);
  return accel_magnitude > MOTION_THRESHOLD;
}

void initOLED()
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
}

void initBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void readIMU()
{
  Vector normGyro = mpu.readNormalizeGyro();
  Vector normAccel = mpu.readNormalizeAccel();

  smooth_gX = alpha * normGyro.XAxis + (1 - alpha) * smooth_gX;
  smooth_gY = alpha * normGyro.YAxis + (1 - alpha) * smooth_gY;
  smooth_gZ = alpha * normGyro.ZAxis + (1 - alpha) * smooth_gZ;

  smooth_aX = normAccel.XAxis;
  smooth_aY = normAccel.YAxis;
  smooth_aZ = normAccel.ZAxis;

//  Serial.println(F("==== Sensor Data ===="));
//  Serial.print(F(" gX: "));
//  Serial.print(smooth_gX);
//  Serial.print(F(" | gY: "));
//  Serial.print(smooth_gY);
//  Serial.print(F(" | gZ: "));
//  Serial.print(smooth_gZ);
//
//  Serial.print(F(" aX: "));
//  Serial.print(smooth_aX);
//  Serial.print(F(" | aY: "));
//  Serial.print(smooth_aY);
//  Serial.print(F(" | aZ: "));
//  Serial.println(smooth_aZ);
}

void drawOLED()
{
  u8g2.firstPage();
  do
  {
    int barWidth = 10;  // Total width of the ammo bar
    int barHeight = 20; // Height of the bar
    int startX = 10;    // X position of the bar
    int startY = 35;    // Y position of the bar

    u8g2.drawFrame(5, 10, 95, 50);

    for (int i = 0; i < ammo_count; i++)
    {
      u8g2.drawTriangle(startX, startY, startX + 5, startY - 20, startX + 10, startY);
      u8g2.drawBox(startX, startY, barWidth, barHeight);
      startX += 15;
    }
  } while (u8g2.nextPage());
}

void setAmmo() {
  static char inputBuffer[5];
  static byte index = 0;
  while (Serial.available()) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      inputBuffer[index] = '\0';
      ammo_count = atoi(inputBuffer);
      index = 0;
    } else {
      if (index < sizeof(inputBuffer) - 1) { 
        inputBuffer[index++] = receivedChar;
      }
    }
  }
}

void reload()
{
  ammo_count = max_ammo;
}

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
            Packet ackPacket('Y', deviceId, dummyData);
            ackPacket.sendPacket();
//            handshaked=true;
        }

        if (packetType == 'A') {  // If received SYN, respond with ACK
//            Serial.println("[INFO] Sending Final ACK...");
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

    Wire.begin();

    // Initialise Peripherals
    initIMU();
    initIR();
    initOLED();
    initBuzzer();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
    // Check for handshake initiation
    handleHandshake();

    // Normal data transmission after handshake
    if(handshaked) {
        readIMU();
//        bool isActivity = isMotionDetected(smooth_gX, smooth_gY, smooth_gZ, smooth_aX, smooth_aY, smooth_aZ);
        bool isActivity = true;

        if (isActivity) {
            uint8_t imuData[17] = {0};
            imuData[0] = smooth_gX;
            imuData[1] = smooth_gY;
            imuData[2] = smooth_gZ;
            imuData[3] = smooth_aX;
            imuData[4] = smooth_aY;
            imuData[5] = smooth_aZ;
            
            Packet imuPacket('M', '1', imuData);
            imuPacket.sendPacket();
        }

        drawOLED();
        setAmmo();
      
        if (digitalRead(BUTTON_PIN) == HIGH && !hasShot && ammo_count > 0 && !isShooting)
        {
          ammo_count -= 1;
          drawOLED();
//          Serial.println("Shooting");
      
          isShooting = true;
          shootStartTime = millis();
      
          // digitalWrite(BUZZER_PIN, HIGH);
          tone(BUZZER_PIN, 1000);
          sendIRSignal();
        }
      
        if (isShooting)
        {
          unsigned long elapsedTime = millis() - shootStartTime;
      
          if (elapsedTime >= SHOOTING_DURATION)
          {
            // digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer after SHOOTING_DURATION
            noTone(BUZZER_PIN);
            isShooting = false;            // Reset shooting state
          }
        }
      
        if (ammo_count == 0 && digitalRead(BUTTON_PIN) == HIGH)
        {
          reload();
        }
      
        hasShot = digitalRead(BUTTON_PIN);
    }
    delay(50);  // Send IMU data every second
}
