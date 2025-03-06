import threading
import time
from bluepy import btle
import struct
from crc import Calculator, Configuration

PACKET_SIZE = 20  # Each packet should be 20 bytes

def parse_imu_data(name, data):
    """
    Parses and prints IMU data received from Arduino.
    """
    if not isinstance(data, bytes) or len(data) < 19:
        print("Invalid IMU data received.")
        return
    
    imu_values = list(data[2:])
    print(f"[{name}] Parsed IMU Data: Gyro X: {imu_values[0]} Gyro Y: {imu_values[1]} Gyro Z: {imu_values[2]} Accel X: {imu_values[3]} Accel Y: {imu_values[4]} Accel Z: {imu_values[5]}")

class Packet:
    PACKET_FORMAT = "1s1s12s5sB"
    PACKET_SIZE = 20

    crc_config = Configuration(
        width=8, 
        polynomial=0x07, 
        init_value=0x00, 
        final_xor_value=0x00, 
        reverse_input=False, 
        reverse_output=False
    )
    
    crc_calculator = Calculator(crc_config)

    def __init__(self, packet_type: str, device_id: str, sensor_data: bytes = b""):
        """Initializes a structured packet."""
        if len(packet_type) != 1 or len(device_id) != 1:
            raise ValueError("Packet Type and Device ID must be exactly 1 character each.")

        if len(sensor_data) > 12:
            raise ValueError("Sensor data must be at most 12 bytes.")

        self.packet_type = packet_type.encode()
        self.device_id = device_id.encode()
        self.sensor_data = sensor_data.ljust(12, b'\x00')
        self.padding = b'\x00' * 5
        self.checksum = self.calculate_checksum()

    def calculate_checksum(self) -> bytes:
        """Computes CRC-8 checksum using the SMBus polynomial (0x07)."""
        data = self.packet_type + self.device_id + self.sensor_data + self.padding
        checksum_value = self.crc_calculator.checksum(data)
        return checksum_value.to_bytes(1, 'big')

    def pack(self) -> bytes:
        """Packs the packet into a 20-byte binary format."""
        return struct.pack(
            self.PACKET_FORMAT, 
            self.packet_type, 
            self.device_id, 
            self.sensor_data, 
            self.padding, 
            int.from_bytes(self.checksum, "big")
        )

    @classmethod
    def unpack(cls, packet_bytes: bytes):
        """Unpacks a 20-byte packet and creates a Packet instance."""
        if len(packet_bytes) != cls.PACKET_SIZE:
            raise ValueError("Invalid packet size. Expected 20 bytes.")

        packet_type, device_id, sensor_data, padding, checksum = struct.unpack(cls.PACKET_FORMAT, packet_bytes)
        data = packet_type + device_id + sensor_data + padding
        expected_checksum = cls.crc_calculator.checksum(data)

        return cls(packet_type.decode().strip('\x00'), device_id.decode().strip('\x00'), sensor_data.rstrip(b'\x00'))

    def __repr__(self):
        return f"Packet(Type={self.packet_type.decode()}, DeviceID={self.device_id.decode()}, SensorData={self.sensor_data}, Checksum={self.checksum.hex()})"

class BlunoDelegate(btle.DefaultDelegate):
    """Handles incoming BLE notifications."""
    def __init__(self, device_name):
        super().__init__()
        self.device_name = device_name
        self.received_packet = None
        self.buffer = b''

    def handleNotification(self, cHandle, data):
        """Processes incoming BLE notifications (20-byte packets)."""
        try:
            self.buffer += data
            if len(self.buffer) < 20:
                return
            packet = self.buffer[:20]
            self.buffer = self.buffer[20:]
            self.received_packet = Packet.unpack(packet)
            print(f"[{self.device_name}] Received Packet: {self.received_packet}")

            packet_type = self.received_packet.packet_type.decode()
            if packet_type == "M":
                parse_imu_data(self.device_name, data)
        except Exception as e:
            print(f"[{self.device_name}] Error decoding packet: {e}")

class BlunoBeetle(threading.Thread):
    """Handles BLE connection, handshake, and data retrieval for a Bluno Beetle."""
    def __init__(self, mac_address, name="Bluno"):
        threading.Thread.__init__(self)
        self.mac_address = mac_address
        self.name = name
        self.running = True
        self.peripheral = None
        self.tx_characteristic = None

    def connect(self):
        """Establishes a BLE connection with the Bluno Beetle."""
        try:
            print(f"[{self.name}] Connecting to {self.mac_address}...")
            self.peripheral = btle.Peripheral(self.mac_address)
            self.delegate = BlunoDelegate(self.name)
            self.peripheral.withDelegate(self.delegate)

            self.tx_characteristic = self.peripheral.getServiceByUUID('dfb0').getCharacteristics('dfb1')[0]
            print(f"[{self.name}] Connected successfully.")
            return True
        except Exception as e:
            print(f"[{self.name}] Connection failed: {e}")
            return False

    def handshake(self):
        """Performs a Three-Way Handshake using structured packets."""
        if not self.tx_characteristic:
            return False

        print(f"[{self.name}] Starting Three-Way Handshake...")
        self.tx_characteristic.write(Packet("S", "D").pack())
        time.sleep(1)

        start_time = time.time()
        while time.time() - start_time < 3.0:
            if self.peripheral.waitForNotifications(1.0):
                if self.delegate.received_packet and self.delegate.received_packet.packet_type.decode() == "Y":
                    print(f"[{self.name}] -> SYN-ACK received.")
                    break
        else:
            print(f"[{self.name}] Handshake failed.")
            return False

        self.tx_characteristic.write(Packet("A", "D").pack())
        time.sleep(1)
        print(f"[{self.name}] Handshake Complete!")
        return True

    def run(self):
        """Main thread function for receiving data."""
        if not self.connect():
            return
        if not self.handshake():
            return

        while self.running:
            try:
                self.peripheral.waitForNotifications(1.0)
            except Exception as e:
                print(f"[{self.name}] Error: {e}")
                break

    def stop(self):
        """Stops the thread and disconnects from the Bluno Beetle."""
        self.running = False
        try:
            if self.peripheral:
                self.peripheral.disconnect()
        except:
            pass
        print(f"[{self.name}] Disconnected.")

# --- MAIN PROGRAM ---
left_glove = BlunoBeetle("B4:99:4C:89:18:4E", "Left Hand")
right_glove = BlunoBeetle("34:08:E1:28:0A:FF", "Right Hand")

left_glove.start()
right_glove.start()

try:
    while True:
        time.sleep(1)  # Keep main thread running
except KeyboardInterrupt:
    print("\nStopping threads...")
    left_glove.stop()
    right_glove.stop()
    left_glove.join()
    right_glove.join()
    print("All threads stopped.")
