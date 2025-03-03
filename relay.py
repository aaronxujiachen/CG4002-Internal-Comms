from bluepy import btle
import struct
import time
from crc import Calculator, Configuration

PACKET_SIZE = 20  # Each packet should be 20 bytes

class Packet:
    PACKET_FORMAT = "1s1s12s5sB"  # PacketType (1 byte), DeviceID (1 byte), SensorData (12 bytes), Padding (5 bytes), Checksum (1 byte)
    PACKET_SIZE = 20

    crc_config = Configuration(
        width=8,               # 8-bit CRC
        polynomial=0x07,        # Polynomial x^8 + x^2 + x + 1 (0x07)
        init_value=0x00,        # Initial value (0x00 for SMBus)
        final_xor_value=0x00,   # No final XOR operation
        reverse_input=False,    # No bit-reversal on input
        reverse_output=False    # No bit-reversal on output
    )

    crc_calculator = Calculator(crc_config)

    def __init__(self, packet_type: str, device_id: str, sensor_data: bytes = b""):
        """Initializes a structured packet."""
        if len(packet_type) != 1 or len(device_id) != 1:
            raise ValueError("Packet Type and Device ID must be exactly 1 character each.")

        if len(sensor_data) > 12:
            raise ValueError("Sensor data must be at most 12 bytes.")

        self.packet_type = packet_type.encode()  # Convert to bytes (1 byte)
        self.device_id = device_id.encode()  # Convert to bytes (1 byte)
        self.sensor_data = sensor_data.ljust(12, b'\x00')  # Fill up sensor data to 12 bytes
        self.padding = b'\x00' * 5  # Fill up 5 bytes to make the whole packet 20 bytes 
        self.checksum = self.calculate_checksum()

    def calculate_checksum(self) -> bytes:
        """Computes CRC-8 checksum using the SMBus polynomial (0x07)."""
        data = self.packet_type + self.device_id + self.sensor_data + self.padding
        checksum_value = self.crc_calculator.checksum(data)  # Compute CRC-8 checksum
        return checksum_value.to_bytes(1, 'big')  # Convert checksum to 1 byte

    def pack(self) -> bytes:
        """Packs the packet into a 20-byte binary format."""
        return struct.pack(
            self.PACKET_FORMAT, 
            self.packet_type, 
            self.device_id, 
            self.sensor_data, 
            self.padding, 
            int.from_bytes(self.checksum, "big")  # Convert checksum from bytes to integer
        )

    @classmethod
    def unpack(cls, packet_bytes: bytes):
        """Unpacks a 20-byte packet and creates a Packet instance."""
        if len(packet_bytes) != cls.PACKET_SIZE:
            raise ValueError("Invalid packet size. Expected 20 bytes.")

        packet_type, device_id, sensor_data, padding, checksum = struct.unpack(cls.PACKET_FORMAT, packet_bytes)

        # Validate checksum
        data = packet_type + device_id + sensor_data + padding
        expected_checksum = cls.crc_calculator.checksum(data)

        if checksum != expected_checksum:
            raise ValueError("Checksum mismatch. Packet might be corrupted.")

        return cls(packet_type.decode().strip('\x00'), device_id.decode().strip('\x00'), sensor_data.rstrip(b'\x00'))

    def __repr__(self):
        return f"Packet(Type={self.packet_type.decode()}, DeviceID={self.device_id.decode()}, SensorData={self.sensor_data}, Checksum={self.checksum.hex()})"

class BlunoDelegate(btle.DefaultDelegate):
    """Handles incoming BLE notifications and validates handshake messages."""
    def __init__(self, device_name):
        super().__init__()
        self.device_name = device_name
        self.received_packet = None
        self.ack_received = False  # Track if ACK is received
        self.buffer = b''

    def handleNotification(self, cHandle, data):
        """Processes incoming BLE notifications (20-byte packets)."""
        try:
            self.buffer += data  # Appending data in bytes to buffer at the end
            if len(self.buffer) < 20:  # Packet is fragmented
                return
            packet = self.buffer[:20]  # Take the first 20 bytes from the buffer
            self.buffer = self.buffer[20:]  # The buffer will discard the first 20 bytes
            self.received_packet = Packet.unpack(packet)
            print(f"[{self.device_name}] Received Packet: {self.received_packet}")

            # Validate handshake messages
            packet_type = self.received_packet.packet_type.decode()
            if packet_type == "A":  # Check if ACK is received
                print(f"[{self.device_name}] ✅ ACK Packet received. Handshake successful.")
                self.ack_received = True
            elif packet_type not in ["S", "Y", "A"]:
                print(f"[{self.device_name}] ❌ Unexpected Packet Type: {packet_type} (Expected: S, Y, or A)")
        except Exception as e:
            print(f"[{self.device_name}] Error decoding packet: {e}")

class BlunoBeetle:
    """Handles BLE connection, handshake, and communication with a Bluno Beetle."""
    def __init__(self, mac_address, name="Bluno"):
        self.mac_address = mac_address
        self.name = name
        self.peripheral = btle.Peripheral()  # Sets up the BLE object without connecting yet
        self.tx_characteristic = None  # The laptop is writing to a characteristic on the Bluno Beetle, the Bluno Beetle is reading from the characteristic and responding with data

    def connect(self):
        """Establishes a BLE connection with the Bluno Beetle."""
        print(f"[{self.name}] Connecting to {self.mac_address}...")
        self.peripheral.connect(self.mac_address)  # Connects to the Bluno Beetle using MAC address

        # Set up delegate for handling notifications
        self.delegate = BlunoDelegate(self.name)
        self.peripheral.withDelegate(self.delegate)

        # Finds a specific service inside Bluno and a specific characteristic for sending data
        self.tx_characteristic = self.peripheral.getServiceByUUID('dfb0').getCharacteristics('dfb1')[0]

        print(f"[{self.name}] Connected successfully.")
        return True

    def wait_for_packet(self, expected_type, timeout=2.0):
        """
        Waits for an incoming packet with a specific type.
        :param expected_type: Expected packet type (e.g., 'Y' for SYN-ACK, 'A' for ACK).
        :param timeout: Time to wait before failing.
        :return: The received Packet object if successful, otherwise None.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.peripheral.waitForNotifications(1.0):  # Waits for Bluno to respond (e.g., ACK)
                packet = self.delegate.received_packet
                if packet and packet.packet_type.decode() == expected_type:  # If the packet exists and the packet type is correct
                    print(f"[{self.name}] -> Valid {expected_type} packet received.")
                    return packet
                else:
                    print(f"[{self.name}] [!] Unexpected packet type: {packet.packet_type.decode() if packet else 'None'}")
        return None

    def handshake(self):
        """Performs a Three-Way Handshake using structured packets with type validation."""
        if self.tx_characteristic is None:
            print(f"[{self.name}] ERROR: Not connected.")
            return False

        print(f"[{self.name}] Starting Three-Way Handshake...")

        # Step 1: Send SYN Packet (Type 'S')
        packet_syn = Packet("S", "D")
        print(f"[{self.name}] -> Sending SYN Packet: {packet_syn}")
        self.tx_characteristic.write(packet_syn.pack())  # Sends SYN packet to Bluno
        time.sleep(1)

        # Step 2: Wait for SYN-ACK response (Expected 'Y')
        packet_response = self.wait_for_packet("Y")
        if not packet_response:
            print(f"[{self.name}] [!] Handshake Failed! No SYN-ACK received.")
            return False

        # Step 3: Send ACK Packet (Type 'A')
        packet_ack = Packet("A", "D")
        print(f"[{self.name}] -> Sending ACK Packet: {packet_ack}")
        self.tx_characteristic.write(packet_ack.pack())  # Sends ACK packet to Bluno
        time.sleep(1)

        # Step 4: Verify ACK Reception
        if self.delegate.ack_received:
            print(f"[{self.name}] [+] Handshake Complete!")
            return True
        else:
            print(f"[{self.name}] [!] Handshake Failed! No ACK confirmation received.")
            return False

    def receive_data(self, timeout=2.0):
        """Waits for incoming data from the Bluno Beetle."""
        return self.peripheral.waitForNotifications(timeout)

    def disconnect(self):
        """Disconnects from the Bluno Beetle."""
        if self.peripheral:
            self.peripheral.disconnect()
            print(f"[{self.name}] Disconnected.")

# --- MAIN PROGRAM ---
while True:
    try:
        beetle = BlunoBeetle("34:08:e1:2a:29:16", "Right Hand")  # do the same for the remaining beetls

        if beetle.connect():
            if beetle.handshake():  # Perform handshake before receiving data
                while True:
                    beetle.receive_data()  # Continuously receive packets
            else:
                print("[!] Handshake failed. Exiting...")
        else:
            print("[!] Connection failed. Exiting...")
    except Exception as e:
        print(e)
