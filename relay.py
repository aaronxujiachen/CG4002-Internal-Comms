from bluepy import btle
import struct
import time

PACKET_SIZE = 20  # Each packet should be 20 bytes (based on the Arduino format)

class BlunoDelegate(btle.DefaultDelegate):
    """Handles incoming BLE notifications from Bluno Beetle."""
    def __init__(self, device_name):
        btle.DefaultDelegate.__init__(self)
        self.device_name = device_name

    def handleNotification(self, cHandle, data):
        """Processes received BLE notifications (20-byte packets)."""
        if len(data) == PACKET_SIZE:
            packet = self.decode_packet(data)
            print(f"[{self.device_name}] Received Packet: {packet}")
        else:
            print(f"[{self.device_name}] Warning: Received unexpected packet size ({len(data)} bytes)")

    def decode_packet(self, data):
        """Decodes a 20-byte packet received from Bluno Beetle."""
        try:
            packet_type = chr(data[0])  # First byte = Packet type (char)
            device_id = chr(data[1])  # Second byte = Device ID (char)
            imu_data = struct.unpack("<hhhhhh", data[2:14])  # 6 int16_t values (Little Endian)
            padding = data[14:19]  # 5-byte padding (unused)
            received_crc = data[19]  # Last byte = CRC-8 checksum

            return {
                "packet_type": packet_type,
                "device_id": device_id,
                "imu_data": imu_data,
                "padding": padding.hex(),
                "checksum": received_crc
            }
        except Exception as e:
            return f"Error decoding packet: {e}"

class BlunoBeetle:
    """Handles BLE connection, handshake, and communication with a Bluno Beetle."""
    def __init__(self, mac_address, name="Bluno"):
        self.mac_address = mac_address
        self.name = name
        self.peripheral = btle.Peripheral()
        self.tx_characteristic = None  # TX characteristic for sending data

    def connect(self):
        """Establishes a BLE connection with the Bluno Beetle."""
        print(f"[{self.name}] Connecting to {self.mac_address}...")
        self.peripheral.connect(self.mac_address)

        # Set up delegate for handling notifications
        self.delegate = BlunoDelegate(self.name)
        self.peripheral.withDelegate(self.delegate)

        # Get the correct BLE characteristic (Bluno uses dfb0 -> dfb1)
        self.tx_characteristic = self.peripheral.getServiceByUUID('dfb0').getCharacteristics('dfb1')[0]

        print(f"[{self.name}] Connected successfully.")
        return True

    def handshake(self):
        """Performs a three-way handshake before data transmission."""
        if self.tx_characteristic is None:
            print(f"[{self.name}] ERROR: Not connected.")
            return False

        print(f"[{self.name}] Starting Three-Way Handshake...")

        # Step 1: Send SYN packet
        print(f"[{self.name}] -> Sending SYN")
        self.tx_characteristic.write(b"SYN")
        time.sleep(1)

        # Step 2: Wait for SYN-ACK response
        if self.peripheral.waitForNotifications(2.0):
            print(f"[{self.name}] -> Received SYN-ACK")

            # Step 3: Send ACK to confirm handshake
            print(f"[{self.name}] -> Sending ACK")
            self.tx_characteristic.write(b"ACK")
            time.sleep(1)

            print(f"[{self.name}] [+] Handshake Complete!")
            return True
        else:
            print(f"[{self.name}] [!] Handshake Failed!")
            return False

    def receive_data(self, timeout=2.0):
        """Waits for incoming data from the Bluno Beetle."""
        if self.peripheral.waitForNotifications(timeout):
            return True
        return False

    def disconnect(self):
        """Disconnects from the Bluno Beetle."""
        if self.peripheral:
            self.peripheral.disconnect()
            print(f"[{self.name}] Disconnected.")

# --- MAIN PROGRAM ---
beetle = BlunoBeetle("B4:99:4C:89:18:4E")

if beetle.connect():
    if beetle.handshake():  # Perform handshake before receiving data
        while True:
            beetle.receive_data()  # Continuously receive packets
    else:
        print("[!] Handshake failed. Exiting...")
else:
    print("[!] Connection failed. Exiting...")
