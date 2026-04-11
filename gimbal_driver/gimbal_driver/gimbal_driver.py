import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports as ser2
import time
from std_msgs.msg import String, Int16MultiArray

#Allows user to send pitch and yaw through uart to siyu a8 mini camera.
# DO NOT TOUCH. Code is complete and used for its member fumctions only.
# Yeah.
# Generate CRC16 Lookup Table
CRC16_TAB = [0] * 256
POLY = 0x1021

def generate_crc16_table():
    """Generate the CRC16 lookup table."""
    global CRC16_TAB
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ POLY
            else:
                crc <<= 1
        CRC16_TAB[i] = crc & 0xFFFF

def calculate_crc16(data: bytes, crc_init: int = 0x0000) -> int:
    """Calculate CRC16 using a lookup table."""
    crc = crc_init
    for byte in data:
        temp = (crc >> 8) & 0xFF
        oldcrc16 = CRC16_TAB[byte ^ temp]
        crc = ((crc << 8) ^ oldcrc16) & 0xFFFF
    return crc

def build_gimbal_rotation_packet(yaw=0, pitch=0):
    """Build a packet for gimbal rotation (cmd_id 0x0E)."""
    cmd_id = 0x0E
    packet = bytes([0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, cmd_id, yaw & 0xFF, pitch & 0xFF])
    crc = calculate_crc16(packet)
    packet += crc.to_bytes(2, 'little')
    return packet

def build_heartbeat_packet():
    """Build a heartbeat packet (cmd_id 0x00)."""
    cmd_id = 0x00
    packet = bytes([0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00, cmd_id, 0x00, 0x00])
    crc = calculate_crc16(packet)
    packet += crc.to_bytes(2, 'little')
    return packet

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('gimbal_control')
        

         #ports = ser2.grep("10c4:ea60")#"Silicon Labs CP210x UART Bridge")
        ports = ser2.grep("10c4:ea60")# Silicon Labs CP210x UART Bridge


         

        device = "none"

        for p in ports:
            device = p[0]
        self.get_logger().info(f'Gimbal port {device}')




        self.declare_parameter('port', device)
        self.declare_parameter('baud', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        generate_crc16_table()
        self.get_logger().info(f"Opening {self.port} at {self.baud} baud...")

        try:
            self.serial_port = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening {self.port}: {e}")
            return
        
        # Send heartbeat before commands
        self.get_logger().info("Sending Heartbeat...")
        heartbeat = build_heartbeat_packet()
        self.serial_port.write(heartbeat)
        self.serial_port.flush()
        time.sleep(0.2)
        
        # Subscriber for gimbal commands (yaw and pitch)
        self.subscription = self.create_subscription(
            Int16MultiArray, 'gimbal_control', self.control_callback, 10)
        
        self.get_logger().info("Ready to receive gimbal control messages.")

    def control_callback(self, msg):
        """Handle incoming yaw and pitch commands."""
        if len(msg.data) != 2:
            self.get_logger().error("Invalid message format! Expected 2 values (yaw, pitch).")
            return
        
        yaw, pitch = msg.data
        yaw = max(0, min(255, yaw))
        pitch = max(0, min(255, pitch))

        self.get_logger().info(f"Sending Gimbal Yaw: {yaw} Pitch: {pitch}")
        gimbal_rot = build_gimbal_rotation_packet(yaw, pitch)
        self.serial_port.write(gimbal_rot)
        self.serial_port.flush()

        time.sleep(0.5)
        resp = self.serial_port.read(64)
        if resp:
            self.get_logger().info(f"Received Response: {resp.hex()}")
        else:
            self.get_logger().info("No Response Received")

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
