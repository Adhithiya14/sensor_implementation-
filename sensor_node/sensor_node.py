import rclpy
from rclpy.node import Node
import socket
import struct


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

       
        self.declare_parameter('interval', 1000) 

       
        self.sensor_ip = '172.20.10.2'  # Change to actual sensor IP if different
        self.sensor_port = 2000
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.sensor_ip, self.sensor_port))

        
        self.voltage_pub = self.create_publisher(int, 'supply_voltage', 10)
        self.temp_pub = self.create_publisher(int, 'env_temp', 10)
        self.yaw_pub = self.create_publisher(int, 'yaw', 10)
        self.pitch_pub = self.create_publisher(int, 'pitch', 10)
        self.roll_pub = self.create_publisher(int, 'roll', 10)

        
        interval = self.get_parameter('interval').value
        self.timer = self.create_timer(interval / 1000.0, self.send_start_command)

       
        self.create_subscription(int, 'start_sensor', self.handle_start_command, 10)
        self.create_subscription(int, 'stop_sensor', self.handle_stop_command, 10)

       
        self.create_timer(0.1, self.receive_message)

    def send_start_command(self):
        
        command_id = '03'
        interval = self.get_parameter('interval').value
        interval_hex = struct.pack('<H', interval).hex()
        message = f"#{command_id}{interval_hex}\r\n"
        self.socket.sendall(message.encode())
        self.get_logger().info(f"Sent start command with interval: {interval} ms")

    def handle_start_command(self, msg):
        interval = msg.data
        self.set_parameter(rclpy.parameter.Parameter('interval', rclpy.Parameter.Type.INTEGER, interval))
        self.get_logger().info(f"Received new interval: {interval} ms")
        self.send_start_command()

    def handle_stop_command(self, msg):
        stop_message = "#09\r\n"
        self.socket.sendall(stop_message.encode())
        self.get_logger().info("Sent stop command")
        self.timer.cancel()  

    def receive_message(self):
        try:
            response = self.socket.recv(1024).decode()
            if response.startswith('$11'):
                self.handle_status_message(response)
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")

    def handle_status_message(self, response):
        
        payload = response[3:-2]  
        try:
            supply_voltage = int(payload[0:4], 16)
            env_temp = int(payload[4:8], 16)
            yaw = int(payload[8:12], 16)
            pitch = int(payload[12:16], 16)
            roll = int(payload[16:20], 16)

            self.voltage_pub.publish(supply_voltage)
            self.temp_pub.publish(env_temp)
            self.yaw_pub.publish(yaw)
            self.pitch_pub.publish(pitch)
            self.roll_pub.publish(roll)

            self.get_logger().info(f"Status: Voltage={supply_voltage}, Temp={env_temp}, "
                                   f"Yaw={yaw}, Pitch={pitch}, Roll={roll}")
        except ValueError as e:
            self.get_logger().error(f"Error decoding message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
