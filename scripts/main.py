#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, MagneticField
from mlx90640_msgs.msg import ThermalStatus
import serial
import struct
import sys
import numpy as np

MICROTESLA_TO_TESLA = 1e-6

def parse_thermal_header(payload: bytes):
    """ Unpacks the 14-byte header and returns the quantized array """
    # <IffBB = uint32 (4), float32 (4), float32 (4), uint8 (1), uint8 (1)
    seq, min_c, max_c, cols, rows = struct.unpack_from('<IffBB', payload, 0)
    q = payload[14:]
    return seq, min_c, max_c, cols, rows, q

class ESP32SensorBridge(Node):
    def __init__(self):
        super().__init__('esp32_sensor_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self._mag_frame_id = "sensor_link"
        self._thermal_frame_id = "thermal_link"

        self._pub_mag = self.create_publisher(MagneticField, '/sensors/mag', 10)
        self._pub_thermal = self.create_publisher(Image, '/sensors/thermal', 10)
        self._pub_thermal_raw = self.create_publisher(Image, '/sensors/thermal_raw', 10)
        self._pub_thermal_status = self.create_publisher(ThermalStatus, '/sensors/thermal_status', 10)

        self._thermal_last_t = None
        self._thermal_rate_hz = 0.0

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Serial: {e}")
            sys.exit(1)

        # Polling timer for reading serial bytes
        self.timer = self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        # We need at least 5 bytes for our header: [0xAA] [0xBB] [Type] [Len_L] [Len_H]
        while self.ser.in_waiting >= 5:
            # Look for sync bytes
            sync = self.ser.read(2)
            if sync != b'\xAA\xBB':
                continue
            
            msg_type = self.ser.read(1)[0]
            length_bytes = self.ser.read(2)
            length = struct.unpack('<H', length_bytes)[0]
            
            # Read the exact payload length
            payload = self.ser.read(length)
            
            if len(payload) == length:
                if msg_type == 1:
                    self._handle_mag(payload)
                elif msg_type == 2:
                    self._handle_thermal(payload)
            else:
                self.get_logger().warn("Incomplete payload received.")

    def _handle_mag(self, payload: bytes):
        if len(payload) < 6:
            return
        x_ut, y_ut, z_ut = struct.unpack_from('<hhh', payload)
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._mag_frame_id
        msg.magnetic_field.x = x_ut * MICROTESLA_TO_TESLA
        msg.magnetic_field.y = y_ut * MICROTESLA_TO_TESLA
        msg.magnetic_field.z = z_ut * MICROTESLA_TO_TESLA
        self._pub_mag.publish(msg)

    def _handle_thermal(self, payload: bytes):
        seq, min_c, max_c, cols, rows, q = parse_thermal_header(payload)
        
        # Dequantise the 8-bit frame back to °C: min + (q/255)·(max − min).
        span = max_c - min_c
        celsius = (min_c + np.frombuffer(q, dtype=np.uint8).astype(np.float32)
                   * (span / 255.0)).reshape(rows, cols)

        now = self.get_clock().now()
        stamp = now.to_msg()
        
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self._thermal_frame_id
        img.height = rows
        img.width = cols
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = cols * 4
        img.data = np.ascontiguousarray(celsius, dtype='<f4').tobytes()
        self._pub_thermal.publish(img)
        self._pub_thermal_raw.publish(img)

        now_s = now.nanoseconds * 1e-9
        if self._thermal_last_t is not None:
            dt = now_s - self._thermal_last_t
            if dt > 0.0:
                self._thermal_rate_hz = 1.0 / dt
        self._thermal_last_t = now_s

        hotspot = int(np.argmax(celsius))
        st = ThermalStatus()
        st.header.stamp = stamp
        st.header.frame_id = self._thermal_frame_id
        st.sequence = int(seq)
        st.sensor_ok = True
        st.min_temperature = float(min_c)
        st.max_temperature = float(max_c)
        st.center_temperature = float(celsius[rows // 2, cols // 2])
        st.hotspot_x = hotspot % cols
        st.hotspot_y = hotspot // cols
        st.total_read_errors = 0
        st.consecutive_read_errors = 0
        st.acquisition_rate_hz = float(self._thermal_rate_hz)
        self._pub_thermal_status.publish(st)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()