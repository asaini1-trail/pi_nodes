#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pigpio
from std_msgs.msg import Int32
import fcntl
import os
import sys
import time


class LEDPWMController(Node):
    def __init__(self):
        super().__init__('led_pwm_controller')
        
        self.pi = None
        self._connect_to_pigpiod()
        
        self.led_channels = {
            "main_led": 5,
        }
        
        self.pulsewidth_min = 500
        self.pulsewidth_max = 2500
        
        for led_name, pin in self.led_channels.items():
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(pin, self.pulsewidth_min)
        
        self.subscription = self.create_subscription(
            Int32,
            'led_state',
            self.led_callback,
            10
        )

    def _connect_to_pigpiod(self, max_retries=10, retry_delay=1.0):
        for attempt in range(max_retries):
            self.pi = pigpio.pi()
            if self.pi.connected:
                return
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
        raise RuntimeError("Failed to connect to pigpiod daemon")

    def _check_connection(self):
        if not self.pi.connected:
            self.pi.stop()
            time.sleep(0.1)
            self.pi = pigpio.pi()
            if not self.pi.connected:
                return False
            for led_name, pin in self.led_channels.items():
                self.pi.set_mode(pin, pigpio.OUTPUT)
                self.pi.set_servo_pulsewidth(pin, self.pulsewidth_min)
        return True

    def led_callback(self, msg):
        pulsewidth = msg.data
        
        if pulsewidth < self.pulsewidth_min:
            pulsewidth = self.pulsewidth_min
        elif pulsewidth > self.pulsewidth_max:
            pulsewidth = self.pulsewidth_max
        
        if not self._check_connection():
            return

        for led_name, pin in self.led_channels.items():
            self.pi.set_servo_pulsewidth(pin, pulsewidth)

    def destroy_node(self):
        for led_name, pin in self.led_channels.items():
            try:
                self.pi.set_servo_pulsewidth(pin, self.pulsewidth_min)
            except:
                pass
        try:
            self.pi.stop()
        except:
            pass
        super().destroy_node()


def main(args=None):
    lock_file_path = '/tmp/led_pwm_controller.lock'
    lock_file = None
    try:
        lock_file = open(lock_file_path, 'w')
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock_file.write(str(os.getpid()))
        lock_file.flush()
    except (IOError, OSError):
        print(f'ERROR: Another instance of led_pwm_controller is already running!', file=sys.stderr)
        print(f'If you are sure no other instance is running, delete: {lock_file_path}', file=sys.stderr)
        sys.exit(1)
    
    rclpy.init(args=args)
    node = LEDPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if lock_file:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                lock_file.close()
                os.remove(lock_file_path)
            except:
                pass


if __name__ == '__main__':
    main()
