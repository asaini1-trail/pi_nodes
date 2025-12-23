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

        # Connect to pigpio daemon with retry logic
        self.pi = None
        self._connect_to_pigpiod()

        # LED registry (add more LEDs easily)
        self.led_channels = {
            "main_led": 18,  # GPIO18 has hardware PWM
            "headlamp": 5,   # GPIO5 for headlamp PWM
            # "red_led": 12,
            # "green_led": 13,
        }

        # PWM pulse width range in microseconds (500-2500 μs)
        self.pulsewidth_min = 500  # microseconds
        self.pulsewidth_max = 2500  # microseconds
        
        # Error tracking
        self.consecutive_errors = 0
        self.last_error_log_time = 0.0
        self.error_log_interval = 5.0  # seconds between error logs
        self.last_clamp_warning_time = 0.0
        self.clamp_warning_interval = 5.0  # seconds between clamp warnings

        # Initialize all LED pins
        for led_name, pin in self.led_channels.items():
            try:
                self.pi.set_mode(pin, pigpio.OUTPUT)
                # Set initial pulse width to minimum (LED off)
                self.pi.set_servo_pulsewidth(pin, self.pulsewidth_min)
                self.get_logger().info(f"Initialized {led_name} on GPIO{pin} (PWM range: {self.pulsewidth_min}-{self.pulsewidth_max} μs)")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize {led_name} on GPIO{pin}: {e}")
                raise

        # Subscribe to /led_state
        self.subscription = self.create_subscription(
            Int32,
            'led_state',
            self.led_callback,
            10
        )

        # Subscribe to /headlamp_pwm
        self.headlamp_subscription = self.create_subscription(
            Int32,
            'headlamp_pwm',
            self.headlamp_callback,
            10
        )

        self.get_logger().info("LED PWM Controller initialized on Raspberry Pi.")
        self.get_logger().info(f"Subscribed to /led_state (PWM pulse width range: {self.pulsewidth_min}-{self.pulsewidth_max} microseconds)")
        self.get_logger().info(f"Subscribed to /headlamp_pwm for GPIO5 (PWM pulse width range: {self.pulsewidth_min}-{self.pulsewidth_max} microseconds)")

    def _connect_to_pigpiod(self, max_retries=10, retry_delay=1.0):
        """Connect to pigpio daemon with retry logic"""
        self.get_logger().info("Connecting to pigpiod daemon...")
        
        for attempt in range(max_retries):
            self.pi = pigpio.pi()
            if self.pi.connected:
                self.get_logger().info("Successfully connected to pigpiod daemon")
                return
            
            # Not connected, wait and retry
            if attempt < max_retries - 1:
                self.get_logger().warn(
                    f"Failed to connect to pigpiod (attempt {attempt + 1}/{max_retries}). "
                    f"Retrying in {retry_delay} seconds..."
                )
                time.sleep(retry_delay)
            else:
                # Last attempt failed
                self.get_logger().error("=" * 60)
                self.get_logger().error("Failed to connect to pigpiod daemon after all retries")
                self.get_logger().error("")
                self.get_logger().error("To fix this issue:")
                self.get_logger().error("1. Start the pigpiod daemon: sudo pigpiod")
                self.get_logger().error("2. If pigpiod is not installed, build from source:")
                self.get_logger().error("   cd /tmp")
                self.get_logger().error("   wget https://github.com/joan2937/pigpio/archive/master.zip")
                self.get_logger().error("   unzip master.zip")
                self.get_logger().error("   cd pigpio-master")
                self.get_logger().error("   make")
                self.get_logger().error("   sudo make install")
                self.get_logger().error("   sudo pigpiod")
                self.get_logger().error("=" * 60)
                raise RuntimeError("Failed to connect to pigpiod daemon after all retries")

    def _check_connection(self):
        """Check if still connected to pigpio daemon"""
        if not self.pi.connected:
            current_time = time.time()
            if current_time - self.last_error_log_time > self.error_log_interval:
                self.get_logger().error("Lost connection to pigpiod daemon. Attempting to reconnect...")
                self.last_error_log_time = current_time
            # Try to reconnect
            self.pi.stop()
            time.sleep(0.1)
            self.pi = pigpio.pi()
            if not self.pi.connected:
                return False
            # Reinitialize pins after reconnection
            for led_name, pin in self.led_channels.items():
                try:
                    self.pi.set_mode(pin, pigpio.OUTPUT)
                    self.pi.set_servo_pulsewidth(pin, self.pulsewidth_min)
                except Exception as e:
                    self.get_logger().error(f"Failed to reinitialize {led_name} on GPIO{pin}: {e}")
                    return False
            self.get_logger().info("Reconnected to pigpiod daemon")
        return True

    def led_callback(self, msg):
        pulsewidth = msg.data  # expect 500-2500 microseconds
        original_pulsewidth = pulsewidth
        
        # Clamp pulse width to valid range (500-2500 microseconds)
        current_time = time.time()
        if pulsewidth < self.pulsewidth_min:
            pulsewidth = self.pulsewidth_min
            if current_time - self.last_clamp_warning_time > self.clamp_warning_interval:
                self.get_logger().warn(f"Clamped pulse width from {original_pulsewidth} to {self.pulsewidth_min} μs (minimum)")
                self.last_clamp_warning_time = current_time
        elif pulsewidth > self.pulsewidth_max:
            pulsewidth = self.pulsewidth_max
            if current_time - self.last_clamp_warning_time > self.clamp_warning_interval:
                self.get_logger().warn(f"Clamped pulse width from {original_pulsewidth} to {self.pulsewidth_max} μs (maximum)")
                self.last_clamp_warning_time = current_time
        
        # Check connection before setting PWM
        if not self._check_connection():
            self._handle_error("Cannot set PWM: not connected to pigpiod")
            return

        # Set PWM pulse width for all LEDs (in microseconds)
        try:
            for led_name, pin in self.led_channels.items():
                self.pi.set_servo_pulsewidth(pin, pulsewidth)
            
            # Reset error counter on successful update
            self.consecutive_errors = 0
            if original_pulsewidth != pulsewidth:
                self.get_logger().debug(f"Updated PWM pulse width to {pulsewidth} μs (clamped from {original_pulsewidth} μs) for all LEDs.")
            else:
                self.get_logger().debug(f"Updated PWM pulse width to {pulsewidth} μs for all LEDs.")
        except Exception as e:
            self._handle_error(f"Failed to set PWM pulse width {pulsewidth} μs: {e}")

    def headlamp_callback(self, msg):
        """Callback for headlamp_pwm topic - controls GPIO 5"""
        pulsewidth = msg.data  # expect 500-2500 microseconds
        original_pulsewidth = pulsewidth
        headlamp_pin = self.led_channels.get("headlamp")
        
        if headlamp_pin is None:
            self.get_logger().error("Headlamp GPIO pin not configured!")
            return
        
        # Clamp pulse width to valid range (500-2500 microseconds)
        current_time = time.time()
        if pulsewidth < self.pulsewidth_min:
            pulsewidth = self.pulsewidth_min
            if current_time - self.last_clamp_warning_time > self.clamp_warning_interval:
                self.get_logger().warn(f"Clamped headlamp pulse width from {original_pulsewidth} to {self.pulsewidth_min} μs (minimum)")
                self.last_clamp_warning_time = current_time
        elif pulsewidth > self.pulsewidth_max:
            pulsewidth = self.pulsewidth_max
            if current_time - self.last_clamp_warning_time > self.clamp_warning_interval:
                self.get_logger().warn(f"Clamped headlamp pulse width from {original_pulsewidth} to {self.pulsewidth_max} μs (maximum)")
                self.last_clamp_warning_time = current_time
        
        # Check connection before setting PWM
        if not self._check_connection():
            self._handle_error("Cannot set headlamp PWM: not connected to pigpiod")
            return

        # Set PWM pulse width for headlamp on GPIO 5 (in microseconds)
        try:
            self.pi.set_servo_pulsewidth(headlamp_pin, pulsewidth)
            
            # Reset error counter on successful update
            self.consecutive_errors = 0
            if original_pulsewidth != pulsewidth:
                self.get_logger().debug(f"Updated headlamp PWM pulse width to {pulsewidth} μs (clamped from {original_pulsewidth} μs) on GPIO{headlamp_pin}.")
            else:
                self.get_logger().debug(f"Updated headlamp PWM pulse width to {pulsewidth} μs on GPIO{headlamp_pin}.")
        except Exception as e:
            self._handle_error(f"Failed to set headlamp PWM pulse width {pulsewidth} μs on GPIO{headlamp_pin}: {e}")

    def _handle_error(self, error_msg):
        """Handle errors with throttled logging"""
        self.consecutive_errors += 1
        current_time = time.time()
        
        # Only log errors periodically to avoid spam
        if current_time - self.last_error_log_time > self.error_log_interval:
            self.get_logger().warn(f'{error_msg} (consecutive errors: {self.consecutive_errors})')
            self.last_error_log_time = current_time

    def destroy_node(self):
        # Cleanup all LEDs (set to minimum pulse width = off)
        self.get_logger().info("Shutting down LED PWM Controller...")
        for led_name, pin in self.led_channels.items():
            try:
                self.pi.set_servo_pulsewidth(pin, self.pulsewidth_min)
                self.get_logger().debug(f"Turned off {led_name} on GPIO{pin} (set to {self.pulsewidth_min} μs)")
            except Exception as e:
                self.get_logger().warn(f"Error turning off {led_name}: {e}")
        
        try:
            self.pi.stop()
        except Exception as e:
            self.get_logger().warn(f"Error stopping pigpio connection: {e}")
        
        super().destroy_node()


def main(args=None):
    # Prevent multiple instances from running simultaneously
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
        # Release lock
        if lock_file:
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                lock_file.close()
                os.remove(lock_file_path)
            except:
                pass


if __name__ == '__main__':
    main()