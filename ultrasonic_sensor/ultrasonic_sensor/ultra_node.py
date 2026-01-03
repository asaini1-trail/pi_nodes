import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import fcntl
import os
import sys
from std_msgs.msg import Float32


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.publisher = self.create_publisher(Float32, '/front_distance', 10)

        self.trig = 23
        self.echo = 24
        
        # Distance validation parameters (HC-SR04 typical range: 2-400 cm)
        # Lowered min to 1.5 cm to account for sensor noise near minimum range
        self.min_distance = 1.5  # cm
        self.max_distance = 400.0  # cm
        
        # Error tracking
        self.consecutive_errors = 0
        self.last_error_log_time = 0.0
        self.error_log_interval = 5.0  # seconds between error logs
        self.last_diagnostic_time = 0.0
        self.diagnostic_interval = 10.0  # seconds between diagnostics
        self.recovery_threshold = 50  # Attempt recovery after this many errors
        self.last_recovery_attempt = 0.0
        self.recovery_cooldown = 30.0  # seconds between recovery attempts
        self.last_stuck_high_warning = 0.0
        self.stuck_high_warning_interval = 2.0  # seconds between stuck HIGH warnings

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.5)  # allow sensor to settle
        
        # Check initial echo state
        initial_echo = GPIO.input(self.echo)
        self.get_logger().info(f'Initial echo pin state: {initial_echo} (0=LOW, 1=HIGH)')
        self.get_logger().info(f'Using GPIO pins - TRIG: {self.trig}, ECHO: {self.echo}')

        self.timer = self.create_timer(0.1, self.measure_distance)  # 10 Hz

    def _reset_sensor(self):
        """Attempt to reset/recover the sensor by reinitializing GPIO pins"""
        self.get_logger().info('Attempting sensor recovery - resetting GPIO pins...')
        try:
            # Clean up current setup
            GPIO.cleanup()
            time.sleep(0.1)
            
            # Reinitialize
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)
            
            # Reset trigger to LOW
            GPIO.output(self.trig, False)
            
            # Wait for sensor to settle
            time.sleep(0.5)
            
            # Try a few trigger pulses to "wake up" the sensor
            for _ in range(3):
                GPIO.output(self.trig, True)
                time.sleep(0.00002)
                GPIO.output(self.trig, False)
                time.sleep(0.01)
            
            # Check echo state after reset
            echo_state = GPIO.input(self.echo)
            self.get_logger().info(f'Sensor reset complete. Echo pin state: {echo_state}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error during sensor reset: {e}')
            return False

    def measure_distance(self):
        # Attempt recovery if too many consecutive errors
        if self.consecutive_errors >= self.recovery_threshold:
            current_time = time.time()
            if current_time - self.last_recovery_attempt > self.recovery_cooldown:
                self.last_recovery_attempt = current_time
                if self._reset_sensor():
                    self.consecutive_errors = 0  # Reset counter if recovery successful
                    self.get_logger().info('Sensor recovery attempted, resuming measurements...')
                else:
                    self.get_logger().warn('Sensor recovery failed')
        
        # Check if echo is stuck HIGH (common failure mode)
        echo_before = GPIO.input(self.echo)
        if echo_before == 1:
            # Echo is stuck HIGH - try to clear it
            GPIO.output(self.trig, False)
            time.sleep(0.00001)
            
            # Send multiple trigger pulses to try to reset the sensor
            for _ in range(5):
                GPIO.output(self.trig, True)
                time.sleep(0.00002)
                GPIO.output(self.trig, False)
                time.sleep(0.001)
                if GPIO.input(self.echo) == 0:
                    break
            
            # Wait for echo to clear (up to 200ms)
            clear_timeout = 0.2
            clear_start = time.time()
            while GPIO.input(self.echo) == 1:
                if time.time() - clear_start > clear_timeout:
                    # Still stuck after timeout - publish 0 and skip this measurement
                    current_time = time.time()
                    if current_time - self.last_stuck_high_warning > self.stuck_high_warning_interval:
                        self.get_logger().warn(
                            'ECHO pin stuck HIGH - sensor may be faulty, disconnected, or needs power cycle. Check wiring: TRIG=23, ECHO=24, VCC=5V, GND'
                        )
                        self.last_stuck_high_warning = current_time
                    self.consecutive_errors += 1
                    msg = Float32()
                    msg.data = 10000.0
                    self.publisher.publish(msg)
                    time.sleep(0.1)  # Longer delay when stuck
                    return
                time.sleep(0.001)
            
            # Echo cleared, wait before next measurement
            time.sleep(0.02)
        # Ensure trigger is LOW first
        GPIO.output(self.trig, False)
        time.sleep(0.00002)  # 20 microseconds
        
        # Send trigger pulse - HC-SR04 needs at least 10us, use 15us for safety
        GPIO.output(self.trig, True)
        time.sleep(0.000015)  # 15 microseconds
        GPIO.output(self.trig, False)
        
        # Wait a bit for echo to start (sensor needs time to process)
        time.sleep(0.00002)  # 20 microseconds

        timeout = 0.05  # 50 ms timeout for echo response
        timeout_start = time.time()
        initial_echo_state = GPIO.input(self.echo)

        # Wait for echo to go HIGH (with timeout)
        if initial_echo_state == 0:
            while GPIO.input(self.echo) == 0:
                if time.time() - timeout_start > timeout:
                    self._handle_error_with_diagnostics(
                        f'Timeout waiting for echo to go HIGH. Echo was {initial_echo_state} after trigger'
                    )
                    msg = Float32()
                    msg.data = 10000.0
                    self.publisher.publish(msg)
                    time.sleep(0.05)
                    return
        else:
            # Echo is already HIGH - this shouldn't happen if stuck HIGH check worked
            # But if it does, wait a bit and check again
            time.sleep(0.001)
            if GPIO.input(self.echo) == 1:
                # Still HIGH, publish 10000 and skip this measurement
                self.consecutive_errors += 1
                msg = Float32()
                msg.data = 10000.0
                self.publisher.publish(msg)
                time.sleep(0.05)
                return
        
        # Record when echo goes HIGH
        start = time.time()
        timeout_start = time.time()

        # Wait for echo to go LOW (with timeout)
        while GPIO.input(self.echo) == 1:
            if time.time() - timeout_start > timeout:
                self._handle_error('Timeout waiting for echo to go LOW')
                msg = Float32()
                msg.data = 10000.0
                self.publisher.publish(msg)
                time.sleep(0.05)
                return
        
        stop = time.time()

        elapsed = stop - start
        
        # Calculate distance in cm
        distance = (elapsed * 34300) / 2
        
        # Validate distance range
        if distance < self.min_distance or distance > self.max_distance:
            # Only log invalid readings occasionally to reduce spam
            if self.consecutive_errors % 20 == 0:
                self._handle_error(f'Invalid distance reading: {distance:.2f} cm (out of range {self.min_distance}-{self.max_distance} cm)')
            else:
                self.consecutive_errors += 1
            msg = Float32()
            msg.data = 10000.0
            self.publisher.publish(msg)
            time.sleep(0.01)  # Small delay after invalid reading
            return
        
        # Valid reading - reset error counter and publish
        self.consecutive_errors = 0
        
        msg = Float32()
        msg.data = distance
        self.publisher.publish(msg)
    
    def _handle_error(self, error_msg):
        """Handle errors with throttled logging"""
        self.consecutive_errors += 1
        current_time = time.time()
        
        # Only log errors periodically to avoid spam
        if current_time - self.last_error_log_time > self.error_log_interval:
            self.get_logger().warn(f'{error_msg} (consecutive errors: {self.consecutive_errors})')
            self.last_error_log_time = current_time
    
    def _handle_error_with_diagnostics(self, error_msg):
        """Handle errors with additional diagnostic information"""
        self.consecutive_errors += 1
        current_time = time.time()
        
        # Log diagnostics less frequently
        if current_time - self.last_diagnostic_time > self.diagnostic_interval:
            echo_state = GPIO.input(self.echo)
            self.get_logger().warn(
                f'{error_msg} | Current ECHO state: {echo_state} (0=LOW, 1=HIGH) | '
                f'Consecutive errors: {self.consecutive_errors} | '
                f'Check: 1) Sensor power (5V), 2) GND connection, 3) GPIO pins TRIG={self.trig}/ECHO={self.echo}'
            )
            self.last_diagnostic_time = current_time
            self.last_error_log_time = current_time
        elif current_time - self.last_error_log_time > self.error_log_interval:
            self.get_logger().warn(f'{error_msg} (consecutive errors: {self.consecutive_errors})')
            self.last_error_log_time = current_time


def main(args=None):
    # Prevent multiple instances from running simultaneously
    lock_file_path = '/tmp/ultra_node.lock'
    lock_file = None
    try:
        lock_file = open(lock_file_path, 'w')
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock_file.write(str(os.getpid()))
        lock_file.flush()
    except (IOError, OSError):
        print(f'ERROR: Another instance of ultra_node is already running!', file=sys.stderr)
        print(f'If you are sure no other instance is running, delete: {lock_file_path}', file=sys.stderr)
        sys.exit(1)
    
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()
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