#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from pynput import keyboard

class TeleopJog(Node):
    def __init__(self):
        super().__init__('teleop_jog')

        self.num_joints = 6
        self.current_velocity = 0.0
        self.active_keys = set()
        self.positive_keys = {
            '1': 0, 
            '2': 1, 
            '3': 2, 
            '4': 3, 
            '5': 4, 
            '6': 5
        }
        self.negative_keys = {
            'q': 0, 
            'w': 1, 
            'e': 2, 
            'r': 3, 
            't': 4, 
            'y': 5
        }
        self.options_keys = {
            keyboard.Key.up: self.increase_speed,
            keyboard.Key.down: self.decrease_speed,
            keyboard.Key.enter: self.call_stop_service,
            keyboard.Key.esc: self.call_reset_service,
            keyboard.Key.alt: self.call_set_home_service
        }

        self.msg = JointState()
        self.msg.name = [f'joint_{i+1}' for i in range(self.num_joints)]
        self.msg.velocity = [0.0] * self.num_joints
        self.step = 0.05

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.print_help()

        self.joint_publisher = self.create_publisher(JointState, 'joint_cmd', 10)
        self.stop_client = self.create_client(Trigger, 'stop')
        self.reset_client = self.create_client(Trigger, 'reset')
        self.set_home_client = self.create_client(Trigger, 'set_home')
        self.create_timer(0.1, self.timer_callback)

    def increase_speed(self):
        self.current_velocity += self.step
        self.get_logger().info(f"Global Speed: {self.current_velocity:.2f}")

    def decrease_speed(self):
        self.current_velocity -= self.step
        self.current_velocity = max(0.0, self.current_velocity)
        self.get_logger().info(f"Global Speed: {self.current_velocity:.2f}")

    def print_help(self):
        self.get_logger().info(
            "\n--- Multi-Joint Teleop Jog ---\n"
            "Hold Keys [1-6]: Move Joint 1-6 Forward (+vel)\n"
            "Hold Keys [q-y]: Move Joint 1-6 Backward (-vel)\n"
            "Arrows Up/Down: Change Global Speed \n"
            "Press [Enter]: Call /stop service to stop all motors\n"
            "Press [ESC]: Call /reset service to reset all motors\n"
            "Press [Alt]: Call /set_home service to set home position\n"
            "-------------------------------"
        )

    def on_press(self, key):
        try:
            if hasattr(key, 'char'):
                char = key.char
                if char in self.positive_keys or char in self.negative_keys:
                    self.active_keys.add(char)

            if key == keyboard.Key.up:
                self.current_velocity += self.step
                self.get_logger().info(f"Speed: {self.current_velocity:.2f}")
            elif key == keyboard.Key.down:
                self.current_velocity -= self.step
                self.current_velocity = max(0.0, self.current_velocity)
                self.get_logger().info(f"Speed: {self.current_velocity:.2f}")
            elif key == keyboard.Key.enter:
                self.call_stop_service()
            elif key == keyboard.Key.esc:
                self.call_reset_service()
            elif key == keyboard.Key.alt:
                self.call_set_home_service()

        except Exception as e:
            pass

    def on_release(self, key):
        if hasattr(key, 'char'):
            char = key.char
            if char in self.positive_keys or char in self.negative_keys:
                self.active_keys.discard(char)

    def timer_callback(self):
        new_velocities = [0.0] * self.num_joints
        should_publish = False

        for char in self.active_keys:
            if char in self.positive_keys:
                idx = self.positive_keys[char]
                new_velocities[idx] += float(self.current_velocity)
                should_publish = True
            elif char in self.negative_keys:
                idx = self.negative_keys[char]
                new_velocities[idx] -= float(self.current_velocity)
                should_publish = True

        if should_publish:
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.velocity = new_velocities
            self.joint_publisher.publish(self.msg)
    
    def call_reset_service(self):
        if not self.reset_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('/reset service not available')
            return
        
        self.get_logger().warn('ESC pressed: Reset all motors...')
        req = Trigger.Request()
        future = self.reset_client.call_async(req)
        future.add_done_callback(self.reset_service_callback)

    def reset_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Reset response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    def call_stop_service(self):
        if not self.stop_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('/stop service not available')
            return
        
        self.get_logger().warn('ENTER pressed: Stop all motors...')
        req = Trigger.Request()
        future = self.stop_client.call_async(req)
        future.add_done_callback(self.stop_service_callback)

    def stop_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Stop response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    def call_set_home_service(self):
        if not self.set_home_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('/set_home service not available')
            return
        
        self.get_logger().warn('Alt pressed: Set home position...')
        req = Trigger.Request()
        future = self.set_home_client.call_async(req)
        future.add_done_callback(self.set_home_service_callback)

    def set_home_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Set Home response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
