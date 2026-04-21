#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter, ParameterType
from rcl_interfaces.msg import SetParametersResult
# from hardware_control.mks import MKS
import numpy as np
import can
import time
from can.bus import BusState

class MKS():
    def __init__(self):
        try:
            self.bus = can.interface.Bus(
                interface='slcan', 
                channel='/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209C36A83945-if00', 
                bitrate=500000)
            
            try:
                self.bus.state = BusState.PASSIVE
            except NotImplementedError:
                pass
            
            print("Connected to CAN bus via Openlight CANable2")
        except can.CanError as e:
            print(f"Failed to connect to CAN bus: {e}")

    def send_speed(self, canID, speed, acc):
        if speed < 0:
            dir = 0x80 # CW
        else:
            dir = 0x00 # CCW
        speed = abs(speed)
        byte1 = 0xF6
        speed_high = (speed >> 8) & 0x0F
        byte2 = dir | speed_high
        byte3 = speed & 0xFF
        byte4 = max(0, min(acc, 255))  
        data = [canID, byte1, byte2, byte3, byte4]
        self.send_command(canID, data[1:])

    def read_speed(self, canID):
        byte1 = 0x32
        self.send_command(canID, [byte1])

        timeout = 0.005
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(0.001)
            if msg is not None and msg.arbitration_id == canID and msg.data[0] == byte1:
                rmp = int.from_bytes(msg.data[1:3], byteorder='big', signed=True)
                return rmp
        return None

    def read_position(self, canID):
        byte1 = 0x30
        self.send_command(canID,[byte1])
        
        timeout = 0.005
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(0.001)
            if msg is not None and msg.arbitration_id == canID and msg.data[0] == byte1:
                carry = int.from_bytes(msg.data[1:5], byteorder='big', signed=True)
                encoder = int.from_bytes(msg.data[5:7], byteorder='big', signed=False)
                return carry, encoder
        return None

    def stop_motor(self, canID):
        byte1 = 0xF7
        self.send_command(canID, [byte1])

    def reset_motor(self, canID):
        byte1 = 0x41   
        self.send_command(canID, [byte1])

    def send_command(self,canID, data):
        crc = (canID + sum(data)) & 0xFF
        full_data = data + [crc]

        msg = can.Message(
            arbitration_id=canID,
            data=full_data,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Communication Error: {e}")

    def go_home(self, canID):
        pass


class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        freq = 10.0
        self.declare_parameter('acc', 5)
        self.acc = self.get_parameter('acc').get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self.param_callback)

        self.canID = [0x01, 0x02, 0x04, 0x05, 0x06]
        self.gear_ratio = np.array([13.5, 150, 48, 67.82, 67.82, 150])
        self.mks = MKS()

        self.create_subscription(JointState, 'joint_cmd', self.joint_state_callback, 10)
        self.feedback_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0/freq, self.publish_feedback)
        self.create_service(Trigger, 'stop', self.stop_callback)
        self.create_service(Trigger, 'reset', self.reset_callback)

        self.get_logger().info("Motor Control Node: RUNNING...")

    def param_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == 'acc' and param.type_ == ParameterType.PARAMETER_INTEGER and param.value is not None:
                self.acc = max(0, min(int(param.value), 255))
                self.get_logger().info(f"acc set {self.acc}")
        return SetParametersResult(successful=True)

    def publish_feedback(self):
        joint_position = np.zeros(6)
        motor_position = np.zeros(np.size(joint_position))
        joint_speed = np.zeros(np.size(joint_position))
        motor_speed = np.zeros(np.size(joint_position))

        for i, id in enumerate(self.canID):
            carry, encoder = self.mks.read_position(id) or (0.0, 0.0)
            motor_position[i] = carry + encoder / 16383
            motor_speed[i] = self.mks.read_speed(id) or 0.0

        joint_position[0] = motor_position[0] / self.gear_ratio[0]
        joint_position[1] = motor_position[1] / self.gear_ratio[1]
        joint_position[3] = motor_position[2] / self.gear_ratio[2]
        joint_position[4] = 0.5 * (motor_position[3] + motor_position[4]) / self.gear_ratio[3]
        joint_position[5] = 0.5 * (motor_position[3] - motor_position[4]) / self.gear_ratio[4]
        joint_position = joint_position * (2 * np.pi)                   # convert rev to rad
        joint_position = (joint_position + np.pi) % (2 * np.pi) - np.pi # normalize to [-pi, pi]

        joint_speed[0] = motor_speed[0] / self.gear_ratio[0]
        joint_speed[1] = motor_speed[1] / self.gear_ratio[1]
        joint_speed[2] = motor_speed[2] / self.gear_ratio[2]
        joint_speed[4] = 0.5 * (motor_speed[3] + motor_speed[4]) / self.gear_ratio[3]   
        joint_speed[5] = 0.5 * (motor_speed[3] - motor_speed[4]) / self.gear_ratio[4]
        joint_speed = joint_speed / (60 / (2 * np.pi))                  # convert from rpm to rad/s

        msg = JointState()
        msg.position = joint_position.tolist()
        msg.velocity = joint_speed.tolist()
        self.feedback_publisher.publish(msg)
        # self.get_logger().info(f"{joint_position[0]:.2f} | {joint_speed[0]:.2f}")

    def stop_callback(self, request, response):
        response.success = True
        response.message = "Motors stopped"
        for id in self.canID:
            self.mks.stop_motor(id)
        return response

    def reset_callback(self, request, response):
        response.success = True
        response.message = "Motors reset"
        for id in self.canID:
            self.mks.reset_motor(id)
        return response

    def joint_state_callback(self, msg: JointState):
        motor_speed = np.zeros(5)
        motor_speed[0] = msg.velocity[0] * self.gear_ratio[0]                           # j1
        motor_speed[1] = msg.velocity[1] * self.gear_ratio[1]                           # j2
        motor_speed[2] = msg.velocity[3] * self.gear_ratio[3]                           # j4
        motor_speed[3] = 0.5 * (msg.velocity[4] + msg.velocity[5]) * self.gear_ratio[4] # j5
        motor_speed[4] = 0.5 * (msg.velocity[4] - msg.velocity[5]) * self.gear_ratio[5] # j6

        for i, id in enumerate(self.canID):
            speed = int(motor_speed[i] * 60 / (2 * np.pi))                              # convert to rpm
            speed = max(-1500, min(1500, speed))
            self.mks.send_speed(canID=id, speed=speed, acc=self.acc) 


def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
