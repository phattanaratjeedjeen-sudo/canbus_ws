#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter, ParameterType
from rcl_interfaces.msg import SetParametersResult
from hardware_control.mks import MKS
import numpy as np
import time
import os
import json


class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        freq = 10.0
        self.declare_parameter('acc', 5)
        self.acc = self.get_parameter('acc').get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self.param_callback)

        self.canID = (0x01, 0x02, 0x04, 0x05, 0x06)
        self.gear_ratio = np.array([13.5, 150, 48, 67.82, 67.82, 150])
        self.mks = MKS()
        for id in self.canID:
            self.mks.reset_motor(id)
            time.sleep(0.1)
        
        self.storage_file = os.path.join(os.path.expanduser('~'), 'canbus_ws', 'joint_positions.json')
        self.joint_position_offset = self.load_joint_positions()
        self.joint_position = np.copy(self.joint_position_offset)

        self.create_subscription(JointState, 'joint_cmd', self.joint_state_callback, 10)
        self.feedback_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0/freq, self.publish_feedback)
        self.create_service(Trigger, 'stop', self.stop_callback)
        self.create_service(Trigger, 'reset', self.reset_callback)
        self.create_service(Trigger, 'set_home', self.set_home_callback)
        self.create_service(Trigger, 'go_home', self.go_home_callback)

        self.get_logger().info("Motor Control Node: RUNNING...")

    def param_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == 'acc' and param.type_ == ParameterType.PARAMETER_INTEGER and param.value is not None:
                self.acc = max(0, min(int(param.value), 255))
                self.get_logger().info(f"acc set {self.acc}")
        return SetParametersResult(successful=True)

    def load_joint_positions(self):
        if os.path.exists(self.storage_file):
            try:
                with open(self.storage_file, 'r') as f:
                    data = json.load(f)
                    self.get_logger().info("Loaded joint positions from storage.")
                    return np.array(data)
            except Exception as e:
                self.get_logger().error(f"Failed to load joint positions: {e}")
        return np.zeros(6)

    def save_joint_positions(self):
        try:
            with open(self.storage_file, 'w') as f:
                json.dump(self.joint_position.tolist(), f)
                self.get_logger().info("Saved joint positions to storage.")
        except Exception as e:
            self.get_logger().error(f"Failed to save joint positions: {e}")

    def publish_feedback(self):
        motor_rev = np.zeros(6)
        motor_speed = np.zeros(5)

        for i, id in enumerate(self.canID):
            carry, encoder = self.mks.read_position(id) or (0.0, 0.0)
            motor_rev[i] = carry + encoder / 16383
            motor_speed[i] = self.mks.read_speed(id) or 0.0

        joint_delta = np.zeros(6)
        joint_delta[0] = motor_rev[0] / self.gear_ratio[0] 
        joint_delta[1] = motor_rev[1] / self.gear_ratio[1] 
        joint_delta[3] = motor_rev[2] / self.gear_ratio[2] 
        joint_delta[4] = 0.5 * (motor_rev[3] + motor_rev[4]) / self.gear_ratio[3]
        joint_delta[5] = 0.5 * (motor_rev[3] - motor_rev[4]) / self.gear_ratio[4]
        joint_delta_rad = joint_delta * 2 * np.pi
        self.joint_position = self.joint_position_offset + joint_delta_rad
        self.joint_position = (self.joint_position + np.pi) % (2 * np.pi) - np.pi # normalize to [-pi, pi]

        joint_speed = np.zeros(6)
        joint_speed[0] = motor_speed[0] / self.gear_ratio[0]
        joint_speed[1] = motor_speed[1] / self.gear_ratio[1]
        joint_speed[2] = motor_speed[2] / self.gear_ratio[2]
        joint_speed[4] = 0.5 * (motor_speed[3] + motor_speed[4]) / self.gear_ratio[3]   
        joint_speed[5] = 0.5 * (motor_speed[3] - motor_speed[4]) / self.gear_ratio[4]
        joint_speed /= 30 /np.pi              # convert from rpm to rad/s

        msg = JointState()
        msg.position = self.joint_position.tolist()
        msg.velocity = joint_speed.tolist()
        self.feedback_publisher.publish(msg)

    def stop_callback(self, request, response):
        response.success = True
        response.message = "Motors stopped"
        for id in self.canID:
            self.mks.stop_motor(id)
        return response

    def reset_callback(self, request, response):
        self.save_joint_positions()
        response.success = True
        response.message = "Motors reset"
        for id in self.canID:
            self.mks.reset_motor(id)
        return response
    
    def set_home_callback(self, request, response):
        response.success = True
        response.message = "Home position set"
        for id in self.canID:
            self.mks.set_home_position(id)
            self.mks.reset_motor(id)
        self.joint_position = np.zeros(6)
        self.joint_position_offset = np.zeros(6)
        self.save_joint_positions()
        return response
    
    def go_home_callback(self, request, response):
        response.success = True
        response.message = "Going to home position"
        target_pos = -self.joint_position[[0,1,3,4,5]] * self.gear_ratio[0:5] * 16383 / (2 * np.pi)
        for i, id in enumerate(self.canID):
            self.mks.go2pos(canID=id, target_pos=int(target_pos[i]), speed=100, acc=self.acc)
        return response

    def joint_state_callback(self, msg: JointState):
        motor_speed = np.zeros(5)
        motor_speed[0] = msg.velocity[0] * self.gear_ratio[0]                           # j1
        motor_speed[1] = -msg.velocity[1] * self.gear_ratio[1]                          # j2
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
        node.save_joint_positions()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()