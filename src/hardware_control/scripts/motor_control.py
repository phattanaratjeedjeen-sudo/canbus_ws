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

        freq = 50.0
        self.declare_parameter('acc', 5)
        self.acc = self.get_parameter('acc').get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self.param_callback)

        self.n = 1 
        self.canID = (0x01, 0x02, 0x03, 0x04)
        self.gear_ratio = np.array([13.5, 150, 150, 48, 67.82, 67.82])
        self.mks = MKS()
        for id in self.canID:
            self.mks.reset_motor(id)
            time.sleep(0.1)
        
        self.joint_memory = os.path.join(os.path.expanduser('~'), 'canbus_ws', 'joint_positions.json')
        self.joint_position_offset = self.load_joint_positions()
        self.joint_position = np.copy(self.joint_position_offset)
        self.joint_speed = np.zeros(6+self.n)
        threshold = np.deg2rad(10)
        self.upper_limit = np.array([np.deg2rad(180), np.deg2rad(30), np.deg2rad(150), np.deg2rad(180)]) - threshold
        self.lower_limit = np.array([-np.deg2rad(180), np.deg2rad(-180), np.deg2rad(-5), np.deg2rad(-180)]) + threshold

        self.create_subscription(JointState, 'motor', self.motor_callback, 10)
        self.create_subscription(JointState, 'step_joint_states', self.step_joint_state_callback, 10)
        self.feedback_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.step_cmd_publisher = self.create_publisher(JointState, 'step_joint_cmd', 10)
        self.create_timer(1.0/freq, self.publish_feedback)
        self.create_service(Trigger, 'stop', self.stop_callback)
        # self.create_service(Trigger, 'reset', self.reset_callback)
        # self.create_service(Trigger, 'set_home' ,self.set_home_callback)
        # self.create_service(Trigger, 'go_home', self.go_home_callback)

        self.get_logger().info("Motor Control Node: RUNNING...")


    def param_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == 'acc' and param.type_ == ParameterType.PARAMETER_INTEGER and param.value is not None:
                self.acc = max(0, min(int(param.value), 255))
                self.get_logger().info(f"acc set {self.acc}")
        return SetParametersResult(successful=True)

    def load_joint_positions(self):
        if os.path.exists(self.joint_memory):
            try:
                with open(self.joint_memory, 'r') as f:
                    data = json.load(f)
                    self.get_logger().info("Loaded joint positions from storage.")
                    return np.array(data)
            except Exception as e:
                self.get_logger().error(f"Failed to load joint positions: {e}")
        return np.zeros(4)

    def save_joint_positions(self):
        try:
            with open(self.joint_memory, 'w') as f:
                json.dump(self.joint_position.tolist(), f)
                self.get_logger().info("Saved joint positions to storage.")
        except Exception as e:
            self.get_logger().error(f"Failed to save joint positions: {e}")

    def step_joint_state_callback(self, msg: JointState):
        joint5_speed = 0.5 * (msg.velocity[4] + msg.velocity[5]) / self.gear_ratio[4]
        joint6_speed = 0.5 * (msg.velocity[4] - msg.velocity[5]) / self.gear_ratio[5]
        joint5_pos = 0.5 * (msg.position[4] + msg.position[5]) / self.gear_ratio[4]
        joint6_pos = 0.5 * (msg.position[4] - msg.position[5]) / self.gear_ratio[5]
        self.joint_position[4+self.n:6+self.n] = self.joint_position_offset[4+self.n:6+self.n] + [joint5_pos, joint6_pos]
        self.joint_speed[4+self.n:6+self.n] = [joint5_speed, joint6_speed]

    def publish_feedback(self):
        motor_rev = np.zeros(4)
        motor_speed = np.zeros(4)

        for i, id in enumerate(self.canID):
            encoder = self.mks.read_abs_position(id) or 0.0
            motor_rev[i] = encoder / 16383
            motor_speed[i] = self.mks.read_speed(id) or 0.0

        joint_delta = np.zeros(4)
        joint_delta[0] = motor_rev[0] / self.gear_ratio[0]      # j1
        joint_delta[1] = -motor_rev[1] / self.gear_ratio[1]     # j2
        joint_delta[2] = motor_rev[2] / self.gear_ratio[2]      # j3
        joint_delta[3] = motor_rev[3] / self.gear_ratio[3]      # j4
        joint_delta_rad = joint_delta * 2 * np.pi
        self.joint_position[0+self.n:4+self.n] = self.joint_position_offset[0+self.n:4+self.n] + joint_delta_rad

        self.joint_speed[0] = motor_speed[0] / self.gear_ratio[0]    # j1
        self.joint_speed[1] = motor_speed[1] / self.gear_ratio[1]    # j2
        self.joint_speed[2] = motor_speed[2] / self.gear_ratio[2]    # j3
        self.joint_speed[3] = motor_speed[3] / self.gear_ratio[3]    # j4
        self.joint_speed[0+self.n:4+self.n] /= 30 / np.pi                          # convert from rpm to rad/s

        msg = JointState()
        msg.position = self.joint_position.tolist()
        msg.velocity = self.joint_speed.tolist()
        self.feedback_publisher.publish(msg)

    def stop_callback(self, request, response):
        response.success = True
        response.message = "Motors stopped"
        for id in self.canID:
            self.mks.stop_motor(id)
        return response

    # def reset_callback(self, request, response):
    #     self.save_joint_positions()
    #     self.joint_position = self.load_joint_positions()
    #     self.joint_position_offset = np.copy(self.joint_position)
    #     response.success = True
    #     response.message = "Motors reset"
    #     for id in self.canID:
    #         self.mks.reset_motor(id)
    #     return response
    
    # def set_home_callback(self, request, response):
    #     self.save_joint_positions()
    #     response.success = True
    #     response.message = "Home position set"
    #     return response
    
    # def go_home_callback(self, request, response):
    #     response.success = True
    #     response.message = "Going to home position"
    #     target = np.zeros(4)
    #     mask = np.array([1, -1, -1, -1])
    #     target[0] = self.joint_position[0] * self.gear_ratio[0]     # j1
    #     target[1] = -self.joint_position[1] * self.gear_ratio[1]    # j2
    #     target[2] = -self.joint_position[2] * self.gear_ratio[2]    # j3
    #     target[3] = -self.joint_position[3] * self.gear_ratio[3]    # j4
    #     target *= 16384 / (2 * np.pi)

    #     for i, id in enumerate(self.canID):
    #         self.mks.go2pos(canID=id, target_pos=int(target[i]), speed=int(5*self.gear_ratio[i]), acc=self.acc)

    #     return response
    
    def motor_callback(self, msg: JointState):
        mask = np.array([1, -1, -1, 1])
        motor_speed = np.zeros(6)
        motor_speed[0] = msg.velocity[0+self.n] * self.gear_ratio[0]                          
        motor_speed[1] = -msg.velocity[1+self.n] * self.gear_ratio[1]                         
        motor_speed[2] = -msg.velocity[2+self.n] * self.gear_ratio[2]                         
        motor_speed[3] = msg.velocity[3+self.n] * self.gear_ratio[3]
        motor_speed[4] = msg.velocity[4+self.n] * self.gear_ratio[4] + msg.velocity[5+self.n] * self.gear_ratio[5]    
        motor_speed[5] = msg.velocity[5+self.n] * self.gear_ratio[4] - msg.velocity[4+self.n] * self.gear_ratio[5]

        self.publish_step_cmd(motor_speed=[motor_speed[4], motor_speed[5]])

        for i, id in enumerate(self.canID):
            can_move_up = (self.joint_position[i+self.n] < self.upper_limit[i] or motor_speed[i]*mask[i] < 0)
            can_move_down = (self.joint_position[i+self.n] > self.lower_limit[i] or motor_speed[i]*mask[i] > 0)
            speed = int(motor_speed[i] * 60 / (2 * np.pi)) * int(can_move_up and can_move_down)     # convert to rpm
            speed = max(-1200, min(1200, speed))
            self.mks.send_speed(canID=id, speed=speed, acc=self.acc) 

    def publish_step_cmd(self, motor_speed):
        msg = JointState()
        mask = np.array([1, 1])
        for i in range(len(motor_speed)):
            can_move_up = (self.joint_position[i+self.n] < self.upper_limit[i] or motor_speed[i]*mask[i] < 0)
            can_move_down = (self.joint_position[i+self.n] > self.lower_limit[i] or motor_speed[i]*mask[i] > 0)
            motor_speed[1] = max(-700, min(700, motor_speed[1])) * int(can_move_up and can_move_down)
        msg.velocity = motor_speed
        self.step_cmd_publisher.publish(msg)


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