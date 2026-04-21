#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter, ParameterType
from rcl_interfaces.msg import SetParametersResult
from hardware_control.mks import MKS
import numpy as np
import can
from can.bus import BusState

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        freq = 50
        self.declare_parameter('acc', 5)
        self.acc = self.get_parameter('acc').get_parameter_value().integer_value

        self.joint_speed = np.zeros(5)
        self.joint_speed_fb = np.zeros(5)
        self.joint_position = np.zeros(5)
        self.joint_position_fb = np.zeros(5)
        self.motor_speed = np.zeros(5)
        self.motor_speed_fb = np.zeros(5)
        self.motor_position = np.zeros(5)
        self.motor_position_fb = np.zeros(5)

        self.canID = [0x01, 0x02, 0x04, 0x05, 0x06]
        self.gear_ratio = np.array([13.5, 150, 48, 67.82, 67.82])
        self.bus = can.interface.Bus(interface='slcan', channel='/dev/ttyACM1', bitrate=500000)

        try:
            self.bus.state = BusState.PASSIVE
        except NotImplementedError:
            pass

        self.motors = [MKS(canID=id, bus=self.bus) for id in self.canID]

        self.create_subscription(JointState, 'joint_cmd', self.joint_state_callback, 10)
        self.feedback_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0/freq, self.publish_feedback)
        self.create_service(Trigger, 'stop', self.stop_callback)
        self.create_service(Trigger, 'reset', self.reset_callback)

        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == 'acc' and param.type_ == ParameterType.PARAMETER_INTEGER and param.value is not None:
                self.acc = max(0, min(int(param.value), 255))
                self.get_logger().info(f"acc set {self.acc}")
        return SetParametersResult(successful=True)

    def publish_feedback(self):
        for motor in self.motors:
            motor.read_position()
        msg = self.bus.recv(1)

        if msg is not None:

            # Bytes 1 to 5 (index 1:5) represent the 'carry' (int32)
            carry = int.from_bytes(msg.data[1:5], byteorder='big', signed=True)

            # Bytes 5 to 7 (index 5:7) represent the single-turn 'value' (uint16)
            value = int.from_bytes(msg.data[5:7], byteorder='big', signed=False)

            self.get_logger().info(f"id: {msg.arbitration_id}, Carry: {carry}")



        # for i, motor in enumerate(self.motors):
        #     self.motor_position_fb[i] = motor.read_position() or 0.0
        #     self.motor_speed_fb[i] = motor.read_speed() or 0.0

        # self.joint_position_fb[0] = self.motor_position_fb[0] / self.gear_ratio[0]
        # self.joint_position_fb[1] = self.motor_position_fb[1] / self.gear_ratio[1]
        # self.joint_position_fb[2] = self.motor_position_fb[2] / self.gear_ratio[2]
        # self.joint_position_fb[3] = 0.5 * (self.motor_position_fb[3] + self.motor_position_fb[4]) / self.gear_ratio[3]
        # self.joint_position_fb[4] = 0.5 * (self.motor_position_fb[3] - self.motor_position_fb[4]) / self.gear_ratio[4]

        # self.joint_speed_fb[0] = self.motor_speed_fb[0] / self.gear_ratio[0]
        # self.joint_speed_fb[1] = self.motor_speed_fb[1] / self.gear_ratio[1]
        # self.joint_speed_fb[2] = self.motor_speed_fb[2] / self.gear_ratio[2]
        # self.joint_speed_fb[3] = 0.5 * (self.motor_speed_fb[3] + self.motor_speed_fb[4]) / self.gear_ratio[3]   
        # self.joint_speed_fb[4] = 0.5 * (self.motor_speed_fb[3] - self.motor_speed_fb[4]) / self.gear_ratio[4]

        # msg = JointState()
        # msg.position = self.joint_position_fb.tolist()
        # msg.velocity = self.joint_speed_fb.tolist()
        # self.feedback_publisher.publish(msg)
        

    def stop_callback(self, request, response):
        response.success = True
        response.message = "Motors stopped"

        for motor in self.motors:
            motor.stop_motor()

        return response

    def reset_callback(self, request, response):
        response.success = True
        response.message = "Motors reset"

        for motor in self.motors:
            motor.reset_motor()

        return response

    def joint_state_callback(self, msg: JointState):
        self.motor_speed[0] = msg.velocity[0] * self.gear_ratio[0]
        self.motor_speed[1] = msg.velocity[1] * self.gear_ratio[1]
        self.motor_speed[2] = msg.velocity[2] * self.gear_ratio[2]
        self.motor_speed[3] = 0.5 * (msg.velocity[3] + msg.velocity[4])
        self.motor_speed[4] = 0.5 * (msg.velocity[3] - msg.velocity[4])

        for i, speed in enumerate(self.motor_speed):
            speed = max(-1500, min(1500, int(speed * 60 / (2 * np.pi))))
            self.motor_speed[i] = speed
            self.motors[i].send_speed(speed=speed, acc=self.acc)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
