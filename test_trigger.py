import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import numpy as np

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.srv = self.create_service(Trigger, 'test_srv', self.callback)
        self.joint_position_offset = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.gear_ratio = np.array([13.5, 150, 48, 67.82, 67.82, 150])
        
    def callback(self, request, response):
        response.success = True
        response.message = "Test message"
        
        joint_delta = -self.joint_position_offset / (2 * np.pi)
        target_motor_rev = np.zeros(5)
        # do some dummy math
        target_motor_rev[0] = joint_delta[0] * self.gear_ratio[0]
        target_pos = target_motor_rev * 16383
        for i in range(5):
            val = int(target_pos[0])
            
        return response

rclpy.init()
node = TestNode()
client = node.create_client(Trigger, 'test_srv')
while not client.wait_for_service(timeout_sec=1.0):
    pass
req = Trigger.Request()
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)
print("Result:", future.result())
rclpy.shutdown()
