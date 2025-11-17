import rclpy
from rclpy.node import Node
from navigation.util import draw_map_and_drone
from std_msgs.msg import Float64MultiArray
import numpy as np

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        #screen is 400x400
        #must land directly on target_pos
        self.target_pos = np.array([346.41, 200])
        self.current_pos = np.array([0,0])
        self.pos_publisher = self.create_publisher(Float64MultiArray, 'currentpos', 10)
        self.target_publisher = self.create_publisher(Float64MultiArray, 'targetpos', 10)
        timer_period = 0.5
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.vel_subscription = self.create_subscription(Float64MultiArray,
            'measuredvel',
            self.update_position,
            10)

    def timer_callback(self):
        pos_msg = Float64MultiArray()
        pos_msg.data = [float(self.current_pos[0]), float(self.current_pos[1])]
        targ_msg = Float64MultiArray()
        targ_msg.data = [float(self.target_pos[0]), float(self.target_pos[1])]

    def update_position(self, msg):
        x_vel = round(msg.data[0], 2)
        y_vel = round(msg.data[1], 2)
        x_pos = self.current_pos[0]+x_vel
        y_pos = self.current_pos[1]+y_vel
        self.current_pos = np.array([x_pos, y_pos])
        if (round(self.target_pos[0], 2) == round(x_pos,2) and 
            round(self.target_pos[1], 2) == round(y_pos,2)):
            self.get_logger().info(
                "Arrived at destination!"
            )
        return