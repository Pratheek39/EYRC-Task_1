#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose
import numpy as np

class BattalionController(Node):
    def __init__(self):
        super().__init__('battalion_controller')

        # --- Service client (for targets) ---
        self.cli = self.create_client(Trigger, '/eyantrasim/get_coordinates')
        self.req = Trigger.Request()

        # --- Publishers (for bots) ---
        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)
        self.pub_crystal = self.create_publisher(Twist , '/eyantrasim/crystal/cmd_vel', 10)
        self.pub_frostbite = self.create_publisher(Twist , '/eyantrasim/frostbite/cmd_vel',10)

        # --- Subscribers (for bot poses) ---
        self.sub_glacio = self.create_subscription(Pose, '/eyantrasim/glacio/pose', self.pose_glacio_cb, 10)
        self.sub_crystal = self.create_subscription(Pose, '/eyantrasim/crystal/pose', self.pose_crystal_cb, 10)
        self.sub_frostbite = self.create_subscription(Pose, '/eyantrasim/frostbite/pose', self.pose_frostbite_cb, 10)

        # --- Storage placeholders ---
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []

        self.current_pose_glacio = None
        self.current_pose_crystal = None
        self.current_pose_frostbite = None

        # Example: get target coordinates
        self.get_targets()

        self.get_logger().info(self.current_pose_crystal)
        # --- Timer for control loop ---
        self.timer = self.create_timer(0.1, self.control_loop)
        #self.timer_2 = self.create_timer(0.1 , self.get_coords)


    # --- Example method: request targets from service ---
    def get_targets(self):
        while not self.cli.wait_for_service(1.0):
            self.get_logger().warn("get coords service not up...")

        future = self.cli.call_async(self.req)
        future.add_done_callback(self.get_targets_callback)

    
    def get_targets_callback(self,future):
        response = future.result()
        coords = eval(response.message)
        self.targets_glacio = coords[0:50]
        self.targets_crystal = coords[50:100]
        self.targets_frostbite = coords[100:150]
        print(self.targets_glacio)


    # --- Pose callbacks ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_crystal_cb(self, msg):
        self.current_pose_crystal = msg

    def pose_frostbite_cb(self, msg):
        self.current_pose_frostbite= msg


    # --- Example controller ---
    def compute_velocity(self, current_pose : Pose, target_pose):
        vel = Twist()
        # implement P/PD controller
        
        dx = current_pose.x - target_pose[0]
        dy = current_pose.y - target_pose[1]

        if(abs(dx) <= 1e-5 ):
            theta = np.pi/2 - current_pose.theta 

        else :
            theta = np.arctan(dy/dx) - current_pose.theta

        if(abs(theta) >= 1e-5 ):
            vel.angular.z =  theta
            print(f"angle = {theta}")

        else:
            d = np.sqrt(dx**2 + dy**2)
            vel.linear.x = 0.05 * d
            if(abs(d) < 1e-5):
                self.targets_glacio = self.targets_glacio[1:]
                self.get_logger().warn("target_reached")

            print(f"dist = {d}")

    

        return vel

    # --- Control loop for all bots ---
    def control_loop(self):
        if self.current_pose_glacio and self.targets_glacio:
            vel = self.compute_velocity(self.current_pose_glacio, self.targets_glacio[0])
            self.pub_glacio.publish(vel)

    def get_coords(self):
        self.get_logger().info(str(self.current_pose_crystal))

def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()