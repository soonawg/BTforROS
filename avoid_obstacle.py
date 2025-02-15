import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import py_trees

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name, cmd_vel_publisher):
        super().__init__(name)
        self.cmd_vel_publisher = cmd_vel_publisher

    def update(self):
        twist = Twist()
        twist.linear.x = 0.2  # 앞으로 이동
        twist.angular.z = 0.0 # 회전은 하지 않음
        self.cmd_vel_publisher.publish(twist)
        print("Moving forward...")
        return py_trees.common.Status.SUCCESS

class ObstacleCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.scan_data = None
    
    def update(self):
        # 장애물 감지 | LIDAR 센서의 데이터를 기준으로 장애물 유무 판단
        if self.scan_data is not None:
            front_ranges = self.scan_data[len(self.scan_data)//3:2*len(self.scan_data)//3]  # 전방 30도 범위
            if min(front_ranges) < 0.5:  # 0.5m 이내에 장애물 존재할 시
                print("Obstacle detected")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def set_scan_data(self, scan_data):
        self.scan_data = scan_data

class AvoidObstacle(py_trees.behaviour.Behaviour):
    def __init__(self, name, cmd_vel_publisher):
        super().__init__(name)
        self.cmd_vel_publisher = cmd_vel_publisher
    
    def update(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.cmd_vel_publisher.publish(twist)
        print("Avoiding obstacle...")
        return py_trees.common.Status.SUCCESS

class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_behaviour_tree_node")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        self.scan_data = None
        self.behaviour_tree = None

        self.create_behaviour_tree()
    
    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        self.behaviour_tree.root.children[0].children[0].set_scan_data(self.scan_data)
    
    def create_behaviour_tree(self):
        move_forward = MoveForward("Move Forward", self.cmd_vel_publisher)
        obstacle_check = ObstacleCheck("Obstacle Check")
        avoid_obstacle = AvoidObstacle("Avoid Obstacle", self.cmd_vel_publisher)

        # 행동 트리 구성
        sequence = py_trees.composites.Sequence("Move and Avoid Obstacle", memory=True, children=[obstacle_check, move_forward])

        root = py_trees.composites.Selector("Root", memory=True, children=[sequence, avoid_obstacle])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root)

    def run(self):
        while rclpy.ok():
            self.behaviour_tree.tick()
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()