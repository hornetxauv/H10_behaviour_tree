import py_trees
from py_trees_ros.trees import BehaviourTree
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TaskA(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="TaskA"):
        super(TaskA, self).__init__(name)
        self.node = node
        self.publisher = self.node.create_publisher(String, 'task_a_topic', 10)
        print("[DEBUG] TaskA initialized")

    def update(self):
        msg = String()
        msg.data = "Task A running..."
        self.publisher.publish(msg)
        print("[DEBUG] TaskA update called - Published message")
        return py_trees.common.Status.SUCCESS


class TaskB(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="TaskB"):
        super(TaskB, self).__init__(name)
        self.node = node
        self.subscription = self.node.create_subscription(
            String,
            'task_a_topic',  
            self.callback,
            10
        )
        self.message = None
        print("[DEBUG] TaskB initialized")

    def callback(self, msg):
        self.message = msg.data
        print(f"[DEBUG] TaskB received message: {self.message}")

    def update(self):
        if self.message:
            print(f"[DEBUG] TaskB processing message: {self.message}")
            self.message = None  
            return py_trees.common.Status.SUCCESS
        else:
            print("[DEBUG] TaskB waiting for a message...")
            return py_trees.common.Status.RUNNING


def main():
    print("[DEBUG] Starting ROS2 and py_trees")
    rclpy.init()

    node = rclpy.create_node('py_trees_node')
    print("[DEBUG] ROS2 Node initialized")

    parallel = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    print("[DEBUG] Parallel composite initialized")

    task_a = TaskA(node=node, name="Task A")
    task_b = TaskB(node=node, name="Task B")
    parallel.add_children([task_a, task_b])

    print("[DEBUG] Behavior Tree:")
    print(py_trees.display.ascii_tree(parallel))

    behaviour_tree = BehaviourTree(root=parallel)
    behaviour_tree.setup(node=node)

    try:
        rate = node.create_rate(2.0)  # 2 Hz
        while rclpy.ok():
            behaviour_tree.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1)
    except KeyboardInterrupt:
        print("[DEBUG] KeyboardInterrupt detected. Shutting down...")
    finally:
        behaviour_tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
