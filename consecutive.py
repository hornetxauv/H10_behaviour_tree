import py_trees
from py_trees_ros.trees import BehaviourTree
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class PublishTask(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, topic_name, message):
        super(PublishTask, self).__init__(name)
        self.node = node
        self.publisher = self.node.create_publisher(String, topic_name, 10)
        self.message = message
        self.published = False
        print(f"[DEBUG] {name} initialized for topic: {topic_name}")

    def initialise(self):
        self.published = False  

    def update(self):
        if not self.published:
            msg = String()
            msg.data = self.message
            self.publisher.publish(msg)
            print(f"[DEBUG] {self.name} published: {self.message}")
            self.published = True
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.SUCCESS  


def main():
    print("[DEBUG] Starting ROS2 and py_trees")
    rclpy.init()

    node = rclpy.create_node('py_trees_sequential_node')
    print("[DEBUG] ROS2 Node initialized")

    task_1 = PublishTask(
        node=node,
        name="Task 1",
        topic_name="topic",
        message="Message from Task 1"
    )
    task_2 = PublishTask(
        node=node,
        name="Task 2",
        topic_name="topic",
        message="Message from Task 2"
    )
    task_3 = PublishTask(
        node=node,
        name="Task 3",
        topic_name="topic",
        message="Message from Task 3"
    )

    sequence = py_trees.composites.Sequence(name="Sequential Tasks", memory= False)
    sequence.add_children([task_1, task_2, task_3])

    print("[DEBUG] Behavior Tree:")
    print(py_trees.display.ascii_tree(sequence))

    behaviour_tree = BehaviourTree(root=sequence)
    behaviour_tree.setup(node=node)

    print("[DEBUG] Ticking the Behavior Tree:")
    try:
        rate = node.create_rate(1.0)  
        while rclpy.ok():
            print("[DEBUG] Ticking...")
            behaviour_tree.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1)
    except KeyboardInterrupt:
        print("[DEBUG] KeyboardInterrupt detected. Shutting down...")
    finally:
        behaviour_tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("[DEBUG] Shutdown completed")


if __name__ == "__main__":
    main()
