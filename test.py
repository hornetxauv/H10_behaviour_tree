
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import py_trees
import py_trees.behaviours
import py_trees.composites
import py_trees.trees


class PIDController:
    def __init__(self, kp, ki, kd, dt=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class WallAvoidance(py_trees.behaviour.Behaviour):
    """
    If the turtle is too close to any wall, compute a repulsion vector
    and use a PID controller to generate an angular correction.
    Returns RUNNING (overriding lower-priority tasks) if active.
    """
    def __init__(self, name, node, publisher, safe_distance=1.5, arena_size=11.0):
        super(WallAvoidance, self).__init__(name)
        self.node = node
        self.publisher = publisher
        self.safe_distance = safe_distance
        self.arena_size = arena_size
        self.pid = PIDController(kp=2.0, ki=-0.2, kd=0.1, dt=0.1)

    def update(self):
        if not self.node.latest_pose:
            return py_trees.common.Status.FAILURE

        pose = self.node.latest_pose
        repulsion_x = 0.0
        repulsion_y = 0.0

        if pose.x < self.safe_distance:
            repulsion_x += 1.0
        if pose.x > (self.arena_size - self.safe_distance):
            repulsion_x -= 1.0
        if pose.y < self.safe_distance:
            repulsion_y += 1.0
        if pose.y > (self.arena_size - self.safe_distance):
            repulsion_y -= 1.0

        if repulsion_x == 0.0 and repulsion_y == 0.0:
            return py_trees.common.Status.FAILURE

        desired_heading = math.atan2(repulsion_y, repulsion_x)
        error = math.atan2(math.sin(desired_heading - pose.theta),
                           math.cos(desired_heading - pose.theta))
        angular_correction = self.pid.update(error)

        twist = Twist()
        twist.linear.x = 2.0 * 0.5  
        twist.angular.z = angular_correction
        self.publisher.publish(twist)

        self.node.get_logger().info(
            f"[WallAvoidance] desired_heading={desired_heading:.2f}, error={error:.2f}, correction={angular_correction:.2f}"
        )
        return py_trees.common.Status.RUNNING


class ObstacleAvoidance(py_trees.behaviour.Behaviour):
    """
    Checks fixed obstacle points. If any obstacle is detected within a specified
    distance and within the turtle's field-of-view, computes a repulsion vector.
    The turtle then "translates" (drives forward with an angular correction) to avoid the obstacle.
    Returns RUNNING while active.
    """
    def __init__(self, name, node, publisher,
                 detection_distance=2.0, fov_threshold=0.3,
                 nominal_speed=1.0, obstacle_kp=2.0):
        super(ObstacleAvoidance, self).__init__(name)
        self.node = node
        self.publisher = publisher
        self.detection_distance = detection_distance
        self.fov_threshold = fov_threshold 
        self.nominal_speed = nominal_speed
        self.obstacle_kp = obstacle_kp

    def update(self):
        if not self.node.latest_pose or not self.node.obstacles:
            return py_trees.common.Status.FAILURE

        pose = self.node.latest_pose
        repulsion_x = 0.0
        repulsion_y = 0.0
        detected = False

        for obs in self.node.obstacles:
            dx = obs[0] - pose.x
            dy = obs[1] - pose.y
            dist = math.hypot(dx, dy)
            if dist < self.detection_distance:
                angle_to_obs = math.atan2(dy, dx)
                angle_error = math.atan2(math.sin(angle_to_obs - pose.theta),
                                         math.cos(angle_to_obs - pose.theta))
                if abs(angle_error) < self.fov_threshold:
                    weight = (self.detection_distance - dist) / self.detection_distance
                    repulsion_x += -math.cos(angle_to_obs) * weight *1.5
                    repulsion_y += -math.sin(angle_to_obs) * weight *1.5
                    detected = True

        if not detected:
            return py_trees.common.Status.FAILURE

        desired_heading = math.atan2(repulsion_y, repulsion_x)
        error = math.atan2(math.sin(desired_heading - pose.theta),
                           math.cos(desired_heading - pose.theta))
        angular_correction = self.obstacle_kp * error

        twist = Twist()
        twist.linear.x = self.nominal_speed  
        twist.angular.z = angular_correction
        self.publisher.publish(twist)

        self.node.get_logger().info(
            f"[ObstacleAvoidance] desired_heading={desired_heading:.2f}, error={error:.2f}, correction={angular_correction:.2f}"
        )
        return py_trees.common.Status.RUNNING

class GoalScan(py_trees.behaviour.Behaviour):
    """
    Rotates the turtle in place until the goal is within view.
    Returns SUCCESS when the angular error to the goal is below a threshold.
    """
    def __init__(self, name, node, publisher, view_threshold=0.2, scan_speed=0.5):
        super(GoalScan, self).__init__(name)
        self.node = node
        self.publisher = publisher
        self.view_threshold = view_threshold
        self.scan_speed = scan_speed

    def update(self):
        if not self.node.latest_pose or self.node.current_goal is None:
            return py_trees.common.Status.FAILURE

        pose = self.node.latest_pose
        goal_x, goal_y = self.node.current_goal
        desired_heading = math.atan2(goal_y - pose.y, goal_x - pose.x)
        angle_error = math.atan2(math.sin(desired_heading - pose.theta),
                                 math.cos(desired_heading - pose.theta))
        self.node.get_logger().debug(
            f"[GoalScan] desired_heading={desired_heading:.2f}, current_heading={pose.theta:.2f}, angle_error={angle_error:.2f}"
        )
        if abs(angle_error) < self.view_threshold:
            return py_trees.common.Status.SUCCESS
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.scan_speed * (1 if angle_error > 0 else -1)
            self.publisher.publish(twist)
            return py_trees.common.Status.RUNNING

class GoalMove(py_trees.behaviour.Behaviour):
    """
    Drives the turtle forward toward the goal with slight angular corrections.
    If the goal is reached, a new goal is generated.
    """
    def __init__(self, name, node, publisher, goal_stop_distance=0.5, nominal_speed=2.0, goal_kp=2.0):
        super(GoalMove, self).__init__(name)
        self.node = node
        self.publisher = publisher
        self.goal_stop_distance = goal_stop_distance
        self.nominal_speed = nominal_speed
        self.goal_kp = goal_kp

    def update(self):
        if not self.node.latest_pose or self.node.current_goal is None:
            return py_trees.common.Status.FAILURE

        pose = self.node.latest_pose
        goal_x, goal_y = self.node.current_goal
        distance = math.hypot(goal_x - pose.x, goal_y - pose.y)

        if distance < self.goal_stop_distance:
            self.node.get_logger().info("[GoalMove] Goal reached!")
            self.node.current_goal = self.node.generate_new_goal()
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            return py_trees.common.Status.SUCCESS

        desired_heading = math.atan2(goal_y - pose.y, goal_x - pose.x)
        angle_error = math.atan2(math.sin(desired_heading - pose.theta),
                                 math.cos(desired_heading - pose.theta))
        twist = Twist()
        twist.linear.x = self.nominal_speed
        twist.angular.z = self.goal_kp * angle_error
        self.publisher.publish(twist)

        self.node.get_logger().info(
            f"[GoalMove] goal=({goal_x:.2f}, {goal_y:.2f}), distance={distance:.2f}, angle_error={angle_error:.2f}"
        )
        return py_trees.common.Status.RUNNING


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Pose, "turtle1/pose", self.pose_callback, 10
        )
        self.latest_pose = None
        self.arena_size = 11.0
        self.safe_distance = 1.5

        self.obstacles = [(3.0, 3.0), (7.0, 7.0), (5.0, 9.0), (4.0,4.0)]

        self.current_goal = self.generate_new_goal()

    def pose_callback(self, msg):
        self.latest_pose = msg

    def generate_new_goal(self):

        candidates = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        new_goal_x = candidates[int(random.uniform(0, len(candidates)))]
        new_goal_y = candidates[int(random.uniform(0, len(candidates)))]
        self.get_logger().info(f"New goal generated: ({new_goal_x:.2f}, {new_goal_y:.2f})")
        return (new_goal_x, new_goal_y)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    wall_avoidance = WallAvoidance(
        "WallAvoidance", node, node.publisher,
        safe_distance=node.safe_distance,
        arena_size=node.arena_size
    )
    obstacle_avoidance = ObstacleAvoidance(
        "ObstacleAvoidance", node, node.publisher,
        detection_distance=2.0, fov_threshold=0.3,
        nominal_speed=1.0, obstacle_kp=2.0
    )
    goal_scan = GoalScan("GoalScan", node, node.publisher, view_threshold=0.2, scan_speed=0.5)
    goal_move = GoalMove("GoalMove", node, node.publisher, goal_stop_distance=0.5, nominal_speed=2.0, goal_kp=2.0)
    goal_sequence = py_trees.composites.Sequence("GoalSequence", memory=True)
    goal_sequence.add_children([goal_scan, goal_move])

    root = py_trees.composites.Selector("RootSelector", memory=False)
    root.add_children([wall_avoidance, obstacle_avoidance, goal_sequence])
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    node.get_logger().info("Starting behavior tree...")

    rate = 10 
    period = 1.0 / rate
    try:
        while rclpy.ok():
            behaviour_tree.tick()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down...")
    finally:
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        node.publisher.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
