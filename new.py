import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
import py_trees
from time import sleep
import random


# RobotState: holds the sensor readings.

class RobotState:
    def __init__(self):
        self.gate_seen = False
        self.rotation = 0.0          
        self.gate_centered = False
        self.flare_in_way = False
        self.too_close_to_side_wall = False
        self.yaw = 0.0              

    def update_randomly(self):
        # This method is provided for testing purposes only.
        self.gate_seen = random.choice([True, False])
        if self.gate_seen:
            self.rotation = random.uniform(-20, 20) 
            self.gate_centered = random.choice([True, False])
            # Only simulate flare if the gate is centered.
            self.flare_in_way = self.gate_centered and random.choice([True, False])
        else:
            self.rotation = 0.0
            self.gate_centered = False
            self.flare_in_way = False
        self.too_close_to_side_wall = random.choice([True, False])
        self.yaw = random.uniform(-180, 180)


# Behavior Nodes (Logging, Decision, Safety, and Basic Node Behaviors)

class GateDetectionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        # For debugging, you can use the random update if needed:
        # self.state.update_randomly()
        self.logger.info(
            f"[GateDetection] seen={self.state.gate_seen}, "
            f"rotation={self.state.rotation:.1f}, centered={self.state.gate_centered}, "
            f"flare={self.state.flare_in_way}"
        )
        return py_trees.common.Status.SUCCESS

class RotateAndTranslateBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if self.state.gate_seen and abs(self.state.rotation) > 10:
            self.logger.info("RotateAndTranslate: Rotating toward gate & translating toward center.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class TranslateSidewaysBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if self.state.gate_seen and abs(self.state.rotation) < 10 and not self.state.gate_centered:
            self.logger.info("TranslateSideways: Translating sideways toward the gate.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class FlareAvoidanceBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if self.state.gate_seen and self.state.gate_centered and self.state.flare_in_way:
            self.logger.info("FlareAvoidance: Executing flare avoidance sequence.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class MoveForwardBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if self.state.gate_seen and self.state.gate_centered and not self.state.flare_in_way:
            self.logger.info("MoveForward: Moving forward.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class SweepForGateBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if not self.state.gate_seen:
            self.logger.info("SweepForGate: Gate not seen. Sweeping left/right to find it.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# Safety Behaviors
class SideWallAlarmBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if self.state.too_close_to_side_wall:
            self.logger.info("SideWallAlarm: Too close to side wall! Adjusting position.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class RealignBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, state: RobotState):
        super().__init__(name)
        self.state = state

    def update(self):
        if abs(self.state.rotation) > 90:
            self.logger.info("Realign: Rotation > 90°. Forcing realignment.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# Basic Node Behaviors (simulate continuous operation)
class MovementNodeBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
    def update(self):
        self.logger.info("MovementNode: Operating normally.")
        return py_trees.common.Status.SUCCESS

class PIDNodeBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
    def update(self):
        self.logger.info("PIDNode: Operating normally.")
        return py_trees.common.Status.SUCCESS

class CANHandlerNodeBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
    def update(self):
        self.logger.info("CANHandlerNode: Operating normally.")
        return py_trees.common.Status.SUCCESS


# Build the Behavior Tree Structure

def create_behavior_tree(state: RobotState) -> py_trees.trees.BehaviourTree:
    # Branch that logs sensor state.
    gate_detection = GateDetectionBehavior("GateDetection", state)
    
    # Main decision branch.
    decision_selector = py_trees.composites.Selector(name="DecisionLogic", memory=True)
    rotate_and_translate = RotateAndTranslateBehavior("RotateAndTranslate", state)
    
    centered_selector = py_trees.composites.Selector(name="CenteredDecision", memory=True)
    translate_sideways = TranslateSidewaysBehavior("TranslateSideways", state)
    flare_avoidance = FlareAvoidanceBehavior("FlareAvoidance", state)
    move_forward = MoveForwardBehavior("MoveForward", state)
    centered_selector.add_children([translate_sideways, flare_avoidance, move_forward])
    
    gate_seen_sequence = py_trees.composites.Sequence(name="GateSeenSequence", memory=True)
    gate_seen_sequence.add_children([rotate_and_translate, centered_selector])
    
    sweep_for_gate = SweepForGateBehavior("SweepForGate", state)
    decision_selector.add_children([gate_seen_sequence, sweep_for_gate])
    
    # Safety branch.
    safety_parallel = py_trees.composites.Parallel(
        name="SafetyChecks", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    side_wall_alarm = SideWallAlarmBehavior("SideWallAlarm", state)
    realign = RealignBehavior("Realign", state)
    safety_parallel.add_children([side_wall_alarm, realign])
    
    # Basic nodes branch.
    basic_nodes = py_trees.composites.Parallel(
        name="BasicNodes", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    movement = MovementNodeBehavior("MovementNode")
    pid = PIDNodeBehavior("PIDNode")
    can_handler = CANHandlerNodeBehavior("CANHandlerNode")
    basic_nodes.add_children([movement, pid, can_handler])
    
    # Root: run all branches concurrently.
    root = py_trees.composites.Parallel(
        name="Root", 
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    root.add_children([gate_detection, decision_selector, safety_parallel, basic_nodes])
    
    return py_trees.trees.BehaviourTree(root)


# RobotControlNode: Subscribes to simulated sensor data and ticks the tree.

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.state = RobotState()
        # Subscribe to the external sensor topic.
        self.create_subscription(String, 'sensor_data', self.sensor_callback, 10)
        self.tree = create_behavior_tree(self.state)
        self.tree.setup(timeout=15)
        # Timer to tick the behavior tree every second.
        self.create_timer(1.0, self.tick_tree)
    
    def sensor_callback(self, msg: String):
        try:
            # Expected format: "gate_seen,rotation,gate_centered,flare_in_way,too_close_to_side_wall,yaw"
            parts = msg.data.split(',')
            if len(parts) == 6:
                self.state.gate_seen = parts[0].strip() == "True"
                self.state.rotation = float(parts[1].strip())
                self.state.gate_centered = parts[2].strip() == "True"
                self.state.flare_in_way = parts[3].strip() == "True"
                self.state.too_close_to_side_wall = parts[4].strip() == "True"
                self.state.yaw = float(parts[5].strip())
                self.get_logger().info(f"Updated state: {self.state.__dict__}")
        except Exception as e:
            self.get_logger().error(f"Error parsing sensor data: {e}")
    
    def tick_tree(self):
        self.tree.tick()

# --------------------------------------------------------------------
# Main function
# --------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
