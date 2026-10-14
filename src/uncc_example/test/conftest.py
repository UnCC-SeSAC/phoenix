from pathlib import Path
import sys
from types import ModuleType

sys.path.insert(0, str(Path(__file__).parents[1]))


try:
    import ros_robot_controller_msgs.msg  # noqa: F401
except ImportError:
    controller_package = ModuleType("ros_robot_controller_msgs")
    controller_msg = ModuleType("ros_robot_controller_msgs.msg")

    class MotorState:
        def __init__(self, *, id=0, rps=0.0):
            self.id = id
            self.rps = rps

    class MotorsState:
        def __init__(self):
            self.data = []

    controller_msg.MotorState = MotorState
    controller_msg.MotorsState = MotorsState
    controller_package.msg = controller_msg
    sys.modules["ros_robot_controller_msgs"] = controller_package
    sys.modules["ros_robot_controller_msgs.msg"] = controller_msg
