# ROS imports
import rclpy
from rclpy.node import Node

class GraphMember(Node):
    """The base class for all objects that could be displayed in the 3D panel of Foxglove

    Class Attributes
    ----------------
    param_reqs : list[str]
        A list of required parameters/keys that a dict-encoded version of a `GraphMember` would need
        to be considered "valid".

    Attributes
    ----------
    frame_name : str
        The string name of the frame that this object represents. (i.e. "base_link")
    name : str
        The string internal name of this object. (i.e. "Chimera D4")
    parent_frame : str
        The string name of the frame of the parent to this object. (i.e. "map")
    tab_char : str
        The desired "tab" string. Used during string formatting of GraphMember and its subclasses
    """
    FRAME_NAME:str
    DISPLAY_NAME:str
    PARENT_FRAME:str
    _tab_char:str

    # Constructors
    def __init__(self):
        super().__init__("graph_member", automatically_declare_parameters_from_overrides=True)
        self.get_logger().info(f"Received node name: {self.get_name()}")
        self.get_logger().info(f"Ingesting Graph Member params...")

        # Ingest ROS parameters. Notify user when defaults are being used.
        if self.has_parameter('frame_name'):
            self.FRAME_NAME = self.get_parameter('frame_name').get_parameter_value().string_value
        else:
            self.default_parameter_warning("frame_name")
            self.FRAME_NAME = "base_link"

        if self.has_parameter('display_name'):
            self.DISPLAY_NAME = self.get_parameter('display_name').get_parameter_value().string_value
        else:
            self.default_parameter_warning("display_name")
            self.DISPLAY_NAME = "Default Vehicle Name"

        if self.has_parameter('parent_frame'):
            self.PARENT_FRAME = self.get_parameter('parent_frame').get_parameter_value().string_value
        else:
            self.get_logger().info(f"parent_frame param not set, using standard default: \"map\"")
            self.PARENT_FRAME = "map"

        if self.has_parameter('tab_char'):
            self._tab_char = self.get_parameter('tab_char').get_parameter_value().string_value
        else:
            self._tab_char = "|   "

    def default_parameter_warning(self, param_name:str):
        self.get_logger().warn(f"Parameter {param_name} not set in config file. using default")

    def __str__(self):
        return f"{self.DISPLAY_NAME}\nTransform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n"

def main(args=None):
    rclpy.init(args=args)

    node = GraphMember()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down...")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
