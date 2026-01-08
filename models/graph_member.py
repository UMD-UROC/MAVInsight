# python imports
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

class GraphMember(Node):
    """The base class/Node for all objects that could be displayed in the 3D panel of
    Foxglove.

    Parameters
    ----------
    display_name: str
        The human-readable display name for the object. (i.e. "Chimera D4")
    frame_name : str
        The name of the frame for this object. Used in TF Frames. (i.e. "base_link")
    parent_frame : str
        The name of the frame that is a parent to this object's frame. Used in TF Frames.
        (i.e. "map")
    tab_char : str
        The desired "tab" string. Used during string formatting of GraphMember and its
        subclasses.
    """

    FRAME_NAME: str
    DISPLAY_NAME: str
    PARENT_FRAME: str
    POSE_FRAME: str #TODO: When getting this from px4_msgs, the frame is embedded in the message. get this dynamically from the message?
                    #TODO: We may need to split this into a position frame and orientation frame (px4 odometry quaternion reported as body->world, but position reported as world->body)
    _tab_char: str

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
            self.get_logger().info(f'parent_frame param not set, using standard default: "map"')
            self.PARENT_FRAME = "map"

        if self.has_parameter('pose_frame'):
            self.POSE_FRAME = self.get_parameter('pose_frame').get_parameter_value().string_value
        else:
            self.get_logger().info(f"pose_frame param not set, using standard default: \"enu_flu\".")
            self.POSE_FRAME = 'enu_flu'

        if self.has_parameter('tab_char'):
            self._tab_char = self.get_parameter('tab_char').get_parameter_value().string_value
        else:
            self._tab_char = "|   "

        # initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # initialize static broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Wait for foxglove to subscribe to our topics
        self.get_logger().info(f"Waiting for Foxglove...")
        i = 0
        while self.count_subscribers('/tf_static') == 0:
            self.get_logger().info(f"...{i}")
            i+=1
            time.sleep(1.0)
        self.get_logger().info(f"Foxglove found.")

    def default_parameter_warning(self, param_name: str):
        """Helper method to output a boilerplate warning indicating that a default
        parameter is being used in the absence of a defined valid parameter.

        Parameters
        ----------
        param_name : str
            The name of the parameter to be used in the warning.
        """
        self.get_logger().warn(f"Parameter {param_name} not set in config file. using default")

    def __str__(self):
        return f"{self.DISPLAY_NAME}\nTransform: {self.PARENT_FRAME} -> {self.FRAME_NAME}\n"

    @classmethod
    def main(cls, args=None):
        rclpy.init(args=args)
        node = cls()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            print("Shutting down...")
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
