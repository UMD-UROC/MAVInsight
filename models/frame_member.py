# MAVInsight imports
from models.graph_member import GraphMember

class FrameMember(GraphMember):
    FRAME_NAME: str
    PARENT_FRAME: str
    POSE_FRAME: str #TODO: When getting this from px4_msgs, the frame is embedded in the message. get this dynamically from the message?
                    #TODO: We may need to split this into a position frame and orientation frame (px4 odometry quaternion reported as body->world, but position reported as world->body)

    def __init__(self):
        super().__init__()

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Ingesting Frame Member params....")
        # Ingest ROS parameters. Notify user when defaults are being used.
        if self.has_parameter('frame_name'):
            self.FRAME_NAME = self.get_parameter('frame_name').get_parameter_value().string_value
        else:
            self.default_parameter_warning("frame_name")
            self.FRAME_NAME = "base_link"

        if self.has_parameter('parent_frame'):
            self.PARENT_FRAME = self.get_parameter('parent_frame').get_parameter_value().string_value
        else:
            self.default_parameter_warning("parent_frame")
            self.PARENT_FRAME = "map"

        if self.has_parameter('pose_frame'):
            self.POSE_FRAME = self.get_parameter('pose_frame').get_parameter_value().string_value
        else:
            self.default_parameter_warning("pose_frame")
            self.POSE_FRAME = 'enu_flu'

        self.get_logger().info(f"[{self.DISPLAY_NAME}]: Frame Member initialized!")
