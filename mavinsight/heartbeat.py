"""Optional liveness timer for nodes without a dependable periodic output."""
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String


def start_heartbeat(node, topic=None):
    """Call after construction; publish the node name from its executor at 1 Hz.

    Onboard launch supplies a topic only for nodes needing an explicit
    heartbeat. Other launches and nodes with existing liveness sources do
    not get another publisher or timer.
    """
    if topic is None:
        parameter = 'heartbeat.topic.node_liveness'
        if not node.has_parameter(parameter):
            node.declare_parameter(parameter, '')
        topic = node.get_parameter(parameter).value
    if not topic:
        return
    publisher = node.create_publisher(String, topic, 1)
    return node.create_timer(
        1.0, lambda: publisher.publish(String(data=node.get_fully_qualified_name())),
        clock=Clock(clock_type=ClockType.STEADY_TIME))
