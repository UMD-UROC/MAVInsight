#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from visualization_msgs.msg import MarkerArray

from models.vehicle import FlightPathChunks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--chunk-size', type=int, default=256)
    parser.add_argument('--rate', type=float, default=1.0)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.input), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = reader.get_all_topics_and_types()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    path_topics = {}
    for topic in topics:
        if re.fullmatch(r'/uas[^/]+/flightPath', topic.name):
            number = topic.name.split('/')[1]
            path_topics[number] = topic
            continue
        writer.create_topic(topic)
    for number in path_topics:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=f'/{number}/flightPathChunks',
            type='visualization_msgs/msg/MarkerArray',
            serialization_format='cdr',
            offered_qos_profiles=''))
    paths = {number: FlightPathChunks(f'{number}_ekf_origin', args.chunk_size)
             for number in path_topics}
    next_publish = {number: None for number in path_topics}
    period = int(1_000_000_000 / args.rate)
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        number = topic.split('/')[1] if re.fullmatch(r'/uas[^/]+/local_position/pose', topic) else None
        if number in paths:
            pose = deserialize_message(data, PoseStamped)
            paths[number].add(pose)
            if next_publish[number] is None:
                next_publish[number] = timestamp
            if timestamp >= next_publish[number]:
                message = paths[number].message()
                if message.markers:
                    writer.write(f'/{number}/flightPathChunks', serialize_message(message), timestamp)
                next_publish[number] += period
        if not re.fullmatch(r'/uas[^/]+/flightPath', topic):
            writer.write(topic, data, timestamp)
    for number, path in paths.items():
        message = path.message()
        if message.markers:
            writer.write(f'/{number}/flightPathChunks', serialize_message(message),
                         next_publish[number] or 0)


if __name__ == '__main__':
    main()
