from rosbags.rosbag2 import Reader as ROS2Reader
import sqlite3

from rosbags.serde import deserialize_cdr
import matplotlib.pyplot as plt
import os
import collections
import argparse
        
parser = argparse.ArgumentParser(description='Extract images from rosbag.')
# input will be the folder containing the .db3 and metadata.yml file
parser.add_argument('--input','-i',type=str, help='rosbag input location')
# run with python filename.py -i rosbag_dir/

args = parser.parse_args()

rosbag_dir = args.input

topic = "/topic/name"
frame_counter = 0

with ROS2Reader(rosbag_dir) as ros2_reader:

    ros2_conns = [x for x in ros2_reader.connections]
    # This prints a list of all topic names for sanity
    print([x.topic for x in ros2_conns])

    ros2_messages = ros2_reader.messages(connections=ros2_conns)
    
    for m, msg in enumerate(ros2_messages):
        (connection, timestamp, rawdata) = msg
        
        if (connection.topic == topic):
            print(connection.topic) # shows topic
            print(connection.msgtype) # shows message type
            print(type(connection.msgtype)) # shows it's of type string

            # TODO
            # this is where things crash when it's a custom message type
            data = deserialize_cdr(rawdata, connection.msgtype)
            print(data)