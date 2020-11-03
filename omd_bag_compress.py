#! /usr/bin/env python3
import argparse
import rosbag
from cv_bridge import CvBridge
import cv2

# compress raw colour and depth images from 'sensor_msgs/Image' to 'sensor_msgs/CompressedImage'

# original IMD rosbag topics:
# /realsense_rgbd/aligned_depth_to_color/camera_info  : sensor_msgs/CameraInfo
# /realsense_rgbd/aligned_depth_to_color/image_raw    : sensor_msgs/Image
# /realsense_rgbd/color/camera_info                   : sensor_msgs/CameraInfo
# /realsense_rgbd/color/image_raw                     : sensor_msgs/Image


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # source rosbag file
    parser.add_argument("source_bag", type=str, help="original rosbag file with raw images")
    parser.add_argument("--source_topic_colour_image", type=str, default="/realsense_rgbd/color/image_raw", help="original colour topic name")
    parser.add_argument("--source_topic_depth_image", type=str, default="/realsense_rgbd/aligned_depth_to_color/image_raw", help="original depth topic name")
    parser.add_argument("--source_topic_colour_info", type=str, default="/realsense_rgbd/color/camera_info", help="original colour info topic name")
    parser.add_argument("--source_topic_depth_info", type=str, default="/realsense_rgbd/aligned_depth_to_color/camera_info", help="original depth info topic name")

    # target rosbag file
    parser.add_argument("bag_export", type=str, help="path to exported bag file with compressed images")
    parser.add_argument("--topic_colour_image", type=str, default="/camera/rgb/image_rect_color", help="rename base colour image topic")
    parser.add_argument("--topic_depth_image", type=str, default="/camera/depth/image_rect_raw", help="rename base depth image topic")
    parser.add_argument("--topic_colour_info", type=str, default="/camera/rgb", help="rename base colour camera info topic")
    parser.add_argument("--topic_depth_info", type=str, default="/camera/depth", help="rename base depth camera info topic")

    args = parser.parse_args()

    # map topics from source to target
    topic_map = dict()
    topic_map[args.source_topic_colour_image] = args.topic_colour_image+"/compressed"
    topic_map[args.source_topic_depth_image] = args.topic_depth_image+"/compressed"
    topic_map[args.source_topic_colour_info] = args.topic_colour_info+"/camera_info"
    topic_map[args.source_topic_depth_info] = args.topic_depth_info+"/camera_info"

    cvbridge = CvBridge()

    source_bag = rosbag.Bag(args.source_bag, 'r')
    target_bag = rosbag.Bag(args.bag_export, 'w')

    for topic, message, timestamp in source_bag.read_messages(topics=topic_map.keys()):
        if message._type == "sensor_msgs/Image":
            # print(message.encoding)
            img = cvbridge.imgmsg_to_cv2(message)
            # compress
            if topic==args.source_topic_colour_image:
                # jpeg compression
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                tgt_msg = cvbridge.cv2_to_compressed_imgmsg(img, "jpg")
            elif topic==args.source_topic_depth_image:
                # 16bit png compression
                tgt_msg = cvbridge.cv2_to_compressed_imgmsg(img, "png")
                # https://github.com/ros-perception/vision_opencv/issues/250
                tgt_msg.format = "16UC1; png compressed"
        else:
            tgt_msg = message

        target_bag.write(topic_map[topic], tgt_msg, t=timestamp)

    target_bag.close()
    source_bag.close()
