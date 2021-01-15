#! /usr/bin/env python3
import argparse
import os
import glob
import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
import yaml
from sensor_msgs.msg import CameraInfo


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("image_path", type=str, help="path to folder with image files")
    parser.add_argument("--calibration_file", type=str, default=None, help="path to calibration file (intrinsics.csv)")
    parser.add_argument("bag_export", type=str, help="path to exported bag file")
    parser.add_argument("--topic_colour_image", type=str, default="/camera/rgb/image_rect_color", help="base colour image topic name")
    parser.add_argument("--topic_depth_image", type=str, default="/camera/depth/image_rect_raw", help="base depth image topic name")
    parser.add_argument("--topic_colour_info", type=str, default="/camera/rgb", help="base colour camera info topic name")
    parser.add_argument("--topic_depth_info", type=str, default="/camera/depth", help="base depth camera info topic name")
    parser.add_argument("--fps", type=float, default=30, help="target frames per second")
    parser.add_argument("--camera_frame", type=str, default="camera_rgb_optical_frame", help="camera optical frame")
    args = parser.parse_args()

    ci = CameraInfo()
    ci.header.frame_id = args.camera_frame
    if not args.calibration_file:
        args.calibration_file = os.path.join(args.image_path, "intrinsics.csv")
    with open(args.calibration_file) as file:
        fx, fy, cx ,cy, w, h = file.readline().split(" ")
        ci.width, ci.height = int(w), int(h)
        ci.K[0] = float(fx)
        ci.K[4] = float(fy)
        ci.K[2] = float(cx)
        ci.K[5] = float(cy)

    bag = rosbag.Bag(args.bag_export, 'w')

    img_paths_colour = sorted(glob.glob(os.path.join(args.image_path, "rgb/Image*.png")), reverse=False)

    time = rospy.Time()
    cvbridge = CvBridge()

    for img_path in img_paths_colour:
        time += rospy.Duration.from_sec(1/args.fps)

        # load colour and depth image pair
        seq_nr_str = os.path.split(img_path)[1][5:9]
        cimg = cv2.imread(img_path)
        dimg_path = os.path.join(args.image_path, "depth/Image{}.png".format(seq_nr_str))

        if not os.path.isfile(dimg_path):
            # skip this image pair
            continue

        dimg = cv2.imread(dimg_path, flags=cv2.IMREAD_UNCHANGED)

        # format and write image messages
        ci.header.stamp = time
        msg_colour = cvbridge.cv2_to_compressed_imgmsg(cimg, dst_format="jpg")
        msg_depth = cvbridge.cv2_to_compressed_imgmsg(dimg, dst_format="png")

        # fix the format specifier for 16bit png images
        # https://github.com/ros-perception/vision_opencv/issues/250
        msg_depth.format = "16UC1; png compressed"

        msg_colour.header = ci.header
        msg_depth.header = ci.header

        bag.write(args.topic_colour_image+"/compressed", msg_colour, time)
        bag.write(args.topic_colour_info+"/camera_info", ci, time)
        bag.write(args.topic_depth_image+"/compressed", msg_depth, time)
        bag.write(args.topic_depth_info+"/camera_info", ci, time)

    bag.close()