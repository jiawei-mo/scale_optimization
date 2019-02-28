import argparse
import rospy
import rosbag
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('rosbag', help='rosbag file')
    parser.add_argument('topic0', help='camera topic 0')
    parser.add_argument('topic1', help='camera topic 1')
    parser.add_argument('odom', help='ground truth topic')
    parser.add_argument('save_path', help='save path')

    bridge = CvBridge()

    args = parser.parse_args()
    get0 = False
    get1 = False
    get_odom = False
    count = 0

    cam0_path = args.save_path + "/cam0";
    cam1_path = args.save_path + "/cam1";
    img0_path = cam0_path + "/data";
    img1_path = cam1_path + "/data";

    try:  
        os.mkdir(args.save_path)
        os.mkdir(cam0_path)
        os.mkdir(cam1_path)
        os.mkdir(img0_path)
        os.mkdir(img1_path)
    except OSError:  
        print ("Creation of the directory failed")

    data0_file = open(cam0_path+"/data.csv", "w")
    data1_file = open(cam1_path+"/data.csv", "w")
    time_file = open(args.save_path+"/times.txt", "w")
    gt_file = open(args.save_path+"/pose.txt", "w")

    data0_file.write("timestamp [ns], filename\n")
    data1_file.write("timestamp [ns], filename\n")

    for topic, msg, t in rosbag.Bag(args.rosbag).read_messages():
        if (topic == args.topic0):
            try:
                image0 = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print e
            get0 = True;

        if (topic == args.topic1):
            try:
                image1 = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print e
            get1 = True;

        if (topic == args.odom):
            gt_pose = msg.pose.pose
            get_odom = True

        if (get0 and get1):
            get0 = False
            get1 = False
            cv2.imwrite("%s/%05d.png"%(img0_path,count), image0)
            cv2.imwrite("%s/%05d.png"%(img1_path,count), image1)
            data0_file.write("%ld, %05d.png\n"%(msg.header.stamp.to_sec()*10e6,count))
            data1_file.write("%ld, %05d.png\n"%(msg.header.stamp.to_sec()*10e6,count))
            time_file.write("%f\n"%msg.header.stamp.to_sec())
            if(get_odom):
                gt_file.write("%f %f %f %f %f %f %f %f\n"%(msg.header.stamp.to_sec(), \
                                                              gt_pose.position.x,\
                                                              gt_pose.position.y,\
                                                              gt_pose.position.z,\
                                                              gt_pose.orientation.w,\
                                                              gt_pose.orientation.x,\
                                                              gt_pose.orientation.y,\
                                                              gt_pose.orientation.z\
                                                              ))

            count = count+1

    time_file.close()
    gt_file.close()