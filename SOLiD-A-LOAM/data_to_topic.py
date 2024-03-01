import os
import tf
import rospy
import struct
import argparse
import numpy as np
import sensor_msgs.point_cloud2 as pcl2

from sensor_msgs.msg import PointCloud2, PointField, Imu
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from datetime import datetime

def read_bin(bin_path, data_type):
    if data_type == 'kitti':
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])
    else:
        raise ValueError("Unsupported lidar type")

    points = np.fromfile(bin_path, dtype=dtype)
    return points

def read_imu(imu_msg, imu_data, data_type):
    if data_type == 'kitti':
        q = tf.transformations.quaternion_from_euler(imu_data[0], imu_data[1], imu_data[2])
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.linear_acceleration.x = imu_data[3]
        imu_msg.linear_acceleration.y = imu_data[4]
        imu_msg.linear_acceleration.z = imu_data[5]
        imu_msg.angular_velocity.x = imu_data[6]
        imu_msg.angular_velocity.y = imu_data[7]
        imu_msg.angular_velocity.z = imu_data[8]
        return imu_msg
     
def publish_bin_data(pub_bin, pub_clock, data_type, lidar_frame_id, bin_dir, datatimes):
    bin_files = sorted([file for file in os.listdir(bin_dir) if file.endswith('.bin')])
    
    for i, (bin_file, dt) in enumerate(zip(bin_files, datatimes)):
        if bin_file is None or datatimes is None:
            continue
        #------- pub clock msg -------#
        secs = int(dt)
        nsecs = int((dt - secs) * 1e9)
        clock_msg = Clock()
        clock_msg.clock.secs = secs
        clock_msg.clock.nsecs = nsecs

        pub_clock.publish(clock_msg)
        
        #------- pub point msg -------#
        bin_path = os.path.join(bin_dir, bin_file)
        points = read_bin(bin_path, data_type)
        
        header = Header()
        header.frame_id = lidar_frame_id
        header.stamp = rospy.Time.from_sec(float(dt))
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('i', 12, PointField.FLOAT32, 1)]
        
        pcl_msg = pcl2.create_cloud(header, fields, points)        
        pub_bin.publish(pcl_msg)
        
        if i < len(datatimes)-1:
            next_dt = datatimes[i+1]
            wait_time = next_dt - dt
            rospy.sleep(wait_time)
        
def publish_imu_data(pub_imu, data_type, imu_frame_id, imu_dir, datatimes):
    imu_files = sorted([file for file in os.listdir(imu_dir) if file.endswith('.txt')])
    
    for i, (imu_file, dt) in enumerate(zip(imu_files, datatimes)):
        if imu_file is None or datatimes is None:
            continue
        
        imu_path = os.path.join(imu_dir, imu_file)
        imu_data = np.loadtxt(imu_path)
        
        imu_msg = Imu()
        imu_msg.header.frame_id = imu_frame_id
        imu_msg.header.stamp    = rospy.Time.from_sec(float(dt))        
        imu_msg = read_imu(imu_msg, imu_data, data_type)

        pub_imu.publish(imu_msg)        
        if i < len(datatimes)-1:
            next_dt = datatimes[i+1]
            wait_time = next_dt - dt
            rospy.sleep(wait_time)
    
def main():
    parser = argparse.ArgumentParser(description="Dataset Convertor")
    parser.add_argument('--dataset',       type=str,  default='kitti',     help='Dataset name') 
    parser.add_argument('--dataset_type',  type=str,  default='odom',      help='Dataset name')             # odom  or raw
    parser.add_argument('--dataset_num',   type=str,  default='00',        help='Dataset number')   # 00    or 0926_0001
    parser.add_argument('--method',        type=str,  default='aloam',     help='Odometry method')         # aloam or lio-sam    or fast-lio
    
    args = parser.parse_args()
    
    rospy.init_node('bin_data_publisher')
    
    imu_topic      = None
    lidar_topic    = None
    lidar_frame_id = None
    imu_frame_id   = None
    
    if args.dataset == 'kitti':
        lidar_topic    = '/points_raw'
        lidar_frame_id = 'camera_init'

    if args.dataset == 'kitti'and args.dataset_type == 'raw':
        imu_topic    = '/imu'
        imu_frame_id = 'imu_link'
        
    pub_bin = rospy.Publisher(lidar_topic, PointCloud2, queue_size=100)
    pub_clock = rospy.Publisher('/clock', Clock, queue_size=10)
    # pub_imu = rospy.Publisher(imu_topic,   Imu,         queue_size=100)
 
    if args.dataset == 'kitti' and args.dataset_type == 'odom':
        bin_dir  = "/home/storage1/" + args.dataset + "/" + args.dataset_type + "/" + args.dataset_num + "/bin/"
        times_file = "/home/storage1/" + args.dataset + "/" + args.dataset_type + "/" + args.dataset_num + "/times.txt"        
        datatimes    = np.loadtxt(times_file)

        publish_bin_data(pub_bin, pub_clock, args.dataset,lidar_frame_id, bin_dir, datatimes)
        
    if args.dataset == 'kitti' and args.dataset_type == 'raw':
        bin_dir  = "/home/storage1/" + args.dataset + "/" + args.dataset_type + "/" + args.dataset_num + "/bin/"
        imu_dir  = "/home/storage1/" + args.dataset + "/" + args.dataset_type + "/" + args.dataset_num + "/oxts/data/"
        time_file = imu_path = "/home/storage1/" + args.dataset + "/" + args.dataset_type + "/" + args.dataset_num + "/oxts/timestamps.txt"    
        with open(time_file, 'r') as f:
            lines = f.readlines()
            datatimes = []
            for line in lines:
                if len(line) == 1:
                    continue
                dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
                dt_sf = float(datetime.strftime(dt, "%S.%f"))
                datatimes.append(dt_sf)
        datatimes = np.array(datatimes)
         
        publish_bin_data(pub_bin, args.dataset, lidar_frame_id, bin_dir,  datatimes)
        # publish_imu_data(pub_imu, args.dataset, imu_frame_id,   imu_dir,  datatimes)
    
if __name__ == "__main__":
    main()
