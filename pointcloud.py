

import numpy as np
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2, PointField

import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
# import tf2_geometry_msgs

class PCLWriter():

    def __init__(self, pcl_topic):

        self.point_cloud = None
        rospy.init_node('pcl_listener', anonymous=True)
        self.point_cloud_subscriber = rospy.Subscriber(pcl_topic, PointCloud2, self.pcl_callback)
        

    def pcl_callback(self, pcl):
        
        self.point_cloud = pcl

    def see_object(self, wait_for_first_pointcloud=True):

        if wait_for_first_pointcloud:

            # Do not start until a point cloud has been received

            pcl_msg = self.point_cloud

            # while object_pose_msg is None or object_pose_msg.position.z < 0.1 or object_pose_msg.position.z > 0.7:

            while pcl_msg is None:
                print("hi")
                rospy.sleep(0.1)

                pcl_msg = self.point_cloud

        return pcl_msg

class PCLReader():

    def __init__(self, pcl_filename):
        self.pcl_filename = pcl_filename
    
    def read_pcl(self):
        pcd = o3d.io.read_point_cloud(self.pcl_filename)
        return pcd

if __name__ == "__main__":

    # Write PCL

    pcl = PCLWriter("/yolov5/point_cloud2")

    xyz = np.array([[0,0,0]])
    rgb = np.array([[0,0,0]])
    #self.lock.acquire()
    gen = pc2.read_points(pcl.see_object(), skip_nans=True)
    int_data = list(gen)
    
    for x in int_data:
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
                    # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        rgb = np.append(rgb,[[r,g,b]], axis = 0)

    out_pcd = o3d.geometry.PointCloud()    
    out_pcd.points = o3d.utility.Vector3dVector(xyz)
    out_pcd.colors = o3d.utility.Vector3dVector(rgb)
    # 7 - 15 are same bottle pose different point cloud
    o3d.io.write_point_cloud("/home/portal/pointclouds/bottle_26.ply",out_pcd)

    # Read PCL

    # pcl_reader = PCLReader("/home/portal/cup.ply")
    # pcl = pcl_reader.read_pcl()
    # print(np.asarray(pcl.points).sum())