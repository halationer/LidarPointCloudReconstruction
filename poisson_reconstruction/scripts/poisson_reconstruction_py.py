#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import poisson

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker

cloud_in_topic = ""
file_outputpath = ""
mesh_out_topic = ""
mesh_out_id    = ""
poisson_depth  = 0
mesh_publisher = 0
poisson_density = 0

def MakeMarkerBase():

    global mesh_out_id

    oMeshMsgs = Marker()
    oMeshMsgs.header.frame_id = mesh_out_id
    oMeshMsgs.header.stamp = rospy.Time().now()
    oMeshMsgs.type = Marker.TRIANGLE_LIST
    oMeshMsgs.action = Marker.ADD

    oMeshMsgs.scale.x = 1.0
    oMeshMsgs.scale.y = 1.0
    oMeshMsgs.scale.z = 1.0

    oMeshMsgs.pose.position.x = 0.0
    oMeshMsgs.pose.position.y = 0.0
    oMeshMsgs.pose.position.z = 0.0

    oMeshMsgs.pose.orientation.x = 0.0
    oMeshMsgs.pose.orientation.y = 0.0
    oMeshMsgs.pose.orientation.z = 0.0
    oMeshMsgs.pose.orientation.w = 1.0

    oMeshMsgs.color.a = 1
    oMeshMsgs.color.r = 0.2
    oMeshMsgs.color.g = 1.0
    oMeshMsgs.color.b = 0.2

    return oMeshMsgs

def ReadLaunchParams():
    global cloud_in_topic 
    global file_outputpath
    global mesh_out_topic 
    global mesh_out_id    
    global poisson_depth 
    global poisson_density

    cloud_in_topic  = rospy.get_param("~cloud_in_topic",   "/processed_clouds")
    file_outputpath = rospy.get_param("~file_outputpath",  "./")
    mesh_out_topic  = rospy.get_param("~mesh_out_topic",   "/poisson_mesh")
    mesh_out_id     = rospy.get_param("~mesh_out_id",      "map")
    poisson_depth   = rospy.get_param("~poisson_depth",    10)
    poisson_density = rospy.get_param("~poisson_density",  0.03)

frame_id_received = 0
def HandlePointClouds(vProcessedCloud = PointCloud2()):
    
    global frame_id_received
    frame_id_received = frame_id_received + 1
    # if(frame_id_received % 3):
    #     return 

    # input cloud process(take long time)
    print("processing: ", vProcessedCloud.header.seq, ", ", vProcessedCloud.header.stamp.secs)
    points = point_cloud2.read_points_list(vProcessedCloud, field_names=("x", "y", "z"))
    points = np.array(points)
    print(points.shape)
    normals = point_cloud2.read_points_list(vProcessedCloud, field_names=("normal_x", "normal_y", "normal_z"))
    normals = np.array(normals)
    print(normals.shape)

    if(points.shape[0] < 10000):
        return

    # transform to o3d data
    o3d_point = o3d.geometry.PointCloud()
    o3d_point.points = o3d.utility.Vector3dVector(points)
    o3d_point.normals = o3d.utility.Vector3dVector(normals)

    # reconstruct
    global poisson_density
    global poisson_depth
    global file_outputpath
    print("------------------------density------------------------", poisson_density)
    result_mesh = o3d.geometry.TriangleMesh()
    result_mesh = poisson.possion(
        o3d_point, 
        octree_depth = poisson_depth, 
        min_density = poisson_density, 
        output_file = True,
        output_mesh_file_name = file_outputpath + "PoissonMesh")
    
    # marker and publish
    oMeshMsgs = MakeMarkerBase()
    triangles = np.asarray(result_mesh.triangles)
    print("tri:", triangles.shape)
    vertices = np.asarray(result_mesh.vertices)
    print("ver:", vertices.shape)
    all_vertices = vertices[triangles]
    print("ver_:", all_vertices.shape)
    print(all_vertices[0])

    x = all_vertices[:,:,0]
    y = all_vertices[:,:,1]
    z = all_vertices[:,:,2]

    row, col, _ = all_vertices.shape
    for i in range(row) :
        for j in range(col) :
            point = Point(x[i,j], y[i,j], z[i,j])
            oMeshMsgs.points.append(point)

    print(len(oMeshMsgs.points))
    print("finish frame processing: ", frame_id_received)

    global mesh_publisher
    mesh_publisher.publish(oMeshMsgs)
    
    # [[[x1,y1,z1][x2,y2,z2][x3,y3,z3]]   [][]...]
    # [[  obj1,      obj2,    obj3    ]   [][]...]


if __name__ == '__main__':

    rospy.init_node("poisson_reconstruction_py")

    ReadLaunchParams()

    print("File output path is: ", file_outputpath)

    rospy.Subscriber(cloud_in_topic, PointCloud2, HandlePointClouds, queue_size = 1)

    mesh_publisher = rospy.Publisher(mesh_out_topic, Marker, queue_size = 1)

    rospy.spin()