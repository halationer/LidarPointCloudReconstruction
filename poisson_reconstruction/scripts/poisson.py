#!/usr/bin/env python3
import numpy as np
import open3d as o3d

def buffer_to_pointcloud(buffer, compute_normals=False):
    pcd = o3d.geometry.PointCloud()
    for cloud in buffer:
        pcd += cloud
    if compute_normals:
        pcd.estimate_normals()

    return pcd


def run_poisson(pcd, depth, n_threads, min_density=None):
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, n_threads=n_threads
    )

    # Post-process the mesh
    if min_density:

        vertices_to_remove = densities < np.quantile(densities, min_density)
        mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh.compute_vertex_normals()

    return mesh, densities


def create_mesh_from_map(buffer, depth, n_threads, min_density=None):
    pcd = buffer_to_pointcloud(buffer)
    return run_poisson(pcd, depth, n_threads, min_density)


def possion(
    point_cloud_file_name = "/home/vcc/Dense_ROS/Map_PCNormal_FBack.ply", 
    output_mesh_file_name = "/home/vcc/Dense_ROS/PoissonMesh.ply",
    octree_depth = 10,
    min_density = 0.03
):

    cloud_map = o3d.geometry.PointCloud()
    source = o3d.io.read_point_cloud(point_cloud_file_name)
    cloud_map += source

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    mesh, _ = run_poisson(cloud_map, octree_depth, -1, min_density)

    # change color to white
    num_vertices = np.asarray(mesh.vertices).shape[0]       # number of points
    color = np.ones((num_vertices,3))                       # 3D vector array
    mesh.vertex_colors = o3d.utility.Vector3dVector(color) 

    o3d.io.write_triangle_mesh(output_mesh_file_name, mesh)
    return mesh

output_mesh_id = 0
def possion(
    o3dcloud,
    octree_depth = 10,
    min_density = 0.03,
    output_file = False,
    output_mesh_file_name = "/home/vcc/Dense_ROS/PoissonMesh"
):
    # '''
    cloud_map = o3d.geometry.PointCloud()
    cloud_map += o3dcloud

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    mesh, _ = run_poisson(cloud_map, octree_depth, -1, min_density)

    # change color to white
    num_vertices = np.asarray(mesh.vertices).shape[0]       # number of points
    color = np.ones((num_vertices,3))                       # 3D vector array
    mesh.vertex_colors = o3d.utility.Vector3dVector(color) 

    if(output_file):
        global output_mesh_id
        o3d.io.write_triangle_mesh(output_mesh_file_name + str(output_mesh_id) + ".ply", mesh) # output file
        output_mesh_id = output_mesh_id + 1
    # '''
    
    # mesh = o3d.io.read_triangle_mesh("/home/vcc/Dense_ROS/PoissonMesh6.ply")
    
    return mesh