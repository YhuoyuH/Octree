# -*- coding: UTF-8 -*-
import numpy as np
import open3d as o3d
import sys
sys.path.append("E:\\Project\\Octree\\Open3D\\examples\\Python\\")
import open3d_tutorial as o3dtut

# 加载点云，并采样2000个点
N = 2000
pcd = o3dtut.get_armadillo_mesh().sample_points_poisson_disk(N)
# 点云归一化
pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()),
          center=pcd.get_center())
# 点云着色
pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
# 可视化
o3d.visualization.draw_geometries([pcd])

 
# 创建八叉树， 树深为4
octree = o3d.geometry.Octree(max_depth=4)
# 从点云中构建八叉树，适当扩展边界0.01m
octree.convert_from_point_cloud(pcd, size_expand=0.01)
# 可视化
o3d.visualization.draw_geometries([octree])


# 从点云中创建体素网格， 体素大小为0.05m
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)
# 体素可视化
o3d.visualization.draw_geometries([voxel_grid])
 
# 创建八叉树， 树深为4
octree = o3d.geometry.Octree(max_depth=4)
# 从体素网格中构建八叉树
octree.create_from_voxel_grid(voxel_grid)
# 可视化
o3d.visualization.draw_geometries([octree])