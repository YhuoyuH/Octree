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


def f_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1
            print(
                "{}{}: 内部节点在深度 {} 有 {} 个子节点和 {} 个点（起点 {}）"
                .format('    ' * node_info.depth,
                        node_info.child_index, node_info.depth, n,
                        len(node.indices), node_info.origin))

            # 我们只想处理有足够多点的节点/空间区域
            early_stop = len(node.indices) < 250
    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            print("{}{}: 叶子节点在深度 {} 有 {} 个点，起点为 {}".
                  format('    ' * node_info.depth, node_info.child_index,
                         node_info.depth, len(node.indices), node_info.origin))
    else:
        raise NotImplementedError('未识别的节点类型！')

    # 提前停止遍历：如果为真，则跳过当前节点的子节点遍历
    return early_stop

 
# 创建八叉树， 树深为4
octree = o3d.geometry.Octree(max_depth=4)
# 从点云中创建体素网格， 体素大小为0.01m
octree.convert_from_point_cloud(pcd, size_expand=0.01)
# 遍历
octree.traverse(f_traverse)

print(octree.locate_leaf_node(pcd.points[1952]))