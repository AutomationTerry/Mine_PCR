import open3d as o3d
import numpy as np

# 定义降采样函数
def downsample_point_cloud(pcd, voxel_size=0.1, target_points=100):
    """使用体素网格和均匀采样对点云进行降采样"""
    # 获取点云的边界框
    min_bound = pcd.get_min_bound()
    
    # 将点分配到体素中
    voxel_dict = {}
    for point in pcd.points:
        idx_x = int((point[0] - min_bound[0]) / voxel_size)
        idx_y = int((point[1] - min_bound[1]) / voxel_size)
        idx_z = int((point[2] - min_bound[2]) / voxel_size)
        voxel_index = (idx_x, idx_y, idx_z)
        if voxel_index not in voxel_dict:
            voxel_dict[voxel_index] = []
        voxel_dict[voxel_index].append(point)
    
    # 处理每个体素：如果点数 > target_points 则降采样，否则保留所有点
    new_points = []
    for points in voxel_dict.values():
        points_array = np.array(points)
        if len(points_array) > target_points:
            indices = np.random.choice(len(points_array), target_points, replace=False)
            new_points.extend(points_array[indices])
        else:
            new_points.extend(points_array)
    
    # 创建新的点云对象
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(new_points)
    return new_pcd

# 读取点云文件
pcd = o3d.io.read_point_cloud(r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all.pcd")

# 打印原始点云的点数
print(f"原始点云有 {len(pcd.points)} 个点。")

# 打印降采样参数
voxel_size = 1
target_points = 3
print(f"降采样参数：voxel_size = {voxel_size}, target_points = {target_points}")

# 降采样点云
downsampled_pcd = downsample_point_cloud(pcd, voxel_size=voxel_size, target_points=target_points)

# 打印降采样后点云的点数
print(f"降采样后点云有 {len(downsampled_pcd.points)} 个点。")

# 保存降采样后的点云
o3d.io.write_point_cloud(r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled.pcd", downsampled_pcd)

print("降采样完成")