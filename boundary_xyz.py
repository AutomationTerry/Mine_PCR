import open3d as o3d
import numpy as np

def match_and_update_z(source_pcd_path, boundary_pcd_path, output_pcd_path):
    # 读取原始点云和边界点云
    source_pcd = o3d.io.read_point_cloud(source_pcd_path)
    boundary_pcd = o3d.io.read_point_cloud(boundary_pcd_path)
    
    # 获取点云的 numpy 数组
    source_points = np.asarray(source_pcd.points)
    boundary_points = np.asarray(boundary_pcd.points)
    
    # 构建 KDTree 用于最近邻搜索
    source_kdtree = o3d.geometry.KDTreeFlann(source_pcd)
    
    # 遍历边界点云的每个点，找到原始点云中的最近邻点
    updated_points = boundary_points.copy()
    for i in range(len(boundary_points)):
        # 获取当前边界点的 x, y, z
        query_point = boundary_points[i]
        
        # 在原始点云中搜索最近邻点
        [k, idx, _] = source_kdtree.search_knn_vector_3d(query_point, 1)
        
        # 用原始点云的 Z 值替换边界点的 Z 值
        updated_points[i, 2] = source_points[idx[0], 2]
    
    # 创建新的点云对象
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(updated_points)
    
    # 保存点云为 ASCII 格式的 PCD 文件
    num_points = len(updated_points)
    o3d.io.write_point_cloud(output_pcd_path, new_pcd, write_ascii=True)
    
    # 手动修改 PCD 文件头部以匹配指定格式
    with open(output_pcd_path, 'r') as f:
        lines = f.readlines()
    
    # 构造新的头部
    header = [
        '# .PCD v0.7 - Point Cloud Data file format\n',
        'VERSION 0.7\n',
        'FIELDS x y z\n',
        'SIZE 4 4 4\n',
        'TYPE F F F\n',
        'COUNT 1 1 1\n',
        f'WIDTH {num_points}\n',
        'HEIGHT 1\n',
        'VIEWPOINT 0 0 0 1 0 0 0\n',
        f'POINTS {num_points}\n',
        'DATA ascii\n'
    ]
    
    # 获取点数据部分
    data_lines = [line for line in lines if not line.startswith('#') and not line.startswith(('VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA'))]
    
    # 写入新文件
    with open(output_pcd_path, 'w') as f:
        f.writelines(header + data_lines)
    
    print(f"Updated point cloud saved to {output_pcd_path} in ASCII format with specified header.")

# 文件路径
source_pcd_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all.pcd"
boundary_pcd_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled_boundary.pcd"
output_pcd_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled_boundary_xyz.pcd"

# 执行匹配和更新
match_and_update_z(source_pcd_path, boundary_pcd_path, output_pcd_path)