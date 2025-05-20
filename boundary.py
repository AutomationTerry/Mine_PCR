from pclpy import pcl
import time

# ----------------------------- 加载点云 ----------------------------
print("->正在加载点云...")
cloud = pcl.PointCloud.PointXYZ()
pcl.io.loadPCDFile(r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled.pcd", cloud)
if cloud.size() == 0:
    print("->点云文件不存在！")
    exit(-1)
print("->共加载 {} 个数据点".format(cloud.size()))
# ==================================================================

# ----------------------------- 点云投影 ----------------------------
print("->正在水平面投影...")
for point in cloud.points:
    point.z = -1.0
# ==================================================================

# -------------------------- 投影边界下采样 --------------------------
# cloud_sub = pcl.PointCloud.PointXYZ()
# vg = pcl.filters.VoxelGrid.PointXYZ()
# vg.setInputCloud(cloud)
# vg.setLeafSize(0.05, 0.05, 0.05)
# vg.filter(cloud_sub)

# ConcaveHull 提取边界
cloud_boundary = pcl.PointCloud.PointXYZ()
ch = pcl.surface.ConcaveHull.PointXYZ()
ch.setInputCloud(cloud)
ch.setAlpha(1)
ch.reconstruct(cloud_boundary)

# 保存边界点云
output_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all_downsampled_boundary.pcd"
pcl.io.savePCDFile(output_path, cloud_boundary)
print(f"->边界点云已保存到: {output_path}")

# 可视化
if cloud_boundary.size() > 0:
    print("->共提取到 {} 个边界点".format(cloud_boundary.size()))
    print("->正在显示边界点云...")
    
    viewer = pcl.visualization.PCLVisualizer("3D Viewer")
    viewer.setBackgroundColor(1.0, 1.0, 1.0)  # 背景设为白色
    viewer.addPointCloud(cloud_boundary, "boundary_cloud")
    viewer.setPointCloudRenderingProperties(
        pcl.visualization.PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "boundary_cloud"
    )
    viewer.setPointCloudRenderingProperties(
        pcl.visualization.PCL_VISUALIZER_POINT_SIZE, 3, "boundary_cloud"
    )

    # 添加坐标系
    # viewer.addCoordinateSystem(1.0)
    # viewer.initCameraParameters()

    # 保持窗口打开直到关闭
    while not viewer.wasStopped():
        viewer.spinOnce(100)
        time.sleep(0.1)

    viewer.close()
else:
    print("边界点云为空！")