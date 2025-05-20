import open3d as o3d
import numpy as np
from sklearn.neighbors import KDTree

def IterativeClosestPoint(source_pts, target_pts, init_transform, tau=1e-6):
    '''
    This function implements iterative closest point algorithm with an initial transformation.
    
    inputs:
    source_pts : 3 x N
    target_pts : 3 x M
    init_transform : 4 x 4 initial transformation matrix
    tau : threshold for convergence

    outputs:
    R : Rotation Matrix (3 x 3)
    t : translation vector (3 x 1)
    k : num_iterations
    '''
    print("Starting ICP refinement")
    k = 0
    # Apply initial transformation to source points
    current_pts = ApplyTransformation(source_pts, init_transform[:3, :3], init_transform[:3, 3].reshape(3, 1))
    last_rmse = float('inf')  # Initialize with infinity for first iteration
    total_R = init_transform[:3, :3].copy()  # Accumulate total rotation
    total_t = init_transform[:3, 3].reshape(3, 1).copy()  # Accumulate total translation

    while True:
        neigh_pts = FindNeighborPoints(current_pts, target_pts)
        (R, t) = RegisterPoints(current_pts, neigh_pts)
        current_pts = ApplyTransformation(current_pts, R, t)
        rmse = ComputeRMSE(current_pts, neigh_pts)
        print(f"ICP iteration {k}, RMSE: {rmse}")

        # Accumulate transformations
        total_R = np.dot(R, total_R)
        total_t = np.dot(R, total_t) + t

        if np.abs(rmse - last_rmse) < tau:
            print("ICP converged")
            break
        last_rmse = rmse
        k += 1

    return (total_R, total_t, k)

def ComputeRMSE(p1, p2):
    return np.sum(np.sqrt(np.sum((p1-p2)**2, axis=0)))

def ApplyTransformation(pts, R, t):
    return np.dot(R, pts) + t

def RegisterPoints(p1, p2):
    u1 = np.mean(p1, axis=1).reshape((3, 1))
    u2 = np.mean(p2, axis=1).reshape((3, 1))
    pp1 = p1 - u1
    pp2 = p2 - u2
    W = np.dot(pp1, pp2.T)
    U, _, Vh = np.linalg.svd(W)
    R = np.dot(U, Vh).T
    if np.linalg.det(R) < 0:
        Vh[2, :] *= -1
        R = np.dot(U, Vh).T
    t = u2 - np.dot(R, u1)
    return (R, t)

def FindNeighborPoints(source, target):
    n = source.shape[1]
    kdt = KDTree(target.T, leaf_size=60, metric='euclidean')
    index = kdt.query(source.T, k=1, return_distance=False).reshape((n,))
    return target[:, index]

def main(use_coarse=True, use_fine=True):
    if not use_coarse and not use_fine:
        print("Error: At least one of use_coarse or use_fine must be True")
        return

    # File paths
    source_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\dataset\2025-04-25-10-46-05.pcd"
    target_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\dataset\2025-04-25-10-31-43.pcd"
    output_path = r"D:\25_Tianrui_DR\2025_jinchuan\Mine_PCR\output\21_all_25_all.pcd"

    print("Step 1: Loading point clouds")
    source_pcd = o3d.io.read_point_cloud(source_path)
    target_pcd = o3d.io.read_point_cloud(target_path)

    print("Step 2: Downsampling point clouds")
    source_pcd = source_pcd.voxel_down_sample(voxel_size=0.5)
    target_pcd = target_pcd.voxel_down_sample(voxel_size=0.5)

    if use_coarse:
        print("Step 3: Computing FPFH features for coarse registration")
        radius_normal = 1.0
        source_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        target_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        radius_feature = 5.0
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target_pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

        print("Step 4: Performing coarse registration with RANSAC")
        distance_threshold = 1.5
        result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_pcd, target_pcd, source_fpfh, target_fpfh, mutual_filter=False,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                      o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
        )
        init_transform = result_ransac.transformation
        print("coarse registration matrix:\n",init_transform)

        print("Step 5: Visualizing coarse registration result")
        source_temp = source_pcd.transform(init_transform)
        source_temp.paint_uniform_color([0, 0, 1])  # Blue for transformed source
        target_pcd.paint_uniform_color([1, 0, 0])   # Red for target
        o3d.visualization.draw_geometries([source_temp, target_pcd],
                                          window_name="Coarse Registration Result",
                                          width=800,
                                          height=600)
        merged_pcd = o3d.geometry.PointCloud()
        merged_pcd.points = o3d.utility.Vector3dVector(
            np.vstack((np.asarray(source_temp.points), np.asarray(target_pcd.points)))
        )
        o3d.io.write_point_cloud(output_path, merged_pcd)
    else:
        init_transform = np.eye(4)

    if use_fine:
        print("Step 6: Converting point clouds to numpy arrays")
        source_pts = np.asarray(source_pcd.points).T
        target_pts = np.asarray(target_pcd.points).T

        print("Step 7: Running ICP with initial transformation")
        R, t, num_iterations = IterativeClosestPoint(source_pts, target_pts, init_transform, tau=1e-6)

        # Construct final transformation
        final_transform = np.eye(4)
        final_transform[:3, :3] = R
        final_transform[:3, 3] = t.flatten()
    else:
        final_transform = init_transform
        R = init_transform[:3, :3]
        t = init_transform[:3, 3].reshape(3, 1)
        num_iterations = 0

    print("Step 8: Applying final transformation")
    transformed_pcd = source_pcd.transform(final_transform)

    print("Step 9: Visualizing final registration result")
    transformed_pcd.paint_uniform_color([0, 0, 1])
    target_pcd.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([transformed_pcd, target_pcd],
    #                                   window_name="Final Registration Result",
    #                                   width=800,
    #                                   height=600)

    print("Step 10: Saving merged point cloud")
    # merged_pcd = o3d.geometry.PointCloud()
    # merged_pcd.points = o3d.utility.Vector3dVector(
    #     np.vstack((np.asarray(transformed_pcd.points), np.asarray(target_pcd.points)))
    # )
    # o3d.io.write_point_cloud(output_path, merged_pcd)

    print("Registration completed")
    if use_fine:
        print(f"ICP completed in {num_iterations} iterations")
    print("Final Rotation Matrix:\n", R)
    print("Final Translation Vector:\n", t)

if __name__ == "__main__":
    main(use_coarse=True, use_fine=False)