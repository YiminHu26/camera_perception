#!/usr/bin/env python3
import open3d as o3d
import torch
import numpy as np


'''
This denoising function applies both statistical outlier removal and radius outlier removal to the input point cloud.
- nb_neighbors: The number of neighbors to analyze for each point in the statistical outlier removal step. 
                [The higher, the more aggressive the denoising.]
- std_ratio:    The standard deviation ratio for the statistical outlier removal. 
                Points that are farther than this ratio times the standard deviation from the mean distance to their neighbors will be considered outliers and removed.
                [The smaller, the more aggressive the denoising.]
- radius:       The radius for the radius outlier removal. 
                Points that have fewer than min_points neighbors within this radius will be considered outliers and removed.
                [The smaller, the more aggressive the denoising.]
- min_points:   The minimum number of neighbors within the specified radius for a point to be considered an inlier. 
                [The higher, the more aggressive the denoising.]
'''
def denoise_point_cloud(
    points_np: np.ndarray,
    nb_neighbors: int = 30, 
    std_ratio: float = 1.0,
    radius: float = 0.03,
    min_points: int = 12,
) -> tuple[np.ndarray, o3d.geometry.PointCloud]:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)

    pcd_stat, stat_inliers = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    pcd_radius, radius_inliers = pcd_stat.remove_radius_outlier(
        nb_points=min_points,
        radius=radius,
    )

    print(
        f"Denoise: raw={len(points_np)}, "
        f"after_stat={len(stat_inliers)}, after_radius={len(radius_inliers)}"
    )
    return np.asarray(pcd_radius.points), pcd_radius


def compute_obb_pose_with_world_z(
    obb: o3d.geometry.OrientedBoundingBox,
    points_np: np.ndarray,
    world_z: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> tuple[np.ndarray, np.ndarray]:
    center = np.asarray(obb.center)
    rot = np.asarray(obb.R)
    extent = np.asarray(obb.extent)

    axis_order = np.argsort(extent)[::-1]
    world_z = world_z / np.linalg.norm(world_z)

    x_axis = None
    for axis_idx in axis_order:
        candidate = rot[:, axis_idx]
        candidate = candidate - np.dot(candidate, world_z) * world_z
        candidate_norm = np.linalg.norm(candidate)
        if candidate_norm > 1e-8:
            x_axis = candidate / candidate_norm
            break

    if x_axis is None:
        x_axis = np.array([1.0, 0.0, 0.0])

    centered_points = points_np - center
    points_along_x = centered_points @ x_axis
    span_along_x = np.max(np.abs(points_along_x))

    if span_along_x > 1e-8:
        end_band_threshold = 0.6 * span_along_x
        pos_end_points = points_np[points_along_x >= end_band_threshold]
        neg_end_points = points_np[points_along_x <= -end_band_threshold]

        if len(pos_end_points) > 0 and len(neg_end_points) > 0:
            pos_mean_height = np.mean(pos_end_points[:, 2])
            neg_mean_height = np.mean(neg_end_points[:, 2])
            if neg_mean_height > pos_mean_height:
                x_axis = -x_axis

    y_axis = np.cross(world_z, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = world_z

    constrained_rot = np.column_stack((x_axis, y_axis, z_axis))
    return center, constrained_rot


# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775134583_411764992.pt") # 40000 front distant high 2026402 1: rotated
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135733_593223936.pt") # 40000 front distant high 2026402 2: horizontal
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135987_676135936.pt") # 40000 front distant high 2026402 3: vertical
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136907_450576896.pt") # 40000 front distant high 2026402 4: vertical pose2
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136991_594053120.pt") # 40000 front distant high 2026402 5: rotated pose2
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775749844_421216000.pt") # 40000 front distant foam high 20260409 1: rotated pose1
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775749900_943611904.pt") # 40000 front distant foam high 20260409 2: vertical pose1
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750015_554792960.pt") # 40000 front distant foam high 20260409 3: rotated pose2
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750069_704584960.pt") # 40000 front distant foam high 20260409 4: rotated pose3
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750106_360350976.pt") # 40000 front distant foam high 20260409 5: rotated pose4
pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750147_858706944.pt") # 40000 front distant foam high 20260409 6: horizontal pose1

print(pcd_base_torch.shape)

pcd_bounds_base=torch.tensor([[-1.2, -0.5, -0.399], [-0.2, 0.5, 0.601]], dtype=torch.float32) # 40000 front distant high 20260402
pcd_shift_base = (pcd_bounds_base[0] + pcd_bounds_base[1]) / 2 
pcd_resize_base = pcd_bounds_base[1] - pcd_bounds_base[0]

pcd_base_processed_torch = (pcd_base_torch.view(-1, 3) - pcd_shift_base) / pcd_resize_base

# pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.15) & (pcd_base_processed_torch[:, 0] < 0.5) &
#                                          (pcd_base_processed_torch[:, 1] > -0.2) & (pcd_base_processed_torch[:, 1] < 0.3) &
#                                          (pcd_base_processed_torch[:, 2] > 0.0) & (pcd_base_processed_torch[:, 2] < 0.3)] # 40000 front high distant 20260402, with bounds

pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.15) & (pcd_base_processed_torch[:, 0] < 0.5) &
                                         (pcd_base_processed_torch[:, 1] > -0.2) & (pcd_base_processed_torch[:, 1] < 0.3) &
                                         (pcd_base_processed_torch[:, 2] > 0.03) & (pcd_base_processed_torch[:, 2] < 0.3)] # 40000 front high distant foam 20260409, with bounds

pcd_base_numpy = pcd_base_crop.numpy().reshape(-1, 3) # After processing with shift and resize
pcd_base_numpy, pcd_o3d_base = denoise_point_cloud(
    pcd_base_numpy,
    nb_neighbors=30,
    std_ratio=1.0,
    radius=0.03,
    min_points=12,
)
# pcd_o3d_base = o3d.geometry.PointCloud()
# pcd_o3d_base.points = o3d.utility.Vector3dVector(pcd_base_numpy)


# Create an Open3D PointCloud object
# The point cloud above has already been denoised and converted to Open3D.

# Create a coordinate frame for better orientation in the visualization
axis_agv = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
axis_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[-0.7, 0, 0])

# =====================================
# Open3D 0.19.0 does not expose t.geometry PCA/MINIMAL_* enums.
# Build OBBs with legacy APIs that are available across versions.
obbs = []

obb_specs = [
    ("PCA", o3d.geometry.OrientedBoundingBox.create_from_points, (1, 0, 0)),
]

# if hasattr(o3d.geometry.OrientedBoundingBox, "create_from_points_minimal"):
#     obb_specs.append(
#         (
#             "MINIMAL",
#             o3d.geometry.OrientedBoundingBox.create_from_points_minimal,
#             (0, 1, 0),
#         )
#     )


# for method_name, create_fn, color in obb_specs:
#     obb = create_fn(o3d.utility.Vector3dVector(pcd_base_numpy), robust = False)
#     obb.color = color
#     obbs.append(obb)
#     print(f"Volume is {obb.volume()} with {method_name}")

obb = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pcd_base_numpy), robust = False)
obb.color = (1, 0, 0)
obbs.append(obb)
print(f"Volume is {obb.volume()} with PCA")

# bb_axis_aligned = pcd_o3d_base.get_axis_aligned_bounding_box()
# bb_axis_aligned.color = (0, 0, 1)
# obbs.append(bb_axis_aligned) # Aligned BB, not usable

# # bb_min_obb = pcd_o3d_base.get_minimal_oriented_bounding_box()
# # bb_min_obb.color = (0, 1, 1)
# # obbs.append(bb_min_obb) # Same as create_from_points_minimal, but much faster and more stable.

# bb_obb = pcd_o3d_base.get_oriented_bounding_box()
# bb_obb.color = (1, 0, 1)
# obbs.append(bb_obb) 
# # # Same as create_from_points, but much faster and more stable. 
# # # However, it may not be the minimal volume box, 
# # # and the orientation may be less intuitive than PCA-based OBB.


# # o3d.visualization.draw_geometries([pcd_o3d_base, obb, axis_agv, axis_base])
# o3d.visualization.draw_geometries([pcd_o3d_base, obb, axis_agv, axis_base])
# ==================================================

# Visualization of the OBB center and orientation (axis_pose)
center, rot = compute_obb_pose_with_world_z(obb, pcd_base_numpy)  # in agv/base-aligned world frame
# center = np.asarray(bb_obb.center)   # in base_link
# rot = np.asarray(bb_obb.R)           # box orientation in base_link

obb_center_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
obb_center_marker.paint_uniform_color((0.5, 0.5, 1.0))  # Purple-ish
obb_center_marker.translate(center)
obb_pose = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
obb_pose.rotate(rot, center=(0, 0, 0))
obb_pose.translate(center)
print(f"Bounding Box center: {center}")

# Center of gravity (centroid) of the point cloud
cog = np.mean(np.asarray(pcd_o3d_base.points), axis=0)
print(f"Point-cloud center of gravity: {cog}")

# Visual marker for center of gravity
cog_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
cog_marker.paint_uniform_color((0.0, 0.0, 0.0))  # BLACK
cog_marker.translate(cog)

axis_cog = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
axis_cog.rotate(rot, center=(0, 0, 0))
axis_cog.translate(cog)

# **
# cog shows better results than obb center

o3d.visualization.draw_geometries(obbs + [pcd_o3d_base, axis_agv, axis_base, obb_center_marker, obb_pose, cog_marker, axis_cog])
