#!/usr/bin/env python3
import open3d as o3d
import torch
import numpy as np

# Compatibility shim for deps that still reference np.float (removed in NumPy 1.24).
if not hasattr(np, "float"):
    np.float = float  # type: ignore[attr-defined]
from tf_transformations import quaternion_from_matrix, translation_from_matrix, quaternion_matrix

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

- Some best practices for tuning these parameters:
    - Start with a moderate nb_neighbors (e.g., 30) and std_ratio (e.g., 1.0) to remove obvious outliers without being too aggressive.
    - Adjust nb_neighbors and std_ratio based on the density and noise level of your point cloud
        - If you see too many points being removed, try increasing nb_neighbors(e.g. 40) or std_ratio (e.g. 1.5) or radius(e.g. 0.05).
        - If you still see outliers, try decreasing nb_neighbors(e.g. 20) or std_ratio (e.g. 0.8) or min_points(e.g. 16).
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


'''
This function computes a constrained OBB pose where the Z-axis is aligned with the given world Z direction.

The function first tries to find the longest axis of the OBB that is most orthogonal to the world Z direction to use as the X-axis. 
If all axes are nearly parallel to the world Z, it defaults to using the global X-axis.

Then, it checks the distribution of points along the chosen X-axis to determine if it should flip the X-axis direction for better alignment with the point cloud's geometry.
(The higher end should be more likely to have points, so if the negative end has higher mean height, we flip the X-axis).

Finally, it constructs the Y-axis as the cross product of the world Z and the chosen X-axis, and returns the center and the constrained rotation matrix.
'''
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
    y_axis_norm = np.linalg.norm(y_axis)
    if y_axis_norm <= 1e-8:
        y_axis = np.array([0.0, 1.0, 0.0])
    else:
        y_axis /= y_axis_norm
    z_axis = world_z

    constrained_rot = np.column_stack((x_axis, y_axis, z_axis))
    return center, constrained_rot


# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775134583_411764992.pt") # 40000 front distant high 2026402 1: rotated
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135733_593223936.pt") # * 40000 front distant high 2026402 2: horizontal
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135987_676135936.pt") # 40000 front distant high 2026402 3: vertical
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136907_450576896.pt") # * 40000 front distant high 2026402 4: vertical pose2
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136991_594053120.pt") # 40000 front distant high 2026402 5: rotated pose2

# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775749844_421216000.pt") # 40000 front distant foam high 20260409 1: rotated pose1
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775749900_943611904.pt") # 40000 front distant foam high 20260409 2: vertical pose1
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750015_554792960.pt") # 40000 front distant foam high 20260409 3: rotated pose2
pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750069_704584960.pt") # 40000 front distant foam high 20260409 4: rotated pose3
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750106_360350976.pt") # 40000 front distant foam high 20260409 5: rotated pose4
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775750147_858706944.pt") # 40000 front distant foam high 20260409 6: horizontal pose1

print(pcd_base_torch.shape)

pcd_bounds_base=torch.tensor([[-1.2, -0.5, -0.399], [-0.2, 0.5, 0.601]], dtype=torch.float32) # 40000 front distant high 20260402
pcd_shift_base = (pcd_bounds_base[0] + pcd_bounds_base[1]) / 2 
pcd_resize_base = pcd_bounds_base[1] - pcd_bounds_base[0]

pcd_base_processed_torch = (pcd_base_torch.view(-1, 3) - pcd_shift_base) / pcd_resize_base

# pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.15) & (pcd_base_processed_torch[:, 0] < 0.5) &
#                                          (pcd_base_processed_torch[:, 1] > -0.2) & (pcd_base_processed_torch[:, 1] < 0.2) &
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

# Create an Open3D PointCloud object
# The point cloud above has already been denoised and converted to Open3D.

# Create a coordinate frame for better orientation in the visualization
Rz_neg_90 = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.deg2rad(-90)))
axis_agv = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
axis_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[-0.7, 0, 0])

axis_agv.rotate(Rz_neg_90, center=(0, 0, 0))
axis_base.rotate(Rz_neg_90, center=(-0.7, 0, 0))

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

# # bb_axis_aligned = pcd_o3d_base.get_axis_aligned_bounding_box()
# # bb_axis_aligned.color = (0, 0, 1)
# # obbs.append(bb_axis_aligned) # Aligned BB, not usable

# # bb_min_obb = pcd_o3d_base.get_minimal_oriented_bounding_box()
# # bb_min_obb.color = (0, 1, 1)
# # obbs.append(bb_min_obb) # Same as create_from_points_minimal, but much faster and more stable.

# # bb_obb = pcd_o3d_base.get_oriented_bounding_box()
# # bb_obb.color = (1, 0, 1)
# # obbs.append(bb_obb) 
# # # Same as create_from_points, but much faster and more stable. 
# # # However, it may not be the minimal volume box, 
# # # and the orientation may be less intuitive than PCA-based OBB.


# # o3d.visualization.draw_geometries([pcd_o3d_base, obb, axis_agv, axis_base])
# o3d.visualization.draw_geometries([pcd_o3d_base, obb, axis_agv, axis_base])
# ==================================================

# Visualization of the OBB center and orientation (axis_pose)
obb_pose_center, obb_pose_rot = compute_obb_pose_with_world_z(obb, pcd_base_numpy)


obb_pose_center, obb_pose_rot = compute_obb_pose_with_world_z(
            obb,
            pcd_base_numpy,
        )

obb_pose = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
obb_pose.rotate(obb_pose_rot, center=(0, 0, 0))
obb_pose.translate(obb_pose_center)

obb_center_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
obb_center_marker.paint_uniform_color((0.5, 0.5, 1.0))  # Purple-ish
obb_center_marker.translate(obb_pose_center)
# obb_pose = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
# obb_pose.rotate(obb_pose_rot, center=(0, 0, 0))
# obb_pose.translate(obb_pose_center)
print(f"Bounding Box center: {obb_pose_center}")

# Center of gravity (centroid) of the point cloud
cog = np.mean(np.asarray(pcd_o3d_base.points), axis=0)
print(f"Point-cloud center of gravity: {cog}")

# Visual marker for center of gravity
cog_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
cog_marker.paint_uniform_color((0.0, 0.0, 0.0))  # BLACK
cog_marker.translate(cog)

axis_cog = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
axis_cog.rotate(obb_pose_rot, center=(0, 0, 0))
axis_cog.translate(cog)

placement_pose = np.array([-2.052, 0.0, 0.267])
placement_center_r, placement_center_p, placement_center_y = 0.0, np.deg2rad(45), np.deg2rad(-180)
# placement_center_r, placement_center_p, placement_center_y = 0.0, 0.0, 0.0

placement_center_rot = o3d.geometry.get_rotation_matrix_from_xyz((placement_center_r, placement_center_p, placement_center_y))
placement_center = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
placement_center.rotate(placement_center_rot, center=(0, 0, 0))
placement_center.translate(placement_pose)

# Plane dimensions
w, h, t = 0.20, 0.20, 0.002  # width, height, tiny thickness

# Create plane (thin box) centered at origin; normal is +Z in local frame
plane = o3d.geometry.TriangleMesh.create_box(width=w, height=h, depth=t)
plane.translate([-w/2, -h/2, -t/2])  # move geometric center to (0,0,0)
plane.paint_uniform_color([0.2, 0.8, 0.2])

# Align with placement_center (Z axis becomes plane normal)
plane.rotate(placement_center_rot, center=(0, 0, 0))
plane.translate(placement_pose)

'''
======================================================
vmf_input_pcd_base_1775134583_411764992.pt
[[-0.55930525  0.8235235  -0.09479824  0.00148981]
 [ 0.82883596  0.5535591  -0.08126085  0.04169851]
 [-0.01444379 -0.12402181 -0.9921744   0.02583791]
 [ 0.          0.          0.          1.        ]]
Predicted translation: [-0.69851017  0.04169851  0.12683791], quaternion: (-0.4688576731181075, -0.8810559937936515, 0.05824903781215444, 0.02280060859963066)

 NOW: ^agv_table T_grasp, ^base T_agv_table
 TODO: search for ^cog T_grasp = ^cog T_base @ ^base T_agv_table @ ^agv_table T_grasp
======================================================
'''
# grasp_pose = np.array([[-0.55930525,  0.8235235,  -0.09479824,  0.00148981],
#                        [0.82883596,  0.5535591,  -0.08126085,  0.04169851],
#                        [-0.01444379, -0.12402181, -0.9921744,   0.02583791],
#                        [0.         ,  0.         ,  0.         ,  1        ]])



'''
======================================================
vmf_input_pcd_base_1775135733_593223936.pt
[[ 0.93696624 -0.3372127  -0.09155258  0.01481934]
 [-0.32006037 -0.9333838   0.16234529  0.02828353]
 [-0.1401986  -0.1228097  -0.98247755  0.04172907]
 [ 0.          0.          0.          1.        ]]
Predicted translation: [-0.68518066  0.02828353  0.14272907], quaternion: (-0.9814310479348951, 0.16742721138914435, 0.05903399635133402, 0.07263755440995379)

======================================================
'''
grasp_pose = np.array([[ 0.93696624, -0.3372127,  -0.09155258,  0.01481934],
                       [-0.32006037, -0.9333838,   0.16234529,  0.02828353],
                       [-0.1401986,  -0.1228097,  -0.98247755,  0.04172907],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]])



'''
======================================================
vmf_input_pcd_base_1775136907_450576896.pt
[[ 0.28674173 -0.49915227 -0.8176957   0.13470241]
 [-0.8755258  -0.48301828 -0.01216848  0.03119325]
 [-0.38888803  0.71940285 -0.5755221   0.06897386]
 [ 0.          0.          0.          1.        ]]
Predicted translation: [-0.5652976   0.03119325  0.16997387], quaternion: (0.7657156881957456, -0.44882127438196817, -0.39393985599322934, 0.23885212658273647)
======================================================
'''
# grasp_pose = np.array([[ 0.28674173, -0.49915227, -0.8176957,   0.13470241],
#                        [-0.8755258,  -0.48301828, -0.01216848,  0.03119325],
#                        [-0.38888803,  0.71940285, -0.5755221,   0.06897386],
#                        [ 0.        ,  0.        ,  0.        ,  1.        ]])

# =======================================================
agv_T_grasp = grasp_pose

base_T_agv = np.array([[1.        , 0.        , 0.        ,  0.7      ],
                       [0.        , 1.        , 0.        ,  0.       ],
                       [0.        , 0.        , 1.        ,  0.101    ],
                       [0.        , 0.        , 0.        ,  1.       ]])

# NOTE:
# obb_pose_rot is ^agv R_cog (COG frame axes expressed in agv_table_center_link).

agv_T_cog = np.eye(4)
agv_T_cog[:3, :3] = obb_pose_rot
agv_T_cog[:3, 3] = cog
# print(f"agv_T_cog:\n{agv_T_cog}")

cog_T_agv = np.linalg.inv(agv_T_cog)
# print(f"cog_T_agv:\n{cog_T_agv}")

# cog_T_grasp = cog_T_base @ base_T_agv @ agv_T_grasp
cog_T_grasp = cog_T_agv @ agv_T_grasp
print(f"cog_T_grasp:\n{cog_T_grasp}")

agv_T_placement_center = np.eye(4)
agv_T_placement_center[:3, :3] = placement_center_rot
agv_T_placement_center[:3, 3] = placement_pose

# agv_T_place = agv_T_placement_center @ placement_center_T_place
#             = agv_T_placement_center @ cog_T_grasp

agv_T_place = agv_T_placement_center @ cog_T_grasp
print(f"agv_T_place:\n{agv_T_place}")

grasp_center_quat = quaternion_from_matrix(grasp_pose)
grasp_center_translation = translation_from_matrix(grasp_pose)

grasp_center_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
grasp_center_marker.paint_uniform_color((1.0, 0.5, 0.5))  # light red
grasp_center_marker.translate(grasp_center_translation)
grasp_center = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
grasp_center.rotate(quaternion_matrix(grasp_center_quat)[:3, :3], center=(0, 0, 0))
grasp_center.translate(grasp_center_translation)

place_pose_quat = quaternion_from_matrix(agv_T_place)
place_pose_translation = translation_from_matrix(agv_T_place)
# R_y_90 = o3d.geometry.get_rotation_matrix_from_xyz((0.0, np.deg2rad(90.0), 0.0))
# place_pose_rot = quaternion_matrix(place_pose_quat)[:3, :3] @ R_y_90
place_pose_rot = quaternion_matrix(place_pose_quat)[:3, :3]

place_center_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
place_center_marker.paint_uniform_color((0.5, 1.0, 0.5))  # light green
place_center_marker.translate(place_pose_translation)
place_center = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])
place_center.rotate(place_pose_rot, center=(0, 0, 0))
place_center.translate(place_pose_translation)




# **
# cog shows better results than obb center
o3d.visualization.draw_geometries(obbs + 
                                  [pcd_o3d_base, 
                                   axis_agv, 
                                   axis_base, 
                                   obb_center_marker, 
                                   obb_pose, 
                                   cog_marker, 
                                   axis_cog, 
                                   placement_center, 
                                   plane, 
                                   grasp_center,
                                   grasp_center_marker,
                                   place_center,
                                   place_center_marker,
                                ])
