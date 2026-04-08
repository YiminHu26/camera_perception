#!/usr/bin/env python3
import open3d as o3d
import torch
import numpy as np

# Compatibility shim for deps that still reference np.float (removed in NumPy 1.24).
if not hasattr(np, "float"):
    np.float = float  # type: ignore[attr-defined]
from tf_transformations import quaternion_from_matrix, translation_from_matrix, quaternion_matrix





# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775134583_411764992.pt") # 40000 front distant high 2026402 1: rotated
pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135733_593223936.pt") # * 40000 front distant high 2026402 2: horizontal
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135987_676135936.pt") # 40000 front distant high 2026402 3: vertical
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136907_450576896.pt") # * 40000 front distant high 2026402 4: vertical pose2
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136991_594053120.pt") # 40000 front distant high 2026402 5: rotated pose2
print(pcd_base_torch.shape)

pcd_bounds_base=torch.tensor([[-1.2, -0.5, -0.399], [-0.2, 0.5, 0.601]], dtype=torch.float32) # 40000 front distant high 20260402
pcd_shift_base = (pcd_bounds_base[0] + pcd_bounds_base[1]) / 2 
pcd_resize_base = pcd_bounds_base[1] - pcd_bounds_base[0]

pcd_base_processed_torch = (pcd_base_torch.view(-1, 3) - pcd_shift_base) / pcd_resize_base

pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.15) & (pcd_base_processed_torch[:, 0] < 0.5) &
                                         (pcd_base_processed_torch[:, 1] > -0.2) & (pcd_base_processed_torch[:, 1] < 0.25) &
                                         (pcd_base_processed_torch[:, 2] > 0.0) & (pcd_base_processed_torch[:, 2] < 0.3)] # 40000 front high distant 20260402, with bounds

pcd_base_numpy = pcd_base_crop.numpy().reshape(-1, 3) # After processing with shift and resize

# Create an Open3D PointCloud object
pcd_o3d_base = o3d.geometry.PointCloud()
pcd_o3d_base.points = o3d.utility.Vector3dVector(pcd_base_numpy)

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
obb_pose_center = np.asarray(obb.center)   # in base_link
obb_pose_rot = np.asarray(obb.R)           # box orientation in base_link

obb_center_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
obb_center_marker.paint_uniform_color((0.5, 0.5, 1.0))  # Purple-ish
obb_center_marker.translate(obb_pose_center)
obb_pose = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
obb_pose.rotate(obb_pose_rot, center=(0, 0, 0))
obb_pose.translate(obb_pose_center)
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
                                #    obb_center_marker, 
                                #    obb_pose, 
                                   cog_marker, 
                                   axis_cog, 
                                   placement_center, 
                                   plane, 
                                   grasp_center,
                                   grasp_center_marker,
                                   place_center,
                                   place_center_marker,])
