#!/usr/bin/env python3
import open3d as o3d
import torch
import numpy as np

# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775134583_411764992.pt") # 40000 front distant high 2026402 1: rotated
pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135733_593223936.pt") # 40000 front distant high 2026402 2: horizontal
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775135987_676135936.pt") # 40000 front distant high 2026402 3: vertical
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1775136907_450576896.pt") # 40000 front distant high 2026402 4: vertical pose2
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
center = np.asarray(obb.center)   # in base_link
rot = np.asarray(obb.R)           # box orientation in base_link

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
