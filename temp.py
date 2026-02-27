import open3d as o3d
import torch
import numpy as np

# Base 1
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1771421989_182810112.pt")
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1771508271_456421888.pt")
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1771513272_419227904.pt")
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1771517855_611204096.pt")
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1771519029_725381120.pt")
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1772027191_314213888.pt") # 240000 low
# pcd_base_torch = torch.load(f="vmf_input_pcd_base_1772027502_230915072.pt") # 40000 low
pcd_base_torch = torch.load(f="vmf_input_pcd_base_1772028331_672243968.pt") # 40000 high

print(pcd_base_torch.shape)

# Base 2
# pcd_base = o3d.geometry.PointCloud()
# pcd_base.points = o3d.utility.Vector3dVector(pcd_base_torch)
# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

# # vvvvvvvvvvvvvvvvvvvvv
# o3d.visualization.draw_plotly([pcd_base, axis])
# # o3d.visualization.draw_geometries([pcd_base, axis])

# Base 3
# [[x1, y1, z1], [x2, y2, z2]] where (x1, y1, z1) is the minimum corner and (x2, y2, z2) is the maximum corner of the bounding box that we want to fit our point cloud into. 
# The point cloud will be shifted and resized to fit within this bounding box.
# x1+x2 = 2*center_x
# x2-x1 = 1.0
# pcd_bounds_base=torch.tensor([[-0.43, -1.5, -0.43], [0.57, -0.5, 0.57]], dtype=torch.float32)
# pcd_bounds_base=torch.tensor([[-0.5, -1.2, -0.6], [0.5, -0.2, 0.4]], dtype=torch.float32)
# pcd_bounds_base=torch.tensor([[-0.5, -1.25, -0.6], [0.5, -0.25, 0.4]], dtype=torch.float32)
# pcd_bounds_base=torch.tensor([[-0.5, -1.2, -0.6], [0.5, -0.2, 0.4]], dtype=torch.float32)

# pcd_bounds_base=torch.tensor([[-0.5, -1.3, -0.6], [0.5, -0.3, 0.4]], dtype=torch.float32) # 40000 & 240000
# pcd_shift_base = (pcd_bounds_base[0] + pcd_bounds_base[1]) / 2 # Shift to center, half of sum should be in the center
# pcd_resize_base = pcd_bounds_base[1] - pcd_bounds_base[0] # Should be equal to 1.0. Resize to fit in the range, difference between max and min should be the size of the range, and the point cloud will be normalized to fit in a unit cube of size 1x1x1 after this resizing.
# pcd_base_processed_torch = (pcd_base_torch.view(-1, 3) - pcd_shift_base) / pcd_resize_base

# pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.5) & (pcd_base_processed_torch[:, 0] < 0.5) &
#                                          (pcd_base_processed_torch[:, 1] > -0.5) & (pcd_base_processed_torch[:, 1] < 0.5) &
#                                          (pcd_base_processed_torch[:, 2] > -0.05) & (pcd_base_processed_torch[:, 2] < 0.5)]

# pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] >= -0.2) & (pcd_base_processed_torch[:, 0] <= 0.2) &
#                                          (pcd_base_processed_torch[:, 1] >= -0.2) & (pcd_base_processed_torch[:, 1] <= 0.2) &
#                                          (pcd_base_processed_torch[:, 2] >= -0.07) & (pcd_base_processed_torch[:, 2] <= 0.2)]

# pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.3) & (pcd_base_processed_torch[:, 0] < 0.2) &
#                                          (pcd_base_processed_torch[:, 1] > -0.3) & (pcd_base_processed_torch[:, 1] < 0.3) &
#                                          (pcd_base_processed_torch[:, 2] > -0.04) & (pcd_base_processed_torch[:, 2] < 0.5)] # 40000 & 240000 low

# pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.2) & (pcd_base_processed_torch[:, 0] < 0.2) &
#                                          (pcd_base_processed_torch[:, 1] > -0.2) & (pcd_base_processed_torch[:, 1] < 0.3) &
#                                          (pcd_base_processed_torch[:, 2] > 0.065) & (pcd_base_processed_torch[:, 2] < 0.5)] # 40000 & 240000 high

pcd_base_processed_torch = pcd_base_torch.view(-1, 3)
pcd_base_crop = pcd_base_processed_torch[(pcd_base_processed_torch[:, 0] > -0.5) & (pcd_base_processed_torch[:, 0] < 0.5) &
                                         (pcd_base_processed_torch[:, 1] > -1.5) & (pcd_base_processed_torch[:, 1] < -0.5) &
                                         (pcd_base_processed_torch[:, 2] > -0.05) & (pcd_base_processed_torch[:, 2] < 0.5)] # 40000 & 240000 high

# Convert the tensor to a NumPy array
# pcd_numpy = loaded_pcd_2.numpy().reshape(-1, 3) # original
pcd_base_numpy = pcd_base_crop.numpy().reshape(-1, 3) # After processing with shift and resize

# Create an Open3D PointCloud object
pcd_o3d_base = o3d.geometry.PointCloud()
pcd_o3d_base.points = o3d.utility.Vector3dVector(pcd_base_numpy)

# Create a coordinate frame for better orientation in the visualization
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

# Visualize the point cloud
#vvvvvvvv
# o3d.visualization.draw_plotly([pcd_o3d_base, axis])
# o3d.visualization.draw_plotly([pcd_o3d_base,])
o3d.visualization.draw_geometries([pcd_o3d_base, axis]) # Not for Jupyter, use draw_plotly instead