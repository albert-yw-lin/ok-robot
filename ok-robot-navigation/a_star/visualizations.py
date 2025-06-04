import math

import open3d as o3d
import numpy as np
import os

def create_dashed_cylinder_line(points, radius=0.03, dash_length=0.07, gap_length=0.03, color=[0, 0, 1]):  # Default color red
    geometries = []
    for i in range(len(points) - 1):
        start_point = points[i]
        end_point = points[i + 1]
        vec = end_point - start_point
        seg_length = np.linalg.norm(vec)
        vec_normalized = vec / seg_length
        n_dashes = math.ceil(seg_length / (dash_length + gap_length))

        for j in range(n_dashes):
            new_dash_length = min(dash_length, seg_length - (j)*(dash_length + gap_length))
            dash_start = start_point + vec_normalized * j * (new_dash_length + gap_length)
            dash_end = dash_start + vec_normalized * new_dash_length
            cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=new_dash_length)
            cylinder.translate((dash_start + dash_end)/2)

            z_axis = np.array([0, 0, 1])
            rotation_axis = np.cross(z_axis, vec)
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            rotation_angle = np.pi/2
            rotation_matrix = cylinder.get_rotation_matrix_from_axis_angle(rotation_axis*rotation_angle)
            cylinder.rotate(rotation_matrix, center=dash_start)

            cylinder.paint_uniform_color(color)
            geometries.append(cylinder)
    
    return geometries

def add_arrows_to_line(line_set, arrow_length=0.2, cylinder_radius=0.02, cone_radius=0.05):
    arrows = []

    for line in line_set.lines:
        start_idx, end_idx = line
        start_point = np.asarray(line_set.points[start_idx])
        end_point = np.asarray(line_set.points[end_idx])
        cylinder, cone = create_arrow_geometry(start_point, end_point, arrow_length, cylinder_radius, cone_radius)
        arrows.append(cylinder)
        arrows.append(cone)

    return arrows

def create_arrow_geometry(start_point, end_point, arrow_length=0.2, cylinder_radius=0.02, cone_radius=0.05):
    vec = end_point - start_point
    vec_norm = np.linalg.norm(vec)
    arrow_vec = vec / vec_norm * arrow_length

    # Cylinder (arrow's body)
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=cylinder_radius, height=arrow_length)
    cylinder.translate(start_point)
    cylinder.rotate(cylinder.get_rotation_matrix_from_xyz([np.arccos(arrow_vec[2] / np.linalg.norm(arrow_vec)), 0, np.arctan2(arrow_vec[1], arrow_vec[0])]), center=start_point)

    # Cone (arrow's head)
    cone = o3d.geometry.TriangleMesh.create_cone(radius=cone_radius, height=arrow_length)
    cone.translate(start_point + vec - arrow_vec)
    cone.rotate(cone.get_rotation_matrix_from_xyz([np.arccos(arrow_vec[2] / np.linalg.norm(arrow_vec)), 0, np.arctan2(arrow_vec[1], arrow_vec[0])]), center=start_point + vec - arrow_vec)
    
    return cylinder, cone

def visualize_path(path, end_xyz, cfg):

    
    if not os.path.exists(cfg.pointcloud_path):
        print(f'\nNo {cfg.pointcloud_path} found, creating a new one.\n')
        from a_star.data_util import get_pointcloud, get_posed_rgbd_dataset
        get_pointcloud(get_posed_rgbd_dataset(key = 'r3d', path = cfg.dataset_path), cfg.pointcloud_path)
        print(f'\n{cfg.pointcloud_path} created.\n')

    # Example point cloud and path points (replace with your data)
    point_cloud = o3d.io.read_point_cloud(cfg.pointcloud_path)

    if path is not None:
        path = np.array(np.array(path).tolist())
        print(path)
        start_point = path[0, :]
    end_point = np.array(end_xyz.numpy())

    if path is not None:
        path[:, 2] = cfg.min_height
        lines = create_dashed_cylinder_line(path)
    end_point[2] = (cfg.min_height + cfg.max_height)/2

    # Create spheres for start and end points
    if path is not None:
        start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)

    # Set the position of the spheres
    if path is not None:
        start_sphere.translate(start_point)
    end_sphere.translate(end_point)

    # Set different colors for clarity
    # lines.paint_uniform_color([1, 0, 0])  # Red path
    if path is not None:
        start_sphere.paint_uniform_color([0, 1, 0])  # Green start
    end_sphere.paint_uniform_color([1, 0, 0])  # Blue end

    # Visualize
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(visible=True)
    if path is not None:
        geometries = [point_cloud, *lines, start_sphere, end_sphere]
    else:
        geometries = [point_cloud, end_sphere]
    visualizer.poll_events()
    visualizer.update_renderer()
    for geometry in geometries:
        visualizer.add_geometry(geometry)
    visualizer.run()
    visualizer.destroy_window()

# def save_visualization_as_ply(path, end_xyz, cfg, save_dir=None):
#     """
#     Save the navigation visualization as PLY files for external viewing.
#     Creates separate PLY files for point cloud, path, and markers that can be
#     opened in MeshLab, CloudCompare, Blender, etc.
    
#     Args:
#         path: Path waypoints (None for localization only)
#         end_xyz: Target location coordinates
#         cfg: Configuration object
#         save_dir: Directory to save PLY files (default: cfg.save_file/A/)
    
#     Returns:
#         List of saved file paths
#     """
    
#     if not os.path.exists(cfg.pointcloud_path):
#         print(f'\nNo {cfg.pointcloud_path} found, creating a new one.\n')
#         from a_star.data_util import get_pointcloud, get_posed_rgbd_dataset
#         get_pointcloud(get_posed_rgbd_dataset(key = 'r3d', path = cfg.dataset_path), cfg.pointcloud_path)
#         print(f'\n{cfg.pointcloud_path} created.\n')

#     # Set up save directory
#     if save_dir is None:
#         save_dir = f"{cfg.save_file}/ply_exports"
#     os.makedirs(save_dir, exist_ok=True)
    
#     saved_files = []
    
#     # Skip saving separate environment point cloud - we'll include it in the combined scene
#     # 1. Load point cloud for combining with markers
#     point_cloud = o3d.io.read_point_cloud(cfg.pointcloud_path)
    
#     # 2. Create and save target marker (red sphere)
#     end_point = np.array(end_xyz.numpy())
#     end_point[2] = (cfg.min_height + cfg.max_height)/2
    
#     end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
#     end_sphere.translate(end_point)
#     end_sphere.paint_uniform_color([1, 0, 0])  # Red target
    
#     target_path = f"{save_dir}/target_marker.ply"
#     o3d.io.write_triangle_mesh(target_path, end_sphere)
#     saved_files.append(target_path)
#     print(f"Target marker saved: {target_path}")
    
#     # 3. Save path and start marker if path exists
#     if path is not None:
#         path_array = np.array(np.array(path).tolist())
#         start_point = path_array[0, :]
        
#         # Create start marker (green sphere)
#         start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
#         start_sphere.translate(start_point)
#         start_sphere.paint_uniform_color([0, 1, 0])  # Green start
        
#         start_path = f"{save_dir}/start_marker.ply"
#         o3d.io.write_triangle_mesh(start_path, start_sphere)
#         saved_files.append(start_path)
#         print(f"Start marker saved: {start_path}")
        
#         # Create path visualization (blue cylinders)
#         path_array[:, 2] = cfg.min_height
#         path_geometries = create_dashed_cylinder_line(path_array)
        
#         # Combine all path geometries into one mesh
#         combined_path = o3d.geometry.TriangleMesh()
#         for geom in path_geometries:
#             combined_path += geom
        
#         path_viz_path = f"{save_dir}/navigation_path.ply"
#         o3d.io.write_triangle_mesh(path_viz_path, combined_path)
#         saved_files.append(path_viz_path)
#         print(f"Navigation path saved: {path_viz_path}")
        
#         # Create combined scene file
#         combined_scene = point_cloud
#         # Add markers as point clouds for the combined file
#         target_points = np.asarray(end_sphere.vertices) + end_point
#         start_points = np.asarray(start_sphere.vertices) + start_point
        
#         # Create point clouds from sphere vertices for combined scene
#         target_pcd = o3d.geometry.PointCloud()
#         target_pcd.points = o3d.utility.Vector3dVector(target_points)
#         target_pcd.paint_uniform_color([1, 0, 0])
        
#         start_pcd = o3d.geometry.PointCloud()
#         start_pcd.points = o3d.utility.Vector3dVector(start_points)
#         start_pcd.paint_uniform_color([0, 1, 0])
        
#         # Combine everything
#         combined_scene += target_pcd
#         combined_scene += start_pcd
        
#         combined_path = f"{save_dir}/complete_scene.ply"
#         o3d.io.write_point_cloud(combined_path, combined_scene)
#         saved_files.append(combined_path)
#         print(f"Complete scene saved: {combined_path}")
        
#     else:
#         # No path - save target only scene
#         target_points = np.asarray(end_sphere.vertices) + end_point
#         target_pcd = o3d.geometry.PointCloud()
#         target_pcd.points = o3d.utility.Vector3dVector(target_points)
#         target_pcd.paint_uniform_color([1, 0, 0])
        
#         localization_scene = point_cloud + target_pcd
#         localization_path = f"{save_dir}/localization_scene.ply"
#         o3d.io.write_point_cloud(localization_path, localization_scene)
#         saved_files.append(localization_path)
#         print(f"Localization scene saved: {localization_path}")
    
#     # Create instruction file
#     instructions_path = f"{save_dir}/README.txt"
#     with open(instructions_path, 'w') as f:
#         f.write("Navigation Visualization PLY Files\n")
#         f.write("==================================\n\n")
#         f.write("These files can be opened in:\n")
#         f.write("- MeshLab (free): https://www.meshlab.net/\n")
#         f.write("- CloudCompare (free): https://www.cloudcompare.org/\n")
#         f.write("- Blender (free): https://www.blender.org/\n\n")
#         f.write("File descriptions:\n")
#         f.write("- target_marker.ply: Red sphere showing target location\n")
#         if path is not None:
#             f.write("- start_marker.ply: Green sphere showing start position\n")
#             f.write("- navigation_path.ply: Blue path from start to target\n")
#             f.write("- complete_scene.ply: Environment + markers + path combined\n")
#             f.write("\nRecommended: Open complete_scene.ply for full navigation view\n")
#         else:
#             f.write("- localization_scene.ply: Environment + target combined\n")
#             f.write("\nRecommended: Open localization_scene.ply for full view\n")
#     saved_files.append(instructions_path)
    
#     print(f"\n✅ All PLY files saved to: {save_dir}")
#     print(f"📖 Instructions saved to: {instructions_path}")
#     return saved_files
