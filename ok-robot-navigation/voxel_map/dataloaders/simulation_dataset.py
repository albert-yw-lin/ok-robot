"""
Simulation Dataset for OpenAI Gym environments with robot-mounted cameras.
This provides the same interface as R3DSemanticDataset but works with simulation data.
"""

import json
import numpy as np
import open3d as o3d
import tqdm
from PIL import Image
from torch.utils.data import Dataset
from typing import List, Optional
from pathlib import Path

from dataloaders.scannet_200_classes import CLASS_LABELS_200


class SimulationSemanticDataset(Dataset):
    def __init__(
        self,
        data_path: str,
        custom_classes: Optional[List[str]] = CLASS_LABELS_200,
        subsample_freq: int = 1,
        depth_scale: float = 1000.0,
    ):
        """
        Initialize simulation dataset.
        
        Expected data structure:
        data_path/
        ├── metadata.json  # Contains camera intrinsics, poses, image dimensions
        ├── rgb/
        │   ├── 000000.jpg
        │   ├── 000001.jpg
        │   └── ...
        ├── depth/
        │   ├── 000000.npy  # or .png
        │   ├── 000001.npy
        │   └── ...
        └── poses/
            ├── 000000.npy  # 4x4 transformation matrices
            ├── 000001.npy
            └── ...
        """
        self.data_path = Path(data_path)
        self.depth_scale = depth_scale
        self.subsample_freq = subsample_freq
        
        if custom_classes:
            self._classes = CLASS_LABELS_200 + custom_classes
        else:
            self._classes = CLASS_LABELS_200
        self._classes = list(set(self._classes))
        print("The labels you use for OWL-ViT is ", str(self._classes))
        
        # Load metadata
        self._load_metadata()
        
        # Initialize storage
        self._rgb_images = []
        self._depth_images = []
        self.global_xyzs = []
        
        # Load all data
        self._load_simulation_data()
        self._calculate_global_xyzs()
        
    def _load_metadata(self):
        """Load camera parameters and poses from metadata file."""
        metadata_path = self.data_path / "metadata.json"
        
        if not metadata_path.exists():
            raise FileNotFoundError(f"Metadata file not found at {metadata_path}")
            
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)
        
        # Extract camera parameters
        self.rgb_width = metadata["image_width"]
        self.rgb_height = metadata["image_height"]
        self.image_size = (self.rgb_width, self.rgb_height)
        
        # Camera intrinsics matrix (3x3)
        if "camera_matrix" in metadata:
            self.camera_matrix = np.array(metadata["camera_matrix"])
        else:
            # If individual parameters are provided
            fx = metadata["fx"]
            fy = metadata["fy"] 
            cx = metadata["cx"]
            cy = metadata["cy"]
            self.camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])
        
        # Get number of frames
        rgb_dir = self.data_path / "rgb"
        self.total_images = len(list(rgb_dir.glob("*.jpg"))) + len(list(rgb_dir.glob("*.png")))
        
        self._id_to_name = {i: x for (i, x) in enumerate(self._classes)}
        
    def _load_simulation_data(self):
        """Load RGB images and depth data from simulation."""
        rgb_dir = self.data_path / "rgb"
        depth_dir = self.data_path / "depth"
        
        # Get sorted file lists
        rgb_files = sorted(list(rgb_dir.glob("*.jpg")) + list(rgb_dir.glob("*.png")))
        depth_files = sorted(list(depth_dir.glob("*.npy")) + list(depth_dir.glob("*.png")))
        
        assert len(rgb_files) == len(depth_files), "RGB and depth file counts don't match"
        
        for i in tqdm.tqdm(range(0, len(rgb_files), self.subsample_freq), desc="Loading simulation data"):
            # Load RGB image
            rgb_img = np.array(Image.open(rgb_files[i]))
            self._rgb_images.append(rgb_img)
            
            # Load depth image
            depth_file = depth_files[i]
            if depth_file.suffix == '.npy':
                depth_img = np.load(depth_file)
            else:  # PNG depth
                depth_img = np.array(Image.open(depth_file))
                if depth_img.dtype == np.uint16:
                    # Keep as float32, depth values are already in 0-1000 range
                    depth_img = depth_img.astype(np.float32)
                
            self._depth_images.append(depth_img)
    
    def _load_pose(self, frame_idx):
        """Load camera pose for given frame."""
        poses_dir = self.data_path / "poses"
        pose_file = poses_dir / f"{frame_idx:06d}.npy"
        
        return np.load(pose_file)
    
    def _calculate_global_xyzs(self):
        """Calculate global 3D coordinates for all frames."""
        for i in tqdm.trange(len(self._depth_images), desc="Calculating global XYZs"):
            depth_img = self._depth_images[i]
            
            # Create point cloud from depth image
            # Depth values are in range 0-1000, use them directly for Open3D
            
            depth_o3d = o3d.geometry.Image(
                np.ascontiguousarray(depth_img).astype(np.float32)
            )
            rgb_o3d = o3d.geometry.Image(
                np.ascontiguousarray(self._rgb_images[i]).astype(np.uint8)
            )
            
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, depth_o3d, convert_rgb_to_intensity=False
            )
            
            camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
                width=int(self.rgb_width),
                height=int(self.rgb_height),
                fx=self.camera_matrix[0, 0],
                fy=self.camera_matrix[1, 1],
                cx=self.camera_matrix[0, 2],
                cy=self.camera_matrix[1, 2],
            )
            
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, camera_intrinsics
            )
            # Flip the pcd
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

            # Apply camera pose transformation
            frame_idx = i * self.subsample_freq
            pose_matrix = self._load_pose(frame_idx)
            
            pcd.transform(pose_matrix)
            
            # Store global coordinates
            global_xyz = np.asarray(pcd.points)
            self.global_xyzs.append(global_xyz)
    
    def __len__(self):
        return len(self._depth_images)
    
    def __getitem__(self, idx):
        """Return data in same format as R3DSemanticDataset."""
        return {
            "xyz_position": self.global_xyzs[idx],
            "rgb": self._rgb_images[idx],
            "depth": self._depth_images[idx],
            # Note: No confidence data for simulation, we'll handle this in the valid mask
        }