# Using Simulation Data with Semantic Mapping Pipeline

This guide explains how to use OpenAI Gym simulation environments with the semantic mapping pipeline originally designed for Record3D data.

## Overview

The semantic mapping pipeline can now work with simulation data from OpenAI Gym environments. Instead of creating Record3D files, you can use the `SimulationDataset` class to directly process simulation data.

## Data Format

Your simulation data should be organized as follows:

```
simulation_data/
├── metadata.json          # Camera parameters and settings
├── rgb/                   # RGB images
│   ├── 000000.jpg
│   ├── 000001.jpg
│   └── ...
├── depth/                 # Depth images
│   ├── 000000.npy         # Numpy arrays or PNG images
│   ├── 000001.npy
│   └── ...
└── poses/                 # Camera poses (optional)
    ├── 000000.npy         # 4x4 transformation matrices
    ├── 000001.npy
    └── ...
```

### Metadata Format

The `metadata.json` file should contain camera intrinsic parameters:

```json
{
  "image_width": 640,
  "image_height": 480,
  "fx": 525.0,
  "fy": 525.0,
  "cx": 320.0,
  "cy": 240.0
}
```

Or alternatively, you can provide the full camera matrix:

```json
{
  "image_width": 640,
  "image_height": 480,
  "camera_matrix": [
    [525.0, 0.0, 320.0],
    [0.0, 525.0, 240.0],
    [0.0, 0.0, 1.0]
  ]
}
```

## Usage Example

### 1. Collect Data from Gym Environment

```python
import gym
import numpy as np
from pathlib import Path
from PIL import Image

def collect_gym_data(env_name, output_path):
    env = gym.make(env_name)
    
    # Create directories
    data_path = Path(output_path)
    (data_path / "rgb").mkdir(parents=True, exist_ok=True)
    (data_path / "depth").mkdir(parents=True, exist_ok=True)
    (data_path / "poses").mkdir(parents=True, exist_ok=True)
    
    frame_idx = 0
    obs = env.reset()
    
    for step in range(100):  # Collect 100 frames
        # Get RGB image from environment
        rgb_image = env.render(mode='rgb_array')
        if rgb_image is not None:
            Image.fromarray(rgb_image).save(data_path / "rgb" / f"{frame_idx:06d}.jpg")
        
        # Get depth image (environment-specific)
        # This is a placeholder - you'll need to implement depth extraction
        # based on your specific gym environment
        depth_image = get_depth_from_env(env)  # Your implementation
        np.save(data_path / "depth" / f"{frame_idx:06d}.npy", depth_image)
        
        # Get camera pose (optional)
        camera_pose = get_camera_pose_from_env(env)  # Your implementation
        np.save(data_path / "poses" / f"{frame_idx:06d}.npy", camera_pose)
        
        # Take action
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        frame_idx += 1
        
        if done:
            break
    
    # Save metadata
    metadata = {
        "image_width": rgb_image.shape[1],
        "image_height": rgb_image.shape[0],
        "fx": 525.0, "fy": 525.0,
        "cx": rgb_image.shape[1] / 2,
        "cy": rgb_image.shape[0] / 2
    }
    
    with open(data_path / "metadata.json", 'w') as f:
        json.dump(metadata, f)
    
    env.close()
    return data_path
```

### 2. Create Semantic Map

```python
from dataloaders.simulation_dataset import SimulationDataset
from dataloaders.owl_voxel_map import OWLViTLabelledDataset

# Load simulation data
sim_dataset = SimulationDataset(
    data_path="./simulation_data",
    subsample_freq=2,  # Use every 2nd frame
)

# Create semantic labels
owl_dataset = OWLViTLabelledDataset(
    view_dataset=sim_dataset,
    device="cuda",
    threshold=0.1,
    visualize_results=True,
    visualization_path="./visualizations"
)

# Access the labeled point cloud
labeled_points = owl_dataset._label_xyz.cpu().numpy()
point_colors = owl_dataset._label_rgb.cpu().numpy()
semantic_features = owl_dataset._image_features.cpu().numpy()
confidence_scores = owl_dataset._label_weight.cpu().numpy()
```

## Environment-Specific Adaptations

### For Robotics Environments (PyBullet, MuJoCo, etc.)

```python
# Example for PyBullet environment
def get_depth_from_pybullet(env):
    # Get camera parameters
    width, height = 640, 480
    fov = 60
    aspect = width / height
    near = 0.02
    far = 10
    
    # Get camera position and orientation from robot
    camera_pos, camera_orn = get_robot_camera_pose(env)
    
    # Create view and projection matrices
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_pos,
        cameraTargetPosition=[camera_pos[0], camera_pos[1], camera_pos[2] - 1],
        cameraUpVector=[0, 0, 1]
    )
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    
    # Render
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        width, height, view_matrix, projection_matrix
    )
    
    # Convert depth buffer to actual depth values
    depth_array = np.array(depth_img, dtype=np.float32)
    depth_array = far * near / (far - (far - near) * depth_array)
    
    return depth_array

def get_robot_camera_pose(env):
    # Get robot's current pose
    robot_pos, robot_orn = p.getBasePositionAndOrientation(env.robot_id)
    
    # Apply camera offset (camera mounted on robot)
    camera_offset = [0, 0, 0.5]  # 0.5m above robot base
    camera_pos = [robot_pos[0] + camera_offset[0],
                  robot_pos[1] + camera_offset[1], 
                  robot_pos[2] + camera_offset[2]]
    
    return camera_pos, robot_orn
```

### For Custom Environments

If you have a custom gym environment, you'll need to implement:

1. **RGB rendering**: Use `env.render(mode='rgb_array')`
2. **Depth extraction**: Implement based on your simulation (ray casting, depth buffer, etc.)
3. **Camera pose tracking**: Track robot/camera position and orientation
4. **Camera calibration**: Provide accurate intrinsic parameters

## Differences from Record3D

| Feature | Record3D | Simulation |
|---------|----------|------------|
| **Confidence maps** | Available (LiDAR confidence) | Not available (all depth assumed valid) |
| **Depth compression** | LZ4 compressed | Raw numpy arrays or PNG |
| **Data format** | ZIP archive | Directory structure |
| **Depth filtering** | Based on confidence scores | Based on depth value ranges |
| **Pose accuracy** | High (iPhone tracking) | Depends on simulation accuracy |

## Tips for Best Results

1. **Ensure accurate camera calibration**: Wrong intrinsics will lead to poor 3D reconstruction
2. **Use realistic depth ranges**: Set appropriate min/max depth thresholds
3. **Maintain consistent camera poses**: Accurate robot pose tracking is crucial
4. **Consider lighting conditions**: OWL-ViT performance depends on good RGB image quality
5. **Subsample for performance**: Use `subsample_freq` to reduce computational load

## Troubleshooting

### Common Issues

1. **"Metadata file not found"**: Ensure `metadata.json` exists in the data directory
2. **"RGB and depth file counts don't match"**: Check that both directories have the same number of files
3. **Poor segmentation results**: Verify depth values are in meters, not millimeters
4. **Memory errors**: Reduce `subsample_prob` or increase `subsample_freq`

### Performance Optimization

1. Use `subsample_freq=N` to process every Nth frame
2. Reduce `subsample_prob` to sample fewer points per detection
3. Use smaller image resolutions if computational resources are limited
4. Consider using smaller SAM models (`vit_b` instead of `vit_h`)

## Complete Example

See `examples/gym_simulation_example.py` for a complete working example that demonstrates the entire pipeline from data collection to semantic map creation.