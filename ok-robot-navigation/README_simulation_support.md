# Simulation Data Support for Path Planning

This document explains how to use the modified path planning system (`path_planning.py`) with simulation data instead of Record3D data.

## Overview

The path planning system now supports two data types:
- **R3D data**: Traditional Record3D files (.r3d format) from real-world scanning
- **Simulation data**: Data collected from OpenAI Gym simulation environments

## Changes Made

### 1. New Data Type Support
- Added `SimulationDataset` class in `voxel_map/dataloaders/simulation_dataset.py`
- Added `SimulationDataset` class in `a_star/dataset_class.py` for A* planning
- Modified `path_planning.py` to auto-detect and handle both data types

### 2. Configuration Updates
- Added `data_type` parameter in configuration files
- Supports `"r3d"`, `"simulation"`, or `"auto"` (auto-detection)
- Updated comments to explain data paths for both types

### 3. Auto-Detection
- Automatically detects data type based on path structure:
  - `.r3d` files → R3D data
  - Directories with `metadata.json`, `rgb/`, `depth/` → Simulation data

## Data Format

### Simulation Data Structure
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
```json
{
  "image_width": 640,
  "image_height": 480,
  "fx": 525.0,
  "fy": 525.0,
  "cx": 320.0,
  "cy": 240.0,
  "depth_scale": 1.0
}
```

## Usage

### 1. Collect Simulation Data
Use the provided `sim_manual_scan_env.py` script to collect data from gym environments:

```bash
cd scripts
python sim_manual_scan_env.py -e "MyEnvironment-v0" --output-dir "./data"
```

This will create a timestamped directory with the required structure.

### 2. Configure for Simulation Data
Create or modify a configuration file:

```yaml
# Use simulation data
data_type: "simulation"  # or "auto" for auto-detection
dataset_path: "./data/scan_20231201_143022"  # path to simulation data directory
cache_path: 'simulation_sample.pt'  # separate cache for simulation data
```

### 3. Run Path Planning
```bash
python path_planning.py --config-name=path_simulation
```

Or with the default config if using auto-detection:
```bash
python path_planning.py dataset_path="./data/scan_20231201_143022"
```

## Example Configurations

### R3D Data (Traditional)
```yaml
data_type: "r3d"
dataset_path: "r3d/sample.r3d"
cache_path: 'sample.pt'
```

### Simulation Data
```yaml
data_type: "simulation"
dataset_path: "./data/simulation_scan"
cache_path: 'simulation_sample.pt'
```

### Auto-Detection
```yaml
data_type: "auto"
dataset_path: "./path/to/your/data"  # Can be .r3d file or simulation directory
```

## Differences from R3D Data

| Feature | R3D Data | Simulation Data |
|---------|----------|-----------------|
| **File format** | ZIP archive (.r3d) | Directory structure |
| **Depth data** | LZ4 compressed | Raw numpy arrays or PNG |
| **Confidence maps** | Available from LiDAR | Generated (depth > 0 = valid) |
| **Poses** | High accuracy from iPhone tracking | From simulation environment |
| **Calibration** | Automatic from Record3D | Manual specification required |

## Troubleshooting

### Common Issues

1. **"Metadata file not found"**
   - Ensure `metadata.json` exists in the simulation data directory
   - Check that the directory structure is correct

2. **"RGB and depth file counts don't match"**
   - Verify both `rgb/` and `depth/` directories have the same number of files
   - Files should be numbered consistently (000000.jpg, 000000.npy, etc.)

3. **Auto-detection fails**
   - Manually specify `data_type: "simulation"` in config
   - Check that required directories (`rgb/`, `depth/`, `metadata.json`) exist

4. **Poor segmentation results**
   - Verify depth values are in meters (not millimeters)
   - Check camera intrinsics are correct
   - Ensure good lighting in simulation environment

### Performance Tips

1. Use `sample_freq` to subsample frames for faster processing
2. Adjust `subsample_prob` to control point cloud density
3. Use separate cache files for different datasets
4. Consider using smaller SAM models for faster processing

## Example Workflow

1. **Collect data** using `sim_manual_scan_env.py`
2. **Verify data structure** (metadata.json, rgb/, depth/ directories)
3. **Create configuration** file or use `path_simulation.yaml`
4. **Run path planning** with simulation data
5. **Test navigation** queries in debug mode

## Integration with Existing Workflow

The simulation support is fully backward compatible. Existing R3D workflows continue to work without changes. You can:

- Mix R3D and simulation configurations
- Use auto-detection to handle both data types seamlessly
- Switch between data types by changing the `data_type` parameter

## Next Steps

- Experiment with different simulation environments
- Compare navigation performance between R3D and simulation data
- Fine-tune parameters for your specific simulation setup
- Consider combining multiple simulation scans for larger environments 